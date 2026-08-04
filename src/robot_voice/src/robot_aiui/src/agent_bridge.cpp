#include "hpp/agent_bridge.h"
#include <iostream>
#include <sstream>
#include <thread>
#include <atomic>
#include <curl/curl.h>     // 新增 curl 头文件用来发送 HTTP 请求
#include <std_msgs/String.h>
#include <ros/ros.h>


// ===== 外部函数声明（定义在 robot_aiui.cpp 全局作用域中）=====
extern void startTTS(const std::string &text, const std::string &tag = "");
extern bool navigateTo(double x, double y, double theta);
extern void callYoloPick(int mode);
extern void callYoloPlace(int mode);
extern void startRecordAudio();
extern void stopRecordAudio();
extern void triggerCaptureServiceAgent();
extern bool locateObjectSyncAgent(const std::string& target, std::string& locateMsg);
extern std::string getLatestImageDescription();  // 封装 DemoListener::getInstance().getLatestImageDescription()
extern void publishTargetObject(const std::string& target); // 发布检测目标到 /target_object 话题
extern void getLast3DLocation(double &x, double &y, double &z); // 获取最近一次3D坐标   

namespace agent {

// 项目 jsoncpp 在 aiui_va::Json 命名空间下，简化书写
using JsonValue = aiui_va::Json::Value;
using JsonReader = aiui_va::Json::Reader;

// ===== 全局：多轮循环防护 =====
static std::atomic<bool> g_agentRunning{false};
static constexpr int MAX_LLM_ROUNDS = 8;      // 最多几轮 LLM 交互
static constexpr int MAX_SKILL_FAILURES = 3;   // 连续失败上限

// ===== SkillRegistry 实现 =====
void SkillRegistry::registerSkill(const SkillDef &skill) {
    skills_[skill.name] = skill;
}

const SkillDef* SkillRegistry::find(const std::string &name) const {
    auto it = skills_.find(name);
    return (it != skills_.end()) ? &it->second : nullptr;
}

std::vector<SkillDef> SkillRegistry::listAll() const {
    std::vector<SkillDef> result;
    for (const auto &kv : skills_) result.push_back(kv.second);
    return result;
}

std::string SkillRegistry::describeAll() const {
    std::ostringstream oss;
    oss << "你有以下能力（skills），请选择最合适的一个：\n";
    for (const auto &kv : skills_) {
        oss << "  - " << kv.second.name << ": " << kv.second.description << "\n";
    }
    return oss.str();
}

// ===== 初始化所有 Skill =====
void initSkills(SkillRegistry &reg) {
    // --- skill::speak ---
    reg.registerSkill({
        "speak",
        "通过语音播报文字内容给用户。参数: {\"text\": \"播报的文字\"}",
        JsonValue(),
        [](const JsonValue &params) -> SkillResult {
            std::string text = params.get("text", "").asString();
            if (text.empty()) return {false, "speak: text 参数为空", {}};
            startTTS(text);
            return {true, "播报成功: " + text, {}};
        }
    });

    // --- skill::navigate_to ---
    reg.registerSkill({
        "navigate_to",
        "导航到指定坐标。参数: {\"x\": 0.0, \"y\": 0.0, \"theta\": 0.0}",
        JsonValue(),
        [](const JsonValue &params) -> SkillResult {
            double x = params.get("x", 0.0).asDouble();
            double y = params.get("y", 0.0).asDouble();
            double theta = params.get("theta", 0.0).asDouble();
            if (navigateTo(x, y, theta))
                return {true, "导航成功 (" + std::to_string(x) + "," + std::to_string(y) + ")", {}};
            else
                return {false, "导航失败", {}};
        }
    });

    // --- skill::pick ---
    reg.registerSkill({
        "pick",
        "抓取物体。参数: {\"mode\": 1}  (1=apple,2=orange,3=pear,4=peach,5=bottle)",
        JsonValue(),
        [](const JsonValue &params) -> SkillResult {
            int mode = params.get("mode", 1).asInt();
            callYoloPick(mode);
            return {true, "抓取指令已发送, mode=" + std::to_string(mode), {}};
        }
    });

    // --- skill::place ---
    reg.registerSkill({
        "place",
        "放置物体。参数: {\"mode\": 1}",
        JsonValue(),
        [](const JsonValue &params) -> SkillResult {
            int mode = params.get("mode", 1).asInt();
            callYoloPlace(mode);
            return {true, "放置指令已发送, mode=" + std::to_string(mode), {}};
        }
    });

    // --- skill::take_photo ---
    reg.registerSkill({
        "take_photo",
        "拍摄当前画面并获取图像描述。无参数。",
        JsonValue(),
        [](const JsonValue &/*params*/) -> SkillResult {
            triggerCaptureServiceAgent();
            // 阻塞等待视觉识别结果（内部用 ros::spinOnce() 轮询 /image_description 话题）
            std::string description = getLatestImageDescription();   // 这还有问题
            if (description.empty()) {
                startTTS("视觉识别超时，请重试");
                return {false, "视觉识别超时", {}};
            }
            std::cout << "[agent] 获取到的图像描述: " << description << std::endl;
            startTTS("好的，我看到了: " + description);
            return {true, "拍照完成", {}};
        }
    });

    // --- skill::locate_object（合并：设置目标 → 触发定位 → 返回3D坐标）---
    reg.registerSkill({
        "locate_object",
        "定位目标物体并返回3D坐标。参数: {\"target\": \"apple\"}  举例：apple/orange/pear/peach/bottle，或者其他的英文名称",
        JsonValue(),
        [](const JsonValue &params) -> SkillResult {
            std::string target = params.get("target", "").asString();
            if (target.empty()) return {false, "locate_object: target 参数为空", {}};

            // 1. 发布检测目标到 /target_object 话题
            publishTargetObject(target);

            // 2. 触发定位（同步阻塞，等待服务端返回）
            std::string locateMsg;
            if (!locateObjectSyncAgent(target, locateMsg)) {
                return {false, "定位失败: " + locateMsg, {}};
            }

            // 3. 等待 /object_position 回调更新坐标（防止 service 先于 topic 返回）
            ros::Duration(0.3).sleep();
            ros::spinOnce();

            // 4. 获取最近一次 3D 坐标
            double x, y, z;
            getLast3DLocation(x, y, z);

            // 5. 校验坐标合法性（全0说明回调没触发，定位实际未完成）
            if (x == 0.0 && y == 0.0 && z == 0.0) {
                return {false, "定位失败: 未能获取到 " + target + " 的有效3D坐标（可能目标不在视野内或不支持该物体），请尝试其他目标", {}};
            }

            std::ostringstream oss;
            oss << "已定位到 " << target
                << "，坐标: (" << x << ", " << y << ", " << z << ")";
            return {true, oss.str(), {}};
        }
    });

    std::cout << "[agent] 已注册 " << reg.listAll().size() << " 个 skill" << std::endl;
}

// ===== libcurl 回调函数，用于接收 HTTP 响应数据 =====
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    size_t totalSize = size * nmemb;
    ((std::string*)userp)->append((char*)contents, totalSize);
    return totalSize;
}

// ===== LLM 调用（真实 HTTP 请求）=====
static std::string callLLM(const std::string &systemPrompt,
                           const std::string &userMessage,
                           const std::string &context) {
    // 替换为你的大模型实际配置 (这里以适配 OpenAI 接口标准的大模型为例，如 DeepSeek, Qwen)
    std::string LLM_API_KEY  = "sk-f474d800269a47b4a18538a855ced131";
    std::string LLM_BASE_URL = "https://api.deepseek.com/chat/completions"; 
    std::string LLM_MODEL    = "deepseek-v4-flash"; // 或 "deepseek-chat", "qwen-max"
    
    std::cout << "[agent] ===== 发起 LLM CALL 请求 =====\n";

    // 1. 构建 JSON 请求体 (使用项目内的 jsoncpp)
    JsonValue root;
    root["model"] = LLM_MODEL;
    // (可选) 让模型强行返回 JSON，但为了兼容普通文字，不要强制
    // root["response_format"]["type"] = "json_object"; 

    JsonValue messages(aiui_va::Json::arrayValue);
    
    JsonValue sysMsg;
    sysMsg["role"] = "system";
    sysMsg["content"] = systemPrompt + "\n[历史执行上下文]:\n" + context;
    messages.append(sysMsg);
    
    JsonValue usrMsg;
    usrMsg["role"] = "user";
    usrMsg["content"] = userMessage;
    messages.append(usrMsg);
    
    root["messages"] = messages;

    aiui_va::Json::FastWriter writer;
    std::string requestBody = writer.write(root);

    // 2. 初始化 cURL
    CURL* curl = curl_easy_init();
    std::string readBuffer;
    if (curl) {
        struct curl_slist *headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        std::string authHeader = "Authorization: Bearer " + LLM_API_KEY;
        headers = curl_slist_append(headers, authHeader.c_str());
        
        curl_easy_setopt(curl, CURLOPT_URL, LLM_BASE_URL.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, requestBody.c_str());
        
        // 接收响应
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        // SSL/TLS 配置（忽略证书校验错误，以防部分本地环境缺少CA证书）
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
        
        CURLcode res = curl_easy_perform(curl);
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
        
        if (res != CURLE_OK) {
            std::cerr << "[agent] curl 请求失败: " << curl_easy_strerror(res) << "\n";
            return "{\"skill\":\"speak\",\"params\":{\"text\":\"网络请求失败，请检查网络和 API 配置\"}}";
        }
    } else {
        return "{\"skill\":\"speak\",\"params\":{\"text\":\"无法初始化网络客户端\"}}";
    }

    // 3. 解析大模型返回的 JSON 结果
    JsonReader reader;
    JsonValue respJson;
    if (reader.parse(readBuffer, respJson, false) && respJson.isObject()) {
        if (respJson.isMember("choices") && respJson["choices"].size() > 0) {
            std::string content = respJson["choices"][0]["message"]["content"].asString();
            std::cout << "[agent] 大模型返回: " << content << std::endl;
            return content;
        } else if (respJson.isMember("error")) {
            std::cerr << "[agent] 大模型 API 报错: " << respJson["error"].toStyledString() << "\n";
            return "{\"skill\":\"speak\",\"params\":{\"text\":\"大模型请求发生错误，请检查秘钥或额度\"}}";
        }
    }
    
    std::cerr << "[agent] 返回报文解析失败: " << readBuffer << std::endl;
    return "{\"skill\":\"speak\",\"params\":{\"text\":\"我不理解返回的数据格式\"}}";
}

// ===== 解析 LLM 返回：尝试提取 skill 调用 =====
static bool tryParseSkillCall(const std::string &llmOutput,
                              std::string &skillName,
                              JsonValue &params) {
    JsonReader reader;
    JsonValue root;
    // 尝试解析 JSON
    if (reader.parse(llmOutput, root, false) && root.isObject()) {
        if (root.isMember("skill") && root.isMember("params")) {
            skillName = root["skill"].asString();
            params = root["params"];
            return true;
        }
    }
    // 不是 JSON → 当作纯文本播报
    skillName = "speak";
    params["text"] = llmOutput;
    return true;  // 纯文本也算"解析成功"，走 speak skill
}
void visionDescriptionCallback(const std_msgs::String::ConstPtr& msg)
{
  // 在这里处理接收到的消息
  // msg 是一个指向消息常量数据的智能指针
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  
  // 你可以在这里添加更多逻辑，比如解析数据、控制机器人等
}

// ===== [核心] Agent 多轮循环 =====
static bool runAgentLoop(const std::string &userInput) {
    SkillRegistry reg;
    initSkills(reg);

    std::string systemPrompt =
    "# 角色\n"
    "你是一个机器人助手，通过调用 Skill（技能）来控制机器人并与物理世界交互。\n\n"
    
    "# 运行规则\n"
    "1. 仔细阅读下方提供的【候选 Skill 列表】。\n"
    "2. 根据用户输入，判断是需要执行动作还是进行普通交流：\n"
    "   - **普通交流/闲聊**：直接回复文本内容，不要使用 JSON 格式。\n"
    "   - **执行动作**：必须且只能输出标准 JSON 格式，不能包含任何前导词或解释性文本。\n"
    "3. **JSON 格式要求**：\n"
    "   {\n"
    "     \"skill\": \"技能名\",\n"
    "     \"params\": { ... } // 必须严格匹配 Skill 定义中的参数名和数据类型\n"
    "   }\n"
    "4. 严禁在 JSON 外包裹 Markdown 标记（如 ```json ... ```）。\n\n"
    
    "# 候选 Skill 列表\n" + reg.describeAll();

    std::string context;  // 累积上下文（每一轮的结果）
    int totalRounds = 0;
    int consecutiveFailures = 0;

    // 第1轮：把用户输入发给 LLM
    std::string llmOutput = callLLM(systemPrompt, userInput, context);
    totalRounds++;

    while (totalRounds <= MAX_LLM_ROUNDS) {
        std::string skillName;
        JsonValue params;

        if (!tryParseSkillCall(llmOutput, skillName, params)) {
            // LLM 返回格式不可解析，兜底播报
            startTTS("抱歉，我无法理解这个指令");
            return true;
        }

        // 查找 skill
        const SkillDef *skill = reg.find(skillName);
        if (!skill) {
            context += "\n[系统] 未知 skill: " + skillName + "，请重试";
            llmOutput = callLLM(systemPrompt, "上一步出错了，请换个方式", context);
            totalRounds++;
            consecutiveFailures++;
            if (consecutiveFailures >= MAX_SKILL_FAILURES) {
                startTTS("抱歉，操作失败次数过多，请重新试试");
                return true;
            }
            continue;
        }

        // 执行 skill
        std::cout << "[agent] 执行 skill: " << skillName
                  << " params=" << params.toString() << std::endl;
        SkillResult result = skill->execute(params);

        if (result.success) {
            consecutiveFailures = 0;
            context += "\n[系统] skill [" + skillName + "] 执行成功: " + result.message;
        } else {
            consecutiveFailures++;
            context += "\n[系统] skill [" + skillName + "] 执行失败: " + result.message;
        }

        // 把结果喂回 LLM，看它是否要继续
        llmOutput = callLLM(systemPrompt,
                            "上一步已执行完成，还需要我做什么？",
                            context);
        totalRounds++;

        // 如果 LLM 返回纯文本（speak skill），播报后结束
        if (skillName == "speak" || llmOutput.find("\"skill\"") == std::string::npos) {
            // 纯文本回复 → 直接播报 ← 已经在上面 skill 执行时播报了
            // 但 LLM 可能返回新的 speak，再走一轮判断
            if (llmOutput.find("\"skill\"") == std::string::npos) {
                // 没有任何 skill 调用，视为对话结束
                break;
            }
        }
    }

    if (totalRounds > MAX_LLM_ROUNDS) {
        startTTS("任务步骤较多，我先停下了");
    }

    return true;
}

// ===== Agent 入口 =====
bool handleText(const std::string &userText) {
    if (userText.empty()) return false;

    std::cout << "=======================[agent] received: "
              << userText << " ============================================\n";

    // 防止并发
    if (g_agentRunning.exchange(true)) {
        std::cout << "[agent] 上一个任务还在执行中，跳过\n";
        return false;  // fallthrough 到旧逻辑
    }

    // 异步执行，不阻塞 AIUI 回调线程
    std::thread([](std::string text) {
        runAgentLoop(text);
        g_agentRunning = false;
        startRecordAudio();  // 重新开始录音
    }, userText).detach();

    return true;  // Agent 已接管
}

} 
