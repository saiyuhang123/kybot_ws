#ifdef WIN32
#include <windows.h>

#define _HAS_STD_BYTE 0
#define AIUI_SLEEP Sleep
#else
#include <unistd.h>

#define AIUI_SLEEP(x) usleep(x * 1000)
#endif

#undef AIUI_LIB_COMPILING

#include <cstring>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include "aiui/AIUI_V2.h"
#include "aiui/PcmPlayer_C.h"
#include "json/json.h"
#include "utils/StreamNlpTtsHelper.h"
#include "utils/IatResultUtil.h"
#include "utils/Base64Util.h"
#include "std_msgs/String.h"
#include "robot_aiui/NavigateToOffice.h" // 导入自定义服务类型
#include "geometry_msgs/Pose2D.h"
#include <kybot_msgs/pick_and_place.h>
#include <std_srvs/Trigger.h>
#include <thread>
#include <atomic>
#include "hpp/agent_bridge.h"
#include <regex>
#include <map>
#include <memory>
#include <exception>
#include <xmlrpcpp/XmlRpcValue.h>
#include <cmath>
#include <vector>
#include "hpp/agent_bridge.h"
#include <geometry_msgs/PoseStamped.h>

// 是否使用AIUI V2服务（交互大模型）
#define AIUI_V2

// 是否使用语义后合成。当在AIUI平台应用配置页面打开"语音合成"开关时，需要打开该宏
// #define USE_POST_SEMANTIC_TTS

using namespace std;
using namespace aiui_va;
using namespace aiui_v2;

/*********************播放回调函数************************/
void onStarted()
{
    cout << "PcmPlayer, onStarted" << endl;
}

void onPaused()
{
    cout << "PcmPlayer, onPaused" << endl;
}

void onResumed()
{
    cout << "PcmPlayer, onResumed" << endl;
}

void onStopped()
{
    cout << "PcmPlayer, onStopped" << endl;
}

void onError(int error, const char *des)
{
    cout << "PcmPlayer, onError, error=" << error << ", des=" << des << endl;
}

void onProgress(int streamId, int progress, const char *audio, int len, bool isCompleted)
{
    //    cout << "PcmPlayer, onProgress, streamId=" << streamId << ", progress=" << progress
    //         << ", len=" << len << ", isCompleted=" << isCompleted << endl;
}

void startTTS(const string &text, const string &tag = "");
void startRecordAudio(); // 在顶部声明
void stopRecordAudio();  // 也是声明
string gSyncSid;
string gVoiceCloneResId;
ros::Publisher image_pub;
enum TaskState
{
    IDLE,
    NAVIGATING,
    PICKING,
    PLACE,
    NAVIGATING0
};
TaskState currentState = IDLE; // 状态管理

// 新增：发布图片路径的函数
void publishImagePath()
{
    ROS_INFO("发布图片路径");

    std_msgs::String img_msg;
    img_msg.data = "/home/sss/qwen_vision/src/qwen_vision/picture/2.jpg";
    image_pub.publish(img_msg);

    ROS_INFO("已发布图片地址: %s", img_msg.data.c_str());
}
// 抓取服务调用函数
void callYoloPick(int mode)
{
    ros::NodeHandle nh;
    ros::ServiceClient pickClient;
    pickClient = nh.serviceClient<kybot_msgs::pick_and_place>("/yolo_pick");
    // 确保服务已连接
    if (!pickClient.exists())
    {
        ROS_WARN("YOLO pick service not available!");
        return;
    }

    // 创建服务请求
    kybot_msgs::pick_and_place srv;
    srv.request.mode = mode; // 根据实际服务类型修改

    // 调用服务
    if (pickClient.call(srv))
    {
        cout << "抓取指令已发送，模式: " << mode << endl;
        if (currentState == PICKING)
        {
            currentState = NAVIGATING0; // 更新状态为导航
                                        // navigateTo(2.833, 7.567, -1.539); // 回到原点
        }
    }
    else
    {
        cout << "抓取请求失败" << endl;
        startTTS("操作失败，请重试");
    }
}
// 放置服务调用函数
void callYoloPlace(int mode)
{
    ros::NodeHandle nh;
    ros::ServiceClient placeClient;
    placeClient = nh.serviceClient<kybot_msgs::pick_and_place>("/yolo_place");
    // 确保服务已连接
    if (!placeClient.exists())
    {
        ROS_WARN("YOLO place service not available!");
        return;
    }

    // 创建服务请求
    kybot_msgs::pick_and_place srv;
    srv.request.mode = mode; // 根据实际服务类型修改

    // 调用服务
    if (placeClient.call(srv))
    {
        cout << "放置指令已发送，模式: " << mode << endl;
    }
    else
    {
        cout << "放置请求失败" << endl;
        startTTS("操作失败，请重试");
    }
}
// 通用导航函数，接受坐标信息
bool navigateTo(double x, double y, double theta)
{
    ros::NodeHandle n;
    ros::ServiceClient navGoalClient =
        n.serviceClient<robot_aiui::NavigateToOffice>("/kybot_navgoali");

    robot_aiui::NavigateToOffice srv;
    geometry_msgs::Pose2D pose;
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
    srv.request.pose = pose;

    if (!navGoalClient.exists())
    {
        ROS_WARN("Nav service /kybot_navgoali not available!");
        return false;
    }

    if (navGoalClient.call(srv))
    {
        ROS_INFO("导航请求发送成功: (%.3f, %.3f, %.3f)", x, y, theta);
        return true;
    }
    else
    {
        ROS_ERROR("导航请求失败");
        return false;
    }
}

// 从IAIUIListener派生自己的结果监听器
class DemoListener;
class DemoListener : public IAIUIListener
{

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    // 添加服务客户端成员
    ros::ServiceClient trigger_capture_client_;
    ros::Publisher prompt_update_pub_;
    // ===== [ADD] 给 qwen_point 指定目标物体 =====
    ros::Publisher target_object_pub_;

    // ===== [ADD] 同步定位服务（qwen_point.py 提供的 /locate_object_sync）=====
    ros::ServiceClient locate_sync_client_;

    //图片中的3D坐标订阅
    ros::Subscriber vision_3DLocation_sub_;

    // ===== [ADD] 防止并发任务重复触发 =====
    std::atomic<bool> task_running_{false};

    // ===== [ADD] 记住最近一次目标的 mode（用于“放下/松开”这种不带目标的指令）=====
    std::atomic<int> last_target_mode_{1}; // 默认 apple

    std::string description; 

    // ===== [ADD] target -> mode 映射：1 apple,2 orange,3 pear,4 peach,5 bottle =====
    static int targetToMode(const std::string &target_key)
    {
        if (target_key == "apple")
            return 1;
        if (target_key == "orange")
            return 2;
        if (target_key == "pear")
            return 3;
        if (target_key == "peach")
            return 4;

        // 兼容你现有的 water bottle / bottle 写法
        if (target_key == "bottle" || target_key == "water bottle" || target_key == "water_bottle")
            return 5;

        return 1; // 兜底：默认 apple
    }

    // ===== [ADD] 发给 qwen_point 的目标名称规范化（瓶子建议用 water bottle，更匹配你 qwen_point.py 的 alias_map）=====
    static std::string targetToVisionName(const std::string &target_key)
    {
        if (target_key == "bottle")
            return "water bottle";
        return target_key;
    }

    // 从StreamNlpTtsHelper::Listener派生流式合成监听器，用于监听大模型结果的合成
    class TtsHelperListener : public StreamNlpTtsHelper::Listener
    {

    public:
        void onText(const StreamNlpTtsHelper::OutTextSeg &textSeg) override
        {
            if (textSeg.isBegin() || textSeg.isEnd())
            {
                if (aiui_pcm_player_get_state() != PCM_PLAYER_STATE_STARTED)
                {
                    aiui_pcm_player_start();
                }
                if (textSeg.isBegin())
                {
                    aiui_pcm_player_clear();
                }
            }

            // 调用合成
            // startTTS(textSeg.mText, textSeg.mTag);
        }
        string lastTtsText;
        void onFinish(const string &fullText) override
        {
            // 文本合成完成回调
            // cout << "tts, fullText=" << fullText << endl;
            stopRecordAudio();
            // startTTS(fullText);
            if (fullText == lastTtsText)
            {
                // 重复合成，直接返回
                return;
            }
            lastTtsText = fullText; // 更新最后的合成文本
            // 重新开始录音
            startRecordAudio();
        }

        void onTtsData(const Json::Value &bizParamJson, const char *audio, int len) override
        {
            const Json::Value &data = (bizParamJson["data"])[0];
            const Json::Value &content = (data["content"])[0];
            int dts = content["dts"].asInt();
            int progress = content["text_percent"].asInt();

            // 将合成数据写入播放器
            aiui_pcm_player_write(0, audio, len, dts, progress);
        }
    };

private:
    std::shared_ptr<StreamNlpTtsHelper> m_pTtsHelper;

    struct Goal
    {
        double x{0}, y{0}, th{0};
    };

    std::map<std::string, Goal> goals_;
    Goal home_{0.013, 0.088, 0.132}; // 兜底默认值（yaml没配时用）

    static double toDouble(const XmlRpc::XmlRpcValue &v)
    {
        if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
            return static_cast<double>(v);
        if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
            return static_cast<int>(v);
        // 也有人会写成字符串，这里可以按需扩展
        throw std::runtime_error("param type is not int/double");
    }

    bool loadGoalsFromParam()
    {
        XmlRpc::XmlRpcValue goals_param;
        if (!nh_.getParam("goals", goals_param))
        {
            ROS_WARN("~goals not found. Using default hardcoded goals/home.");
            return false;
        }
        if (goals_param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_ERROR("~goals is not a dict/struct in YAML.");
            return false;
        }

        goals_.clear();
        for (auto it = goals_param.begin(); it != goals_param.end(); ++it)
        {
            const std::string key = it->first;
            XmlRpc::XmlRpcValue g = it->second;
            if (g.getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_WARN("goal [%s] is not struct, skip", key.c_str());
                continue;
            }
            Goal goal;
            goal.x = toDouble(g["x"]);
            goal.y = toDouble(g["y"]);
            goal.th = toDouble(g["th"]);
            goals_[key] = goal;
        }

        // home（可选）
        XmlRpc::XmlRpcValue home_param;
        if (nh_.getParam("home", home_param) &&
            home_param.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
            home_.x = toDouble(home_param["x"]);
            home_.y = toDouble(home_param["y"]);
            home_.th = toDouble(home_param["th"]);
        }
        else
        {
            ROS_WARN("~home not found or invalid, keep default home");
        }

        ROS_INFO("Loaded %zu goals from YAML. home=(%.3f, %.3f, %.3f)",
                 goals_.size(), home_.x, home_.y, home_.th);
        return !goals_.empty();
    }
    // ===== [ADD] 采摘任务点位 =====
    std::map<std::string, Goal> harvest_points_;
    int harvest_max_per_point_ = 20; // 每个点最多抓多少个，防止异常死循环
    double same_pose_eps_ = 0.03;    // m：重复目标位置判定阈值
    int same_pose_max_times_ = 3;    // 连续重复次数上限

    bool loadHarvestPointsFromParam()
    {
        XmlRpc::XmlRpcValue hp_param;
        if (!nh_.getParam("harvest_points", hp_param))
        {
            ROS_WARN("~harvest_points not found. Harvest task will NOT run until configured.");
            return false;
        }
        if (hp_param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_ERROR("~harvest_points is not a dict/struct in YAML.");
            return false;
        }

        harvest_points_.clear();
        for (auto it = hp_param.begin(); it != hp_param.end(); ++it)
        {
            const std::string key = it->first; // apple/orange/pear/peach
            XmlRpc::XmlRpcValue g = it->second;
            if (g.getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_WARN("harvest_point [%s] is not struct, skip", key.c_str());
                continue;
            }
            Goal goal;
            goal.x = toDouble(g["x"]);
            goal.y = toDouble(g["y"]);
            goal.th = toDouble(g["th"]);
            harvest_points_[key] = goal;
        }

        ROS_INFO("Loaded %zu harvest_points from YAML.", harvest_points_.size());
        return !harvest_points_.empty();
    }

    bool loadHarvestConfigFromParam()
    {
        // 这些是可选参数：不写就用成员变量默认值
        nh_.param("harvest_max_per_point", harvest_max_per_point_, harvest_max_per_point_);
        nh_.param("same_pose_eps", same_pose_eps_, same_pose_eps_);
        nh_.param("same_pose_max_times", same_pose_max_times_, same_pose_max_times_);

        // 简单的保护，避免配置写错导致异常循环
        if (harvest_max_per_point_ < 1)
            harvest_max_per_point_ = 1;
        if (same_pose_eps_ <= 0.0)
            same_pose_eps_ = 0.01;
        if (same_pose_max_times_ < 1)
            same_pose_max_times_ = 1;

        ROS_INFO("Harvest config: harvest_max_per_point=%d, same_pose_eps=%.4f, same_pose_max_times=%d",
                 harvest_max_per_point_, same_pose_eps_, same_pose_max_times_);
        return true;
    }

    static bool isHarvestIntent(const std::string &text)
    {
        // 你可以按需再加同义说法
        return (text.find("开始采摘") != std::string::npos ||
                text.find("启动采摘") != std::string::npos ||
                text.find("开始摘") != std::string::npos ||
                text.find("采摘开始") != std::string::npos);
    }

    // 每个点位：循环“定位->抓->放(2)”直到没有目标
    void harvestAtPoint(const std::string &target_key, int mode)
    {
        // 目标发布一次（后续循环也可重复发布，保险起见）
        std_msgs::String tmsg;
        tmsg.data = targetToVisionName(target_key);
        target_object_pub_.publish(tmsg);

        last_target_mode_.store(mode);

        startTTS("开始识别并采摘");

        int picked = 0;

        // 用于“同一个位置反复识别”的兜底防死循环
        double last_x = 1e9, last_y = 1e9, last_z = 1e9;
        int same_times = 0;

        for (int i = 0; i < harvest_max_per_point_; ++i)
        {
            // 每轮都再发一次目标，确保 qwen_point 当前目标正确
            target_object_pub_.publish(tmsg);

            std::string locateMsg;
            if (!callLocateObjectSync(locateMsg))
            {
                // 失败一般意味着：没找到目标 / 深度无效 / 超时
                break;
            }

            // 尝试解析返回的 JSON，做“重复目标”检测
            Json::Value root;
            Json::Reader reader;
            if (reader.parse(locateMsg, root, false))
            {
                double x = root.get("x", 0).asDouble();
                double y = root.get("y", 0).asDouble();
                double z = root.get("z", 0).asDouble();

                double dx = x - last_x, dy = y - last_y, dz = z - last_z;
                double d = std::sqrt(dx * dx + dy * dy + dz * dz);

                if (d < same_pose_eps_)
                    same_times++;
                else
                    same_times = 0;

                last_x = x;
                last_y = y;
                last_z = z;

                if (same_times >= same_pose_max_times_)
                {
                    startTTS("目标位置重复，可能未抓取成功，跳过该点");
                    ROS_WARN("Harvest stuck: same target pose repeated.");
                    break;
                }
            }

            // 抓取 + 放置(2)
            startTTS("发现目标，开始抓取");
            callYoloPick(mode);

            // 你要求：每次抓完后放置调用改为 mode=2
            callYoloPlace(2);

            picked++;
            ros::Duration(0.6).sleep(); // 给机械臂一点时间（按你的系统节拍可调）
        }

        if (picked > 0)
            startTTS("该点采摘完成");
        else
            startTTS("未发现目标或目标已采摘完成");
    }

    void runHarvestTask()
    {
        // 必须要求点位配置齐全
        const char *keys[4] = {"apple", "orange", "pear", "peach"};
        for (auto k : keys)
        {
            if (harvest_points_.find(k) == harvest_points_.end())
            {
                startTTS("采摘点位未配置，无法开始采摘任务");
                ROS_ERROR("Missing harvest_points key: %s", k);
                return;
            }
        }

        startTTS("收到，开始采摘任务");

        struct Item
        {
            std::string key;
            std::string target_key;
            int mode;
            std::string speak;
        };

        std::vector<Item> plan = {
            {"apple", "apple", 1, "前往苹果采摘点"},
            {"orange", "orange", 2, "前往橘子采摘点"},
            {"pear", "pear", 3, "前往梨采摘点"},
            {"peach", "peach", 4, "前往桃子采摘点"},
        };

        // 依次四个点
        for (const auto &it : plan)
        {
            startTTS(it.speak);

            const auto &g = harvest_points_[it.key];
            if (!navigateTo(g.x, g.y, g.th))
            {
                startTTS("导航失败，采摘任务终止");
                return;
            }

            startTTS("已到达点位");
            harvestAtPoint(it.target_key, it.mode);
        }

        // 回到起始点（这里用 home_ 作为“起始点/回家点”）
        startTTS("采摘完成，返回起始点");
        navigateTo(home_.x, home_.y, home_.th);

        startTTS("任务完成");
    }

private:
    // ===== 单例：禁止外部构造/拷贝 =====
    DemoListener() : nh_("~")
    {
        // 创建图像描述订阅者 - 添加的4行代码
        image_sub_ = nh_.subscribe("/vision_description", 1,
                                   &DemoListener::VisionImageDescriptionCallback, this);
        // ++++ 初始化trigger_capture服务客户端 ++++
        trigger_capture_client_ = nh_.serviceClient<std_srvs::Trigger>("/vision_trigger_capture"); //曾经是/trigger_capture
        // ++++ 初始化prompt更新发布者 ++++
        prompt_update_pub_ = nh_.advertise<std_msgs::String>("/prompt_update", 10);
        // ===== [ADD] 发布目标物体名到 /target_object（绝对话题名）=====
        target_object_pub_ = nh_.advertise<std_msgs::String>("/target_object", 1, true);

        // ===== [ADD] 同步定位服务客户端 =====
        locate_sync_client_ = nh_.serviceClient<std_srvs::Trigger>("/locate_object_sync");

        vision_3DLocation_sub_ = nh_.subscribe("/object_position", 1, &DemoListener::Vision3DLocationCallback, this);

        // 创建内置的pcm播放器，并初始化，设置回调，启动起来
        aiui_pcm_player_create();
        aiui_pcm_player_init();
        aiui_pcm_player_set_callbacks(
            onStarted, onPaused, onResumed, onStopped, onProgress, onError);
        aiui_pcm_player_start();

        std::shared_ptr<TtsHelperListener> listener = std::make_shared<TtsHelperListener>();
        m_pTtsHelper = std::make_shared<StreamNlpTtsHelper>(listener);
        m_pTtsHelper->setTextMinLimit(20);
        // 最后加载一次
        loadGoalsFromParam();
        loadHarvestPointsFromParam(); // ===== [ADD] 加这一行 =====
        loadHarvestConfigFromParam(); // ===== [ADD] 加这一行 =====
    }

public:

    double last_3d_x_ = 0, last_3d_y_ = 0, last_3d_z_ = 0; // 最近一次3D坐标
    // ===== 单例入口 =====
    static DemoListener& getInstance() {
        static DemoListener instance;  // Meyer's Singleton，线程安全
        return instance;
    }
    DemoListener(const DemoListener&) = delete;
    DemoListener& operator=(const DemoListener&) = delete;

    void callTriggerCaptureService()
    {
        if (!trigger_capture_client_.exists())
        {
            startTTS("视觉服务未就绪");
            std::cout << "视觉服务未就绪!" << endl;
            return;
        }
        std_srvs::Trigger srv;
        if (trigger_capture_client_.call(srv))
        {
            if (srv.response.success)
                startTTS("好的，我看到了");
            else
                startTTS("拍照失败");
                std::cout << "拍照失败!" << endl;
        }
        else
        {
            startTTS("调用视觉服务失败");
            std::cout << "调用视觉服务失败!" << endl;
        }
    }

    // ===== 返回图片描述（非阻塞，立即返回当前值）=====
    string getLatestImageDescription() const {
        return this->description;
    }

    // ===== 阻塞等待图片描述（用于 agent 模式同步获取视觉识别结果）=====
    std::string waitForImageDescription(double timeout_sec = 15.0) {
        this->description.clear();  // 清空旧描述，确保拿到的是新结果
        ros::Rate rate(10);   // 10 Hz 轮询
        ros::Time start = ros::Time::now();
        while (this->description.empty()) {
            if ((ros::Time::now() - start).toSec() > timeout_sec) {
                ROS_WARN("waitForImageDescription: 超时（%.1f秒）未收到图像描述", timeout_sec);
                return "";  // 超时返回空串，由调用方处理
            }
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("waitForImageDescription: 获取到描述，长度=%zu", this->description.size());
        return this->description;
    }

    // ===== 返回target_object_pub_指针（供 agent 模式直接发布目标）=====
    ros::Publisher& getTargetObjectPublisher() {
        return target_object_pub_;
    }

    

    ~DemoListener()
    {
        // 析构时销毁播放器，释放资源
        aiui_pcm_player_destroy();
    }

    /**
     * 重写onEvent方法，SDK通过回调该方法抛出各种事件，在这里针对事件做对应的处理。
     *
     * @param event
     */
    void onEvent(const IAIUIEvent &event) override
    {
        try
        {
            handleEvent(event);
        }
        catch (std::exception &e)
        {
            cout << e.what() << endl;
        }
    }

    // 是否输出更多信息
    bool mMoreDetails = true;
    void processCommand(const string &commandText)
    {
        ROS_INFO("处理指令: %s", commandText.c_str());

        // 1. 导航指令
        if (commandText.find("导航到电梯口") != string::npos ||
            commandText.find("前往电梯间的路线") != string::npos ||
            commandText.find("导航到电梯间") != string::npos)
        {
            navigateTo(-2.440, -0.155, 0.147);
        }
        else if (commandText.find("导航到厕所") != string::npos ||
                 commandText.find("前往卫生间的路线") != string::npos ||
                 commandText.find("导航到卫生间") != string::npos)
        {
            navigateTo(5.359, -23.576, -1.455);
        }
        else if (commandText.find("导航到405") != string::npos ||
                 commandText.find("导航到装配件") != string::npos ||
                 commandText.find("导航到装配间") != string::npos)
        {
            navigateTo(2.987, 7.342, -1.528);
        }
        // 添加更多导航目标...

        // 2. 抓取放置指令
        else if (commandText.find("抓取") != string::npos ||
                 commandText.find("拿取") != string::npos ||
                 commandText.find("取物") != string::npos)
        {
            callYoloPick(last_target_mode_.load());
        }
        else if (commandText.find("放下") != string::npos ||
                 commandText.find("松开") != string::npos ||
                 commandText.find("放置") != string::npos)
        {
            callYoloPlace(1);
        }

        // 3. 视觉相关指令
        else if (commandText.find("观察") != string::npos ||
                 commandText.find("看看") != string::npos ||
                 commandText.find("查看") != string::npos)
        {
            callTriggerCaptureService();
        }
    }

    // ===== [ADD] 从用户话里提取目标物体（返回英文名给 qwen_point）=====
    static std::string extractTarget(const std::string &text)
    {
        // 0) 特殊：水蜜桃必须优先，否则“拿水蜜桃”会被“拿水”兜底误判
        if (text.find("水蜜桃") != std::string::npos ||
            text.find("蜜桃") != std::string::npos ||
            text.find("桃子") != std::string::npos ||
            text.find("黄桃") != std::string::npos ||
            text.find("油桃") != std::string::npos)
            return "peach";

        // 1) 明确水果类
        if (text.find("梨") != std::string::npos ||
            text.find("梨子") != std::string::npos ||
            text.find("雪梨") != std::string::npos ||
            text.find("香梨") != std::string::npos ||
            text.find("鸭梨") != std::string::npos)
            return "pear";

        if (text.find("橙子") != std::string::npos ||
            text.find("脐橙") != std::string::npos ||
            text.find("甜橙") != std::string::npos ||
            text.find("橘子") != std::string::npos ||
            text.find("桔子") != std::string::npos)
            return "orange";

        if (text.find("苹果") != std::string::npos ||
            text.find("红富士") != std::string::npos ||
            text.find("青苹果") != std::string::npos)
            return "apple";

        // 2) 明确“水瓶/矿泉水”等
        if (text.find("水瓶") != std::string::npos ||
            text.find("矿泉水") != std::string::npos ||
            text.find("瓶装水") != std::string::npos ||
            text.find("饮用水") != std::string::npos ||
            text.find("一瓶水") != std::string::npos ||
            text.find("一瓶矿泉水") != std::string::npos)
            return "bottle";

        // 3) 兜底：更自然的“拿水/取水/带水” —— 但要排除“水蜜桃/水果”
        // 说明：
        // - 只在出现“动作词 + 水”的句式才触发
        // - (?!蜜桃|果) 排除 “水蜜桃 / 水果”
        // - {0,6} 表示动作词和“水”之间允许插入少量字：如“帮我拿点水”
        static const std::regex re_fetch_water(
            u8"(拿|取|抓|带|帮我拿|帮我取|帮我抓|给我|递给我|帮我带)[^，。！？\\s]{0,6}水(?!蜜桃|果)");

        if (std::regex_search(text, re_fetch_water))
            return "bottle";

        return "";
    }

    // ===== [ADD] 从用户话里提取目的地（返回内部 key）=====
    static std::string extractDestination(const std::string &text)
    {
        if (text.find("办公室") != std::string::npos)
            return "office";
        if (text.find("厕所") != std::string::npos || text.find("卫生间") != std::string::npos)
            return "toilet";
        if (text.find("电梯") != std::string::npos)
            return "elevator";
        if (text.find("405") != std::string::npos || text.find("装配") != std::string::npos)
            return "room405";
        if (text.find("402") != std::string::npos)
            return "room402";
        if (text.find("406") != std::string::npos)
            return "room406";
        return "";
    }

    // ===== [ADD] 是否是“取/拿/抓”的动作意图 =====
    static bool isFetchIntent(const std::string &text)
    {
        return (text.find("拿") != std::string::npos ||
                text.find("取") != std::string::npos ||
                text.find("抓") != std::string::npos ||
                text.find("带") != std::string::npos);
    }

    // ===== [ADD] 调用 qwen_point.py 的 /locate_object_sync =====
    bool callLocateObjectSync(std::string &outJsonMsg)
    {
        if (!locate_sync_client_.exists())
        {
            ROS_WARN("/locate_object_sync service not available!");
            return false;
        }

        std_srvs::Trigger srv;
        if (!locate_sync_client_.call(srv))
        {
            ROS_ERROR("call /locate_object_sync failed");
            return false;
        }

        if (!srv.response.success)
        {
            ROS_WARN("locate failed: %s", srv.response.message.c_str());
            outJsonMsg = srv.response.message;
            return false;
        }

        outJsonMsg = srv.response.message; // 成功时是 JSON 字符串
        return true;
    }

    // ===== [ADD] 处理用户最终识别文本：入口函数 =====
    void handleUserUtterance(const std::string &userText)
    {
        /**
         * 初步想法在这里建立一个"插桩点"，
         * 通过在这里导入一个新函数来让agent模式从这里分流
         */
        // ===== Agent 模式分流 =====
        if (agent::handleText(userText))
            return;
        
        
        std::string text = userText;
        ROS_INFO("用户指令(ASR最终): %s", text.c_str());
        if (text.empty())
            return;

        // ===== [ADD] 采摘任务入口 =====
        if (isHarvestIntent(text))
        {
            if (task_running_.exchange(true))
            {
                startTTS("我正在执行任务，请稍等");
                return;
            }

            std::thread([this]()
                        {
                            runHarvestTask();
                            task_running_ = false; })
                .detach();
            return;
        }

        // 1) 放下/松开：直接放置
        if (text.find("放下") != std::string::npos ||
            text.find("松开") != std::string::npos ||
            text.find("放回") != std::string::npos)
        {
            int mode = last_target_mode_.load();
            callYoloPlace(1);
            // callYoloPlace(mode);
            startTTS("放置完成");
            return;
        }

        // 2) 先让“导航/观察/看看/查看”等非取物命令能工作（走你原来的 processCommand）
        //    注意：processCommand 里也包含抓取/放置的关键词，所以放在 isFetchIntent 之前兜底更稳
        if (!isFetchIntent(text))
        {
            std::cout<<"=================没有检测到================"<<endl;
            processCommand(text);
            return;
        }

        // 3) 取物意图：解析目的地 + 目标物体
        std::string dest = extractDestination(text);
        std::string target = extractTarget(text);

        if (target.empty())
        {
            startTTS("我没有听清楚要拿什么，比如：拿一瓶水，拿一个苹果");
            return;
        }

        // 4) 防止任务重复触发
        if (task_running_.exchange(true))
        {
            startTTS("我正在执行上一个任务，请稍等");
            return;
        }

        // 5) 不说目的地：就地“定位->抓取”（方案A也很常用）
        if (dest.empty())
        {
            std::cout<<"=================单独的拿起瓶子================"<<endl;
            std::thread([this, target]()
                        {
            runLocalFetchTask(target);
            task_running_ = false; })
                .detach();
            return;
        }

        // 6) 说了目的地：去目的地->定位->抓取->回家->放置
        std::thread([this, dest, target]()
                    {
        runFetchTask(dest, target);
        task_running_ = false; })
            .detach();
    }

    void runLocalFetchTask(const std::string &target_key)
    {
        int mode = targetToMode(target_key);
        last_target_mode_.store(mode);

        std::string vision_target = targetToVisionName(target_key);

        // 发布目标物体给 qwen_point
        std_msgs::String msg;
        msg.data = vision_target;
        target_object_pub_.publish(msg);

        startTTS("好的，我来找一下目标物体");

        // 同步定位
        std::string locateMsg;
        if (!callLocateObjectSync(locateMsg))
        {
            startTTS("没有找到目标物体");
            return;
        }

        startTTS("已定位到目标，开始抓取");
        callYoloPick(mode);
        startTTS("抓取完成");
    }

    /**
     * 
     * 目的地是指用户语音指令中提到的要先去的地点，比如"去办公室拿一个苹果"中的"办公室"就是目的地。
        extractDestination() 函数从用户的话中提取中文地点名，映射为内部 key：
        用户说的	内部 key
        办公室	"office"
        厕所/卫生间	"toilet"
        电梯	"elevator"
        405/装配间	"room405"
        然后 goals_.find(dest) 在 YAML 配置文件中查找该目的地对应的导航坐标（x, y, theta）——这些是地图上预先标定的固定点位，
        通过 navigateTo() 发给导航节点让机器人移动过去。
     */
    // ===== [ADD] 执行任务：去目的地 -> 定位目标 -> 抓取 -> 回来 -> 放置 =====
    void runFetchTask(const std::string &dest, const std::string &target_key)
    {
        int mode = targetToMode(target_key);
        last_target_mode_.store(mode);

        std::string vision_target = targetToVisionName(target_key);

        auto it = goals_.find(dest);
        if (it == goals_.end())
        {
            startTTS("我不知道这个目的地");
            ROS_WARN("Unknown destination key: %s", dest.c_str());
            return;
        }

        // 1) 发布目标物体到 /target_object，给 qwen_point 指定检测目标
        std_msgs::String msg;
        msg.data = vision_target;
        target_object_pub_.publish(msg);

        startTTS("收到指令，开始前往目的地");

        // 2) 导航
        const auto &g = it->second;
        if (!navigateTo(g.x, g.y, g.th))
        {
            startTTS("导航失败");
            return;
        }
        startTTS("已到达目的地，开始寻找目标物体");

        // 3) 同步定位
        std::string locateMsg;
        if (!callLocateObjectSync(locateMsg))
        {
            // locateMsg 是失败原因
            startTTS("没有找到目标物体");
            return;
        }

        // 4) 解析定位 JSON（成功时）
        Json::Value root;
        Json::Reader reader;
        if (reader.parse(locateMsg, root, false))
        {
            std::string name = root.get("name", "").asString();
            std::string frame = root.get("frame_id", "").asString();
            double x = root.get("x", 0).asDouble();
            double y = root.get("y", 0).asDouble();
            double z = root.get("z", 0).asDouble();

            ROS_INFO("定位成功: name=%s frame=%s xyz=(%.3f,%.3f,%.3f)",
                     name.c_str(), frame.c_str(), x, y, z);
            startTTS("已定位到目标，开始抓取");
        }
        else
        {
            // 就算解析失败，也可以继续抓（因为 qwen_point 已经把 /object_position latch 发布出来了）
            startTTS("已定位到目标，开始抓取");
        }

        // 5) 抓取（你的抓取节点如果订阅 /object_position，就能用到刚才定位结果）
        callYoloPick(mode);

        // 6) 返回原点并放置（按你项目流程决定是否需要）
        startTTS("抓取完成，返回并放置");
        navigateTo(home_.x, home_.y, home_.th);
        callYoloPlace(mode);

        startTTS("任务完成");
    }

    //发布定位接口物体函数
    void publichTargetObject(const std::string &target_key)
    {
        std_msgs::String msg;
        msg.data = targetToVisionName(target_key);
        target_object_pub_.publish(msg);
    }


private:
    fstream mFs;

    // 当前合成sid
    string mCurTtsSid;

    // 当前识别sid
    string mCurIatSid;

    // 识别结果缓存
    string mIatTextBuffer;

    // 流式nlp的应答语缓存
    string mStreamNlpAnswerBuffer;

    // 意图的数量
    int mIntentCnt = 0;

private:
    static void processIntentJson(Json::Value &params,
                                  Json::Value &intentJson,
                                  std::string &resultStr,
                                  int eosRsltTime,
                                  std::string &sid,
                                  bool shouldSynthesize = false)
    {
        int rc = intentJson["rc"].asInt();

        Json::Value answerJson = intentJson["answer"];
        std::string answerText = answerJson.get("text", "").asString();

        // cout << "----------------------------------" << endl;
        // cout << "params: " << params.toString() << endl;
        // cout << "nlp: " << resultStr << endl;
        // cout << "eos_result=" << eosRsltTime << "ms" << endl;
        // cout << "结果解析：" << endl;
        // cout << "sid=" << sid << endl;
        // cout << "text（请求文本）: " << intentJson.get("text", "").asString() << endl;
        // cout << "rc=" << rc << ", answer（应答语）: " << answerText << endl;

        if (shouldSynthesize && !answerText.empty())
        {
            startTTS(answerText);
        }
    }
    void VisionImageDescriptionCallback(const std_msgs::String::ConstPtr &msg)
    {
        ROS_INFO("收到图像描述: %s", msg->data.c_str());

        // 截断长描述以避免合成时间过长
        this->description = msg->data;
        if (this->description.length() > 1000)
        {
            this->description = this->description.substr(0, 1000) + "等";
        }

        // 用语音合成描述
        startTTS(this->description, "image_description");
    }

    void Vision3DLocationCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        // 获取 position 的引用
        const auto& p = msg->pose.position;
        
        // 计算 3D 距离
        double dist = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        last_3d_x_ = p.x;
        last_3d_y_ = p.y;
        last_3d_z_ = p.z;


        // 打印日志
        ROS_INFO("📍 Position [%s]: x=%.3f  y=%.3f  z=%.3f (dist=%.3f m)",
                 msg->header.frame_id.c_str(),
                 p.x, p.y, p.z,
                 dist);
    }
    void handleEvent(const IAIUIEvent &event)
    {
        switch (event.getEventType())
        {
        // SDK状态
        case AIUIConstant::EVENT_STATE:
        {
            switch (event.getArg1())
            {
            case AIUIConstant::STATE_IDLE:
            {
                // 空闲状态，即最初始的状态
                cout << "EVENT_STATE: STATE_IDLE" << endl;
            }
            break;

            case AIUIConstant::STATE_READY:
            {
                // 准备好状态（待唤醒），可以进行唤醒
                cout << "EVENT_STATE: STATE_READY" << endl;
            }
            break;

            case AIUIConstant::STATE_WORKING:
            {
                // 工作状态（即已唤醒状态），可以语音交互，也可以再次唤醒
                cout << "EVENT_STATE: STATE_WORKING" << endl;
            }
            break;
            }
        }
        break;

        // 唤醒事件
        case AIUIConstant::EVENT_WAKEUP:
        {
            cout << "EVENT_WAKEUP: " << event.getInfo() << endl;

            // 唤醒时停止播放
            aiui_pcm_player_stop();
            startTTS("我在");
        }
        break;

        // 休眠事件，即一段时间无有效交互或者外部主动要求，SDK会自动进入STATE_READY状态
        case AIUIConstant::EVENT_SLEEP:
        {
            // arg1用来区分休眠类型，是自动休眠还是外部要求，可参考AIUIConstant.h中EVENT_SLEEP的注释
            cout << "EVENT_SLEEP: arg1=" << event.getArg1() << endl;
            // 这里添加语音合成逻辑
            // startTTS("主人，没什么事的话我先休息了。有需要的话，随时呼唤我");
        }
        break;

        // VAD事件，如语音活动检测
        case AIUIConstant::EVENT_VAD:
        {
            // arg1为活动类型
            switch (event.getArg1())
            {
            case AIUIConstant::VAD_BOS_TIMEOUT:
            {
                cout << "EVENT_VAD: VAD_BOS_TIMEOUT" << endl;
            }
            break;

            // 检测到前端点，即开始说话
            case AIUIConstant::VAD_BOS:
            {
                cout << "EVENT_VAD: BOS" << endl;
            }
            break;

            // 检测到后端点，即说话结束
            case AIUIConstant::VAD_EOS:
            {
                cout << "EVENT_VAD: EOS" << endl;
            }
            break;

            // 音量，arg2为音量级别（0-30）
            case AIUIConstant::VAD_VOL:
            {
                // cout << "EVENT_VAD: vol=" << event.getArg2() << endl;
            }
            break;
            }
        }
        break;

        // 结果事件
        case AIUIConstant::EVENT_RESULT:
        {
            Json::Value bizParamJson;
            Json::Reader reader;

            if (!reader.parse(event.getInfo(), bizParamJson, false))
            {
                cout << "parse error! info=" << event.getInfo() << endl;
                break;
            }

            Json::Value &data = (bizParamJson["data"])[0];
            Json::Value &params = data["params"];
            Json::Value &content = (data["content"])[0];

            string sub = params["sub"].asString();

            if (sub != "iat" && sub != "nlp" && sub != "tts" && sub != "cbm_intent_split" && sub != "cbm_semantic")
            {
                return;
            }
            // sid即唯一标识一次会话的id
            string sid = event.getData()->getString("sid", "");
            if (sub == "iat")
            {
                /** if (sid != mCurIatSid) {
                     cout << "**********************************" << endl;
                     cout << "sid=" << sid << endl;

                     mCurIatSid = sid;

                     // 新的会话，清空之前识别缓存
                     mIatTextBuffer.clear();
                     mStreamNlpAnswerBuffer.clear();
                     m_pTtsHelper->clear();
                     mIntentCnt = 0;**/

                string cnt_id = content.get("cnt_id", "").asString();
                int dataLen = 0;
                const char *buffer = event.getData()->getBinary(cnt_id.c_str(), &dataLen);
                string resultStr = string(buffer, dataLen); // 在这里声明并定义 resultStr
                Json::Value resultJson;
                if (reader.parse(resultStr, resultJson, false))
                {
                    Json::Value textJson = resultJson["text"];
                    bool isWpgs = false;
                    if (textJson.isMember("pgs"))
                    {
                        isWpgs = true;
                    }

                    if (isWpgs)
                    {
                        mIatTextBuffer = IatResultUtil::parsePgsIatText(textJson);
                    }
                    else
                    {
                        // 结果拼接起来
                        mIatTextBuffer.append(IatResultUtil::parseIatResult(textJson));
                    }
                    // 是否是该次会话最后一个识别结果
                    bool isLast = textJson["ls"].asBool();
                    if (isLast)
                    {
                        cout << "params: " << params.asString() << endl;
                        cout << "iat: " << mIatTextBuffer << endl;

                        // ===== [MOD] 用户最终语音识别文本（真正的指令入口）=====
                        DemoListener::getInstance().handleUserUtterance(mIatTextBuffer);

                        // 你如果还想保留 prompt_update 给别的功能用，可以保留；否则建议注释掉
                        // std_msgs::String prompt_msg;
                        // prompt_msg.data = mIatTextBuffer;
                        // prompt_update_pub_.publish(prompt_msg);

                        mIatTextBuffer.clear();
                    }
                } // 判断是否包含导航关键词
            }
            else if (sub == "tts")
            {
                if (sid != mCurTtsSid)
                {
                    cout << "**********************************" << endl;
                    cout << "sid=" << sid << endl;

                    mCurTtsSid = sid;
                }
            }

            Json::Value empty;
            string cnt_id = content.get("cnt_id", empty).asString();

            int dataLen = 0;

            // 注意：当buffer里存字符串时也不是以0结尾，当使用C语言时，转成字符串则需要自已在末尾加0
            const char *buffer = event.getData()->getBinary(cnt_id.c_str(), &dataLen);

            if (sub == "tts")
            {
                // 语音合成结果，返回url或者pcm音频
                // cout << "tts: " << content.toString() << endl;

                Json::Value &&isUrl = content.get("url", empty);
                if (isUrl.asString() == "1")
                {
                    // 云端返回的是url链接，可以用播放器播放

                    cout << "tts_url=" << string(buffer, dataLen) << endl;
                }
                else
                {
                    // 云端返回的是pcm音频，分成一块块流式返回
                    int progress = 0;
                    int dts = content["dts"].asInt();

                    string tag = event.getData()->getString("tag", "");
                    if (tag.find("stream_nlp_tts") == 0)
                    {
                        // 流式语义应答的合成
                        m_pTtsHelper->onOriginTtsData(tag, bizParamJson, buffer, dataLen);
                    }
                    else
                    {
                        if (dts == AIUIConstant::DTS_BLOCK_FIRST || dts == AIUIConstant::DTS_ONE_BLOCK)
                        {
                            // 只有碰到开始块，才开启播放器
                            if (aiui_pcm_player_get_state() != PCM_PLAYER_STATE_STARTED)
                            {
                                aiui_pcm_player_start();
                            }
                        }

                        aiui_pcm_player_write(0, buffer, dataLen, dts, progress);

                        // 若要保存合成音频，请打开以下开关
#if 0
                            // 音频开始
                            if (dts == AIUIConstant::DTS_BLOCK_FIRST || dts == AIUIConstant::DTS_ONE_BLOCK) {
                                mFs.open("tts.pcm", ios::binary | ios::out);
                            }

                            mFs.write(buffer, dataLen);

                            // 音频结束
                            if (dts == AIUIConstant::DTS_BLOCK_LAST || dts == AIUIConstant::DTS_ONE_BLOCK) {
                                mFs.close();
                            }
#endif
                    }
                }
            }
            else if (sub == "nlp")
            {
                // 语义理解结果
                string resultStr = string(buffer, dataLen); // 注意：这里不能用string resultStr = buffer，因为buffer不一定以0结尾

                // 从说完话到语义结果返回的时长
                long eosRsltTime = event.getData()->getLong("eos_rslt", -1);

                Json::Value resultJson;
                if (reader.parse(resultStr, resultJson, false))
                {
                    // 判断是否为有效结果
                    if (resultJson.isMember("intent") &&
                        resultJson["intent"].isMember("rc"))
                    {
                        // AIUI v1的语义结果
                        Json::Value intentJson = resultJson["intent"];
                        processIntentJson(params, intentJson, resultStr, eosRsltTime, sid);
                    }
                    else if (resultJson.isMember("nlp"))
                    {
                        // AIUI v2的语义结果
                        Json::Value nlpJson = resultJson["nlp"];
                        string text = nlpJson["text"].asString();

                        if (text.find("{\"intent\":") == 0)
                        {
                            // 通用语义结果
                            Json::Value textJson;
                            if (reader.parse(text, textJson, false))
                            {
                                Json::Value intentJson = textJson["intent"];
                                processIntentJson(params, intentJson, resultStr, eosRsltTime, sid);
                            }
                        }
                        else
                        {
                            // 大模型语义结果
                            // 流式nlp结果里面有seq和status字段
                            int seq = nlpJson["seq"].asInt();
                            int status = nlpJson["status"].asInt();

                            /* 多意图取最后一次问题的结果进行tts合成 */
                            if (mIntentCnt > 0)
                            {
                                int currentIntentIndex = 0;
                                Json::Value metaNlpJson;
                                Json::Value textJson = resultJson["cbm_meta"].get("text", metaNlpJson);
                                if (reader.parse(textJson.asString(), metaNlpJson, false))
                                {
                                    currentIntentIndex = metaNlpJson["nlp"]["intent"].asInt();
                                    if ((mIntentCnt - 1) != currentIntentIndex)
                                    {
                                        cout << "ignore nlp:" << resultStr << endl;
                                        return;
                                    }
                                }
                                else
                                {
                                    cout << "ignore nlp:" << resultStr << endl;
                                    return;
                                }
                            }

#ifndef USE_POST_SEMANTIC_TTS
                            // 如果使用应用的语义后合成不需要在调用下面的函数否则tts的播报会重复
                            m_pTtsHelper->addText(text, seq, status);
#endif

                            // cout << "----------------------------------" << endl;
                            // cout << "params: " << params.asString() << endl;
                            // cout << "nlp: " << resultStr << endl;

                            // if (seq == 0)
                            // {
                            //     long eosRsltTime = event.getData()->getLong("eos_rslt", -1);
                            //     cout << "eos_result=" << eosRsltTime << "ms" << endl;
                            // }

                            // cout << "结果解析：" << endl;
                            // cout << "sid=" << sid << endl;
                            // cout << "seq=" << seq << ", status=" << status << ", answer（应答语）: " << text << endl;
                            // // cout << "fullAnswer=" << (mStreamNlpAnswerBuffer.append(text)) << endl;

                            if (status == 2)
                            {
                                mStreamNlpAnswerBuffer.clear();
                            }
                        }
                    }
                    else
                    {
                        // // 无效结果，把原始结果打印出来
                        // cout << "----------------------------------" << endl;
                        // cout << "nlp: " << resultStr << endl;
                        // cout << "sid=" << sid << endl;
                    }
                }
            }
            else if (sub == "cbm_intent_split")
            {
                // 意图拆分的结果
                string intentStr = string(buffer, dataLen); // 注意：这里不能用string resultStr = buffer，因为buffer不一定以0结尾
                Json::Value tmpJson;
                if (reader.parse(intentStr, tmpJson, false))
                {
                    Json::Value intentTextJson = tmpJson["cbm_intent_split"]["text"];
                    if (!intentTextJson.empty() &&
                        reader.parse(intentTextJson.asString(), tmpJson, false))
                    {
                        mIntentCnt = tmpJson["intent"].size();
                        cout << "cbm_intent_cnt: " << mIntentCnt
                             << " text: " << tmpJson.toString() << endl;
                    }
                }
            }
            else
            {
                // 其他结果
                string resultStr = string(buffer, dataLen); // 注意：这里不能用string resultStr = buffer，因为buffer不一定以0结尾

                cout << sub << ": " << event.getInfo() << endl
                     << resultStr << endl;
            }
        }
        break;

        // 与CMD命令对应的返回结果，arg1为CMD类型，arg2为错误码
        case AIUIConstant::EVENT_CMD_RETURN:
        {
            if (AIUIConstant::CMD_BUILD_GRAMMAR == event.getArg1())
            {
                // 语法构建命令的结果
                // 注：需要集成本地esr引擎才能构建语法
                if (event.getArg2() == 0)
                {
                    cout << "build grammar success." << endl;
                }
                else
                {
                    cout << "build grammar, error=" << event.getArg2() << ", des=" << event.getInfo() << endl;
                }
            }
            else if (AIUIConstant::CMD_UPDATE_LOCAL_LEXICON == event.getArg1())
            {
                // 更新本地语法槽的结果
                if (event.getArg2() == 0)
                {
                    cout << "update lexicon success" << endl;
                }
                else
                {
                    cout << "update lexicon, error=" << event.getArg2() << "des=" << event.getInfo() << endl;
                }
            }
            else if (AIUIConstant::CMD_CLONE_VOICE == event.getArg1())
            {
                // 声音复刻
                int dtype = event.getData()->getInt("sync_dtype", -1);
                int retCode = event.getArg2();
                string dataTypeStr;
                if (dtype == AIUIConstant::VOICE_CLONE_REG)
                { // 注册资源
                    dataTypeStr = "注册音频资源";
                }
                else if (dtype == AIUIConstant::VOICE_CLONE_DEL)
                { // 删除资源
                    dataTypeStr = "删除资源";
                }

                if (AIUIConstant::SUCCESS == retCode)
                {
                    // 上传成功，会话的唯一id，用于反馈问题的日志索引字段，注意留存
                    // 注：上传成功立即生效
                    string sid = event.getData()->getString("sid", "");
                    // 获取上传调用时设置的自定义tag
                    string tag = event.getData()->getString("tag", "");
                    // 获取上传调用耗时，单位：ms
                    long timeSpent = event.getData()->getLong("time_spent", -1);
                    cout << "声音复刻" << dataTypeStr << "成功"
                         << "，耗时：" << timeSpent
                         << "ms, sid=" + sid + "，tag=" + tag;
                    if (dtype == AIUIConstant::VOICE_CLONE_REG)
                    {
                        string resId = event.getData()->getString("res_id", "");
                        cout << "，res id = " << resId << endl;

                        // 保存声音复刻的的res id
                        gVoiceCloneResId = resId;
                        fstream fs;
                        fs.open("./voice_clone_reg_id.txt", ios::binary | ios::out);
                        fs.write(resId.c_str(), resId.length());
                        fs.close();
                    }
                    else
                    {
                        cout << endl;
                    }
                }
                else
                {
                    string result = event.getData()->getString("result", "");
                    cout << "声音复刻" << dataTypeStr << "失败，错误码：" << retCode << " info:" << event.getInfo() << " result:" << result << endl;
                }
            }
            else if (AIUIConstant::CMD_SYNC == event.getArg1())
            {
                // 数据同步的返回
                int dtype = event.getData()->getInt("sync_dtype", -1);
                int retCode = event.getArg2();

#ifdef AIUI_V2
                string dataTypeStr;
                string text;

                if (dtype == AIUIConstant::SYNC_DATA_UPLOAD)
                {
                    dataTypeStr = "上传实体";
                }
                else if (dtype == AIUIConstant::SYNC_DATA_DELETE)
                {
                    dataTypeStr = "删除实体";
                }
                else if (dtype == AIUIConstant::SYNC_DATA_DOWNLOAD)
                {
                    dataTypeStr = "下载实体";
                }
                else if (dtype == AIUIConstant::SYNC_DATA_SEE_SAY)
                {
                    dataTypeStr = "所见即可说";
                }

                if (AIUIConstant::SUCCESS == retCode)
                {
                    // 上传成功，会话的唯一id，用于反馈问题的日志索引字段，注意留存
                    // 注：上传成功立即生效
                    gSyncSid = event.getData()->getString("sid", "");
                    // 获取上传调用时设置的自定义tag
                    string tag = event.getData()->getString("tag", "");
                    // 获取上传调用耗时，单位：ms
                    long timeSpent = event.getData()->getLong("time_spent", -1);
                    cout << "同步" << dataTypeStr << "成功"
                         << "，耗时：" << timeSpent
                         << "ms, sid=" + gSyncSid + "，tag=" + tag;
                    if (dtype == AIUIConstant::SYNC_DATA_UPLOAD)
                    {
                        cout << "，你可以试着说“打电话给刘德华“" << endl;
                    }
                    else
                    {
                        cout << endl;
                    }
                    // 实体内容
                    if (dtype == AIUIConstant::SYNC_DATA_DOWNLOAD)
                    {
                        text = event.getData()->getString("text", "");
                        cout << "下载的实体内容:\n"
                             << Base64Util::decode(text) << endl;
                    }
                }
                else
                {
                    gSyncSid = "";
                    string result = event.getData()->getString("result", "");
                    cout << "同步" << dataTypeStr << "失败，错误码：" << retCode << " info:" << event.getInfo() << " result:" << result << endl;
                }
#else
                if (dtype == AIUIConstant::SYNC_DATA_SCHEMA)
                {
                    if (AIUIConstant::SUCCESS == retCode)
                    {
                        // 上传成功，记录上传会话的sid，以用于查询数据打包状态
                        // 注：上传成功并不表示数据打包成功，打包成功与否应以同步状态查询结果为准，数据只有打包成功后才能正常使用
                        gSyncSid = event.getData()->getString("sid", "");

                        // 获取上传调用时设置的自定义tag
                        string tag = event.getData()->getString("tag", "");

                        // 获取上传调用耗时，单位：ms
                        long timeSpent = event.getData()->getLong("time_spent", -1);

                        cout << "同步成功，"
                             << "耗时：" << timeSpent
                             << "ms, sid=" + gSyncSid + "，tag=" + tag +
                                    "，你可以试着说“打电话给刘德华“"
                             << endl;
                    }
                    else
                    {
                        gSyncSid = "";
                        cout << "同步失败，错误码：" << retCode << endl;
                    }
                }
#endif
            }
#ifndef AIUI_V2
            else if (AIUIConstant::CMD_QUERY_SYNC_STATUS == event.getArg1())
            {
                // 数据同步状态查询的返回
                // 获取同步类型
                int syncType = event.getData()->getInt("sync_dtype", -1);
                if (AIUIConstant::SYNC_DATA_QUERY == syncType)
                {
                    // 若是同步数据查询，则获取查询结果，结果中error字段为0则表示上传数据打包成功，否则为错误码
                    string result = event.getData()->getString("result", "");

                    cout << "查询结果：" << result << endl;
                }
            }
#endif
        }
        break;

        // 开始录音事件
        case AIUIConstant::EVENT_START_RECORD:
        {
            cout << "EVENT_START_RECORD " << endl;
        }
        break;

        // 停止录音事件
        case AIUIConstant::EVENT_STOP_RECORD:
        {
            cout << "EVENT_STOP_RECORD " << endl;
        }
        break;

        // 出错事件
        case AIUIConstant::EVENT_ERROR:
        {
            // 打印错误码和描述信息
            cout << "EVENT_ERROR: error=" << event.getArg1() << ", des=" << event.getInfo() << endl;
        }
        break;

        // 连接到服务器
        case AIUIConstant::EVENT_CONNECTED_TO_SERVER:
        {
            // 获取uid（为客户端在云端的唯一标识）并打印
            string uid = event.getData()->getString("uid", "");

            cout << "EVENT_CONNECTED_TO_SERVER, uid=" << uid << endl;
        }
        break;

        // 与服务器断开连接
        case AIUIConstant::EVENT_SERVER_DISCONNECTED:
        {
            cout << "EVENT_SERVER_DISCONNECTED " << endl;
        }
        break;
        }
    }
};

IAIUIAgent *g_pAgent = nullptr;

#ifdef AIUI_ANDROID
#define TEST_ROOT_DIR "/sdcard/AIUI/"
#
#ifdef TURING_UNIT_SUPPORT
#define CFG_FILE_PATH "/sdcard/AIUI/cfg/turing.cfg"
#else
#define CFG_FILE_PATH "/sdcard/AIUI/cfg/aiui.cfg"
#endif
#
#define TEST_AUDIO_PATH "/sdcard/AIUI/audio/test.pcm"
#define LOG_DIR "/sdcard/AIUI/log/"
#define MSC_DIR "/sdcard/AIUI/msc/"
#define TEST_TTS_PATH "/sdcard/AIUI/text/tts.txt"
#define TEST_SEE_SAY_PATH "/sdcard/AIUI/text/see_say.txt"
#define VOICE_CLONE_AUDIO_PATH "/sdcard/AIUI/audio/voice_clone_1ch24K16bit.pcm"
#define VOICE_CLONE_RES_ID_PATH "/sdcard/AIUI/voice_clone_reg_id.txt"
#else
#define TEST_ROOT_DIR "./AIUI/"
#
#ifdef TURING_UNIT_SUPPORT
#define CFG_FILE_PATH "./AIUI/cfg/turing.cfg"
#else
#define CFG_FILE_PATH "/home/sss/robot_voice/src/robot_aiui/AIUI/cfg/aiui.cfg"
#endif
#
#define TEST_AUDIO_PATH "./AIUI/audio/test.pcm"
#define LOG_DIR "./AIUI/log/"
#define MSC_DIR "./AIUI/msc/"
#define TEST_TTS_PATH "./AIUI/text/tts.txt"
#define TEST_SEE_SAY_PATH "./AIUI/text/see_say.txt"
#define VOICE_CLONE_AUDIO_PATH "./AIUI/audio/voice_clone_1ch24K16bit.pcm"
#define VOICE_CLONE_RES_ID_PATH "./voice_clone_reg_id.txt"
#endif

// 通讯录同步示例内容
string SYNC_CONTACT_CONTENT = // NOLINT
    R"({"name":"刘得华", "phoneNumber":"13512345671"})"
    "\n"
    R"({"name":"张学诚", "phoneNumber":"13512345672"})"
    "\n"
    R"({"name":"张右兵", "phoneNumber":"13512345673"})"
    "\n"
    R"({"name":"吴羞波", "phoneNumber":"13512345674"})"
    "\n"
    R"({"name":"黎晓", "phoneNumber":"13512345675"})";

/**
 * 读取文件内容存到字符串。
 *
 * @param path
 * @return
 */
string readFileAsString(const string &path)
{
    ifstream t(path, ios_base::in | ios::binary);
    if (!t.is_open())
    {
        std::cout << "Error open file: " << path << " fail.";
    }
    string str((istreambuf_iterator<char>(t)), istreambuf_iterator<char>());

    return str;
}

#define SEND_AIUIMESSAGE(cmd, arg1, arg2, params, data)                          \
    do                                                                           \
    {                                                                            \
        if (!g_pAgent)                                                           \
            break;                                                               \
        IAIUIMessage *msg = IAIUIMessage::create(cmd, arg1, arg2, params, data); \
        g_pAgent->sendMessage(msg);                                              \
        msg->destroy();                                                          \
    } while (false)

#define SEND_AIUIMESSAGE4(cmd, arg1, arg2, params) SEND_AIUIMESSAGE(cmd, arg1, arg2, params, nullptr)
#define SEND_AIUIMESSAGE3(cmd, arg1, arg2) SEND_AIUIMESSAGE4(cmd, arg1, arg2, "")
#define SEND_AIUIMESSAGE2(cmd, arg1) SEND_AIUIMESSAGE3(cmd, arg1, 0)
#define SEND_AIUIMESSAGE1(cmd) SEND_AIUIMESSAGE2(cmd, 0)

/**
 * 创建AIUIAgent对象。
 *
 * @param more
 * @param cfgPath
 */
void createAgent(bool more = true, const char *cfgPath = CFG_FILE_PATH)
{
    if (g_pAgent)
    {
        return;
    }

    string aiuiParams = readFileAsString(cfgPath);

    Json::Value paramJson;
    Json::Reader reader;
    if (reader.parse(aiuiParams, paramJson, false))
    {
        if (more)
        {
            cout << paramJson.toString() << endl;
        }

        DemoListener::getInstance().mMoreDetails = more;
        g_pAgent = IAIUIAgent::createAgent(paramJson.toString().c_str(), &DemoListener::getInstance());
    }

    if (!g_pAgent)
    {
        std::cout << string(cfgPath) << ", " << reader.getFormatedErrorMessages() << std::endl;
        return;
    }
}

/**
 * 销毁AIUIAgent对象。
 */
void destroyAgent()
{
    if (g_pAgent)
    {
        g_pAgent->destroy();
        g_pAgent = nullptr;
    }
}

/**
 * 唤醒AIUI。
 */
void wakeup()
{
    // 可以通过clear_data来控制是否要清除唤醒之前的数据（默认会清除），清除则唤醒之前的会话结果（tts除外）会被丢弃从而不再继续抛出
    SEND_AIUIMESSAGE4(AIUIConstant::CMD_WAKEUP, 0, 0, "clear_data=true");
}

/**
 * 重置唤醒，即回到待唤醒状态。
 */
void resetWakeup()
{
    SEND_AIUIMESSAGE1(AIUIConstant::CMD_RESET_WAKEUP);
}

/**
 * 开启AIUI服务，此接口是与stop()对应，调用stop()之后必须调用此接口才能继续与SDK交互。
 *
 * 注：AIUIAgent创建成功之后AIUI会自动开启，故若非调用过stop()则不需要调用start()。
 */
void start()
{
    SEND_AIUIMESSAGE1(AIUIConstant::CMD_START);
}

/**
 * 停止AIUI服务。
 */
void stop()
{
    SEND_AIUIMESSAGE1(AIUIConstant::CMD_STOP);
}

/**
 * 重置AIUI服务，相当于先调用stop()再调用start()。一般用不到。
 */
void resetAIUI()
{
    SEND_AIUIMESSAGE(AIUIConstant::CMD_RESET, 0, 0, "", nullptr);
}

/**
 * 从文件读音频写入SDK，即用文件数据模型实时录音数据。
 *
 * @param repeat
 */
void writeAudioFromLocal(bool repeat)
{
    if (!g_pAgent)
    {
        return;
    }

    ifstream testData(TEST_AUDIO_PATH, std::ios::in | std::ios::binary);

    if (testData.is_open())
    {
        testData.seekg(0, std::ios::end);
        int total = testData.tellg();
        testData.seekg(0, std::ios::beg);

        char *audio = new char[total];
        testData.read(audio, total);
        testData.close();

        int offset = 0;
        int left = total;
        const int frameLen = 1280;
        char buff[frameLen];

        while (true)
        {
            if (left < frameLen)
            {
                if (repeat)
                {
                    offset = 0;
                    left = total;
                    continue;
                }
                else
                {
                    break;
                }
            }

            memset(buff, '\0', frameLen);
            memcpy(buff, audio + offset, frameLen);

            offset += frameLen;
            left -= frameLen;

            // frameData内存会在Message在内部处理完后自动release掉
            AIUIBuffer frameData = aiui_create_buffer_from_data(buff, frameLen);
            SEND_AIUIMESSAGE(AIUIConstant::CMD_WRITE, 0, 0, "data_type=audio,tag=audio-tag", frameData);

            // 必须暂停一会儿模拟人停顿，太快的话后端报错。1280字节16k采样16bit编码的pcm数据对应40ms时长
            AIUI_SLEEP(40);
        }

        // 音频写完后，要发CMD_STOP_WRITE停止写入消息
        SEND_AIUIMESSAGE4(AIUIConstant::CMD_STOP_WRITE, 0, 0, "data_type=audio");

        delete[] audio;
    }
    else
    {
        cout << "open file failed, path=" << TEST_AUDIO_PATH << endl;
    }

    cout << "write finish" << endl;
}

/**
 * 开启录音。
 */
void startRecordAudio()
{
    SEND_AIUIMESSAGE4(
        AIUIConstant::CMD_START_RECORD, 0, 0, "data_type=audio,pers_param={\"uid\":\"\"},tag=record-tag");
}

/**
 * 停止录音。
 */
void stopRecordAudio()
{
    SEND_AIUIMESSAGE1(AIUIConstant::CMD_STOP_RECORD);
}

/**
 * 写入文本进行交互。
 *
 * @param text 文本内容
 * @param needWakeup 是否需要唤醒
 */
void writeText(const string &text, bool needWakeup = true)
{
    AIUIBuffer textData = aiui_create_buffer_from_data(text.c_str(), text.length());

    if (needWakeup)
    {
        SEND_AIUIMESSAGE(AIUIConstant::CMD_WRITE, 0, 0, "data_type=text,pers_param={\"uid\":\"\"}", textData);
    }
    else
    {
        SEND_AIUIMESSAGE(AIUIConstant::CMD_WRITE, 0, 0, "data_type=text,need_wakeup=false", textData);
    }
}

void triggerCaptureServiceAgent() {
    DemoListener::getInstance().callTriggerCaptureService();
    
   
}

bool locateObjectSyncAgent(const std::string& target, std::string& locateMsg) {
    DemoListener::getInstance().publichTargetObject(target);
    return DemoListener::getInstance().callLocateObjectSync(locateMsg);
}

std::string getLatestImageDescription() {
    return DemoListener::getInstance().waitForImageDescription();
}

// ===== 发布检测目标 =====
void publishTargetObject(const std::string& target) {
    DemoListener::getInstance().publichTargetObject(target);
}

// ===== 获取vision_3DLocation的最近一次坐标（供 agent 模式直接调用）=====
    void getLast3DLocation(double &x, double &y, double &z) {
        x =  DemoListener::getInstance().last_3d_x_;
        y = DemoListener::getInstance().last_3d_y_;
        z = DemoListener::getInstance().last_3d_z_;
    }

/**
 * 测试语音合成，返回pcm数据。
 *
 * @param text
 */
void startTTS(const string &text, const string &tag)
{
    // 检查 agent 是否就绪
    if (!g_pAgent)
    {
        std::cerr << "[startTTS] ERROR: g_pAgent is NULL, TTS cannot proceed!" << std::endl;
        return;
    }

    AIUIBuffer textData = aiui_create_buffer_from_data(text.c_str(), text.length());
    // 使用与 aiui.cfg 一致的发音人 x5_lingxiaoyue_flow
    string params = "voice_name=x5_lingxiaoyue_flow";
    if (!tag.empty())
    {
        params.append(",tag=").append(tag);
    }

    std::cout << "[startTTS] sending TTS: text=" << text << " params=" << params << std::endl;
    SEND_AIUIMESSAGE(AIUIConstant::CMD_TTS, AIUIConstant::START, 0, params.c_str(), textData);
}

/**
 * 测试超拟人语音合成，返回pcm数据。
 *
 * @param text
 */
void startHTS(const string &text, const string &tag)
{
    AIUIBuffer textData = aiui_create_buffer_from_data(text.c_str(), text.length());

    // 超拟人合成需要设置scene=IFLYTEK.hts
    string params = "voice_name=x4_lingxiaoxuan_oral,scene=IFLYTEK.hts";
    if (!tag.empty())
    {
        params.append(",tag=").append(tag);
    }

    SEND_AIUIMESSAGE(AIUIConstant::CMD_TTS, AIUIConstant::START, 0, params.c_str(), textData);
}

/**
 * 测试语音合成，返回url。
 *
 * @param text
 */
/**void startTTSUrl(const string& text)
{
    AIUIBuffer textData = aiui_create_buffer_from_data(text.c_str(), text.length());

    // 使用发音人chongchong合成，也可以使用其他发音人
    SEND_AIUIMESSAGE(AIUIConstant::CMD_TTS, AIUIConstant::START, 0,
                     "text_encoding=utf-8,tts_res_type=url,vcn=x2_xiaojuan", textData);
}
**
/**
 * 构建asr语法。
 *
 * 注：当前版本已废弃，只有历史版本支持。
 */
void buildAsrGrammar()
{
    string grammar = readFileAsString("AIUI/asr/call.bnf");

    SEND_AIUIMESSAGE4(AIUIConstant::CMD_BUILD_GRAMMAR, 0, 0, grammar.c_str());
}

/**
 * 构建esr语法。
 *
 * 注：新版本SDK都只支持esr。
 */
void buildEsrGrammar()
{
    string grammar = readFileAsString("AIUI/esr/message.fsa");

    SEND_AIUIMESSAGE4(AIUIConstant::CMD_BUILD_GRAMMAR, 0, 0, grammar.c_str());
}

/**
 * 将麦克风参数切换到单麦设置。
 */
void changeMicTypeToMic1()
{
    constexpr const char *mic1_params = R"(
{
	"ivw": {
		"mic_type": "mic1"
	},
	"recorder": {
		"channel_filter": "0,-1"
	}
}
)";
    SEND_AIUIMESSAGE(AIUIConstant::CMD_SET_PARAMS, 0, 0, mic1_params, nullptr);
}

/**
 * 清除语义对话历史。
 */
void cleanDialogHistory()
{
    cout << "cleanDialogHistory" << endl;

    SEND_AIUIMESSAGE1(AIUIConstant::CMD_CLEAN_DIALOG_HISTORY);
}

/**
 * 同步动态实体。
 */
void syncSchemaData(int type = AIUIConstant::SYNC_DATA_SCHEMA)
{
    string dataStrBase64 = Base64Util::encode(SYNC_CONTACT_CONTENT);

    Json::Value syncSchemaJson;
    Json::Value dataParamJson;

    // 设置id_name为uid，即用户级个性化资源
    // 个性化资源使用方法可参见http://doc.xfyun.cn/aiui_mobile/的用户个性化章节
    dataParamJson["id_name"] = "uid";

    // 设置res_name为联系人
    dataParamJson["res_name"] = "IFLYTEK.telephone_contact";

#ifdef AIUI_V2
    // aiui开放平台的命名空间，在「技能工作室-我的实体-动态实体密钥」中查看
    dataParamJson["name_space"] = "OS13360977719";
#endif

    syncSchemaJson["param"] = dataParamJson;
    if (AIUIConstant::SYNC_DATA_SCHEMA == type || AIUIConstant::SYNC_DATA_UPLOAD == type)
    {
        syncSchemaJson["data"] = dataStrBase64;
    }

    string jsonStr = syncSchemaJson.toString();

    // 传入的数据一定要为utf-8编码
    AIUIBuffer syncData = aiui_create_buffer_from_data(jsonStr.c_str(), jsonStr.length());

    // 给该次同步加上自定义tag，在返回结果中可通过tag将结果和调用对应起来
    Json::Value paramJson;
    paramJson["tag"] = "sync-tag";

    // 用schema数据同步上传联系人
    // 注：数据同步请在连接服务器之后进行，否则可能失败
    SEND_AIUIMESSAGE(AIUIConstant::CMD_SYNC, type, 0,
                     paramJson.toString().c_str(), syncData);
}

#ifdef AIUI_V2
/**
 * 同步所见即可说的内容。
 */
void syncV2SeeSayData()
{
    string seeSayContent = readFileAsString(TEST_SEE_SAY_PATH);
    Json::Value contentJson;
    Json::Reader reader;
    if (!reader.parse(seeSayContent, contentJson, false))
    {
        cout << "syncV2SeeSayData parse error! info=" << seeSayContent << endl;
        return;
    }
    /*注意：传入的数据一定要为utf-8编码
     * 　　可见即可说的内容为json数组
     [{
     　 //资源名称
　　　　"res_name": "IFLYTEK.telephone_contact",
       //内容要进行base64编码
　　　　"data": "base64"　
　　　}]
     */

    cout << "see say content: " << contentJson.asString() << endl;
    // 整个json在进行base64编码
    string dataStrBase64 = Base64Util::encode(contentJson.asString());
    Json::Value syncSeeSayJson;
    syncSeeSayJson["data"] = dataStrBase64;
    string jsonStr = syncSeeSayJson.toString();
    AIUIBuffer syncData = aiui_create_buffer_from_data(jsonStr.c_str(), jsonStr.length());

    // 给该次同步加上自定义tag，在返回结果中可通过tag将结果和调用对应起来
    Json::Value paramJson;
    paramJson["tag"] = "sync_see_say_tag";

    // 注：数据同步请在连接服务器之后进行，否则可能失败
    SEND_AIUIMESSAGE(AIUIConstant::CMD_SYNC, AIUIConstant::SYNC_DATA_SEE_SAY, 0,
                     paramJson.toString().c_str(), syncData);
}

/**
 * 上传需要复刻的音频资源
 * 音频格式:　
 *    采样率: 24000　
 *    通道数: 1　
 *    位深: 16　
 *    编码格式: 裸音频pcm
 */
void voiceCloneReg(string resPath = VOICE_CLONE_AUDIO_PATH)
{
    Json::Value paramJson;
    paramJson["tag"] = "voice_clone_tag_0";
    paramJson["res_path"] = resPath;

    cout << "[func:" << __FUNCTION__ << " line:" << __LINE__ << "] "
         << "上传声音复刻的资源,资源路径: " << resPath << endl;

    // 注：数据同步请在连接服务器之后进行，否则可能失败
    SEND_AIUIMESSAGE(AIUIConstant::CMD_CLONE_VOICE, AIUIConstant::VOICE_CLONE_REG, 0,
                     paramJson.toString().c_str(),
                     nullptr);
}

/**
 * 删除声音复刻的资源
 */
void voiceCloneDelRes()
{
    string resId = gVoiceCloneResId;
    if (resId.empty())
    {
        resId = readFileAsString(VOICE_CLONE_RES_ID_PATH);
    }

    if (resId.empty())
    {
        cout << "[fail func:" << __FUNCTION__ << " line:" << __LINE__ << "] "
             << "删除声音复刻的资源失败，资源ID为NULL" << endl;
        return;
    }

    cout << "[func:" << __FUNCTION__ << " line:" << __LINE__ << "] "
         << "删除声音复刻的资源,res id = " << resId << endl;

    Json::Value paramJson;
    paramJson["tag"] = "voice_clone_tag_１";
    paramJson["res_id"] = resId;

    // 注：数据同步请在连接服务器之后进行，否则可能失败
    SEND_AIUIMESSAGE(AIUIConstant::CMD_CLONE_VOICE, AIUIConstant::VOICE_CLONE_DEL, 0,
                     paramJson.toString().c_str(),
                     nullptr);
}

/**
 * 测试声音复刻的tts
 *
 * @param text
 */
void startVoiceCloneTTS(const string &text, const string &tag)
{
    string resId = gVoiceCloneResId;
    if (resId.empty())
    {
        resId = readFileAsString(VOICE_CLONE_RES_ID_PATH);
    }

    if (resId.empty())
    {
        cout << "[fail func:" << __FUNCTION__ << " line:" << __LINE__ << "] "
             << "请求声音复刻的tts失败，资源ID为NULL" << endl;
        return;
    }

    AIUIBuffer textData = aiui_create_buffer_from_data(text.c_str(), text.length());
    // 发言人必须是x5_clone
    string params = "voice_name=x5_clone";
    params.append(",res_id=").append(resId);
    if (!tag.empty())
    {
        params.append(",tag=").append(tag);
    }

    SEND_AIUIMESSAGE(AIUIConstant::CMD_TTS, AIUIConstant::START, 0, params.c_str(), textData);
}

#endif

/**
 * 查询动态实体同步状态。
 */
void querySyncSchemaStatus()
{
    // 构造查询json字符串，填入同步schema数据返回的sid
    Json::Value queryJson;
    queryJson["sid"] = gSyncSid;

    // 发送同步数据状态查询消息，设置arg1为schema数据类型，params为查询字符串
    SEND_AIUIMESSAGE4(AIUIConstant::CMD_QUERY_SYNC_STATUS,
                      AIUIConstant::SYNC_DATA_SCHEMA,
                      0,
                      queryJson.toString().c_str());
}

/**
 * 同步可见即可说数据。
 */
void syncSpeakableData() {}

#if defined(__linux) || defined(__ANDROID__)
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>

/**
 * 获取mac地址。
 *
 * @param mac
 */
static void GenerateMACAddress(char *mac)
{
    // reference: https://stackoverflow.com/questions/1779715/how-to-get-mac-address-of-your-machine-using-a-c-program/35242525
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        return;
    };

    struct ifconf ifc{};
    char buf[1024];
    int success = 0;

    ifc.ifc_len = sizeof(buf);
    ifc.ifc_buf = buf;
    if (ioctl(sock, SIOCGIFCONF, &ifc) == -1)
    {
        return;
    }

    struct ifreq *it = ifc.ifc_req;
    const struct ifreq *const end = it + (ifc.ifc_len / sizeof(struct ifreq));
    struct ifreq ifr{};

    for (; it != end; ++it)
    {
        strcpy(ifr.ifr_name, it->ifr_name);
        if (ioctl(sock, SIOCGIFFLAGS, &ifr) == 0)
        {
            if (!(ifr.ifr_flags & IFF_LOOPBACK))
            { // don't count loopback
                if (ioctl(sock, SIOCGIFHWADDR, &ifr) == 0)
                {
                    success = 1;
                    break;
                }
            }
        }
        else
        {
            return;
        }
    }

    unsigned char mac_address[6];
    if (success)
        memcpy(mac_address, ifr.ifr_hwaddr.sa_data, 6);

    sprintf(mac,
            "%02x:%02x:%02x:%02x:%02x:%02x",
            mac_address[0],
            mac_address[1],
            mac_address[2],
            mac_address[3],
            mac_address[4],
            mac_address[5]);
    close(sock);
}
#elif defined(WIN32)
#include <stdio.h>
#include <IPHlpApi.h>
#pragma comment(lib, "IPHLPAPI.lib")

static void GenerateMACAddress(char *mac)
{
    ULONG ulSize = 0;
    PIP_ADAPTER_INFO adapterInfo = NULL;
    PIP_ADAPTER_INFO adapterInfoTmp = NULL;

    GetAdaptersInfo(adapterInfo, &ulSize);

    if (0 == ulSize)
    {
        return;
    }

    adapterInfo = (PIP_ADAPTER_INFO)malloc(ulSize);

    if (adapterInfo == NULL)
    {
        return;
    }

    memset(adapterInfo, 0, ulSize);

    adapterInfoTmp = adapterInfo;

    GetAdaptersInfo(adapterInfo, &ulSize);

    while (adapterInfo)
    {
        _snprintf(mac,
                  64,
                  "%02x:%02x:%02x:%02x:%02x:%02x",
                  adapterInfo->Address[0],
                  adapterInfo->Address[1],
                  adapterInfo->Address[2],
                  adapterInfo->Address[3],
                  adapterInfo->Address[4],
                  adapterInfo->Address[5]);

        if (std::strcmp(adapterInfo->GatewayList.IpAddress.String, "0.0.0.0") != 0)
            break;
        e4e4
            adapterInfo = adapterInfo->Next;
    }

    free(adapterInfoTmp);

    adapterInfoTmp = NULL;
}
#endif

/**
 * 初始化设置。
 *
 * @param log
 */
static void initSetting(bool log = true)
{
    AIUISetting::setAIUIDir(TEST_ROOT_DIR);
    AIUISetting::setMscDir(MSC_DIR);
    AIUISetting::setNetLogLevel(log ? aiui_debug : aiui_none);

    char mac[64] = {0};
    GenerateMACAddress(mac);

    // 为每一个设备设置唯一对应的序列号SN（最好使用设备硬件信息(mac地址，设备序列号等）生成），以便正确统计装机量，
    // 避免刷机或者应用卸载重装导致装机量重复计数
    AIUISetting::setSystemInfo(AIUI_KEY_SERIAL_NUM, mac);

    // 6.6.xxxx.xxxx版本设置用户唯一标识uid（可选，AIUI后台服务需要，不设置则会使用上面的SN作为uid）
    // 5.6.xxxx.xxxx版本SDK不能也不需要设置uid
    // AIUISetting::setSystemInfo(AIUI_KEY_UID, "1234567890");
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, ""); // 解决 ROS_INFO 中文输出乱码的问题
    ros::init(argc, argv, "aiui_ros_node");
    ros::NodeHandle n;
    ros::ServiceClient navGoalClient;
    navGoalClient = n.serviceClient<robot_aiui::NavigateToOffice>("/kybot_navgoali");
#ifdef WIN32
    system("chcp 65001 >nul");
    // freopen("nul", "w", stderr);  // 调试期间注释掉，以便查看SDK错误信息
#else
    freopen("/dev/null", "w", stderr);  // 调试期间注释掉，以便查看SDK错误信息
#endif
    image_pub = n.advertise<std_msgs::String>("image_file_path", 10);
    DemoListener::getInstance();  // 触发单例初始化（必须在 ros::init 之后）

    // 打印SDK版本
    std::cout << "Version: " << getVersion() << std::endl;

    initSetting();

    // // 启动 AIUI 代理
    createAgent();

    startRecordAudio(); // 启动录音
    cout << "----等待唤醒中----" << endl;
    ros::Rate loop_rate(10); // 10 Hz
    while (ros::ok())
    {
        // ROS 节点处理
        ros::spinOnce();
        loop_rate.sleep();
    }
    // destroyAgent();
    return 0;
}