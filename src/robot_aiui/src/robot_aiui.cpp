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
#include <rclcpp/rclcpp.hpp>
#include "aiui/AIUI_V2.h"
#include "aiui/PcmPlayer_C.h"
#include "json/json.h"
#include "utils/StreamNlpTtsHelper.h"
#include "utils/IatResultUtil.h"
#include "utils/Base64Util.h"
#include "std_msgs/msg/string.hpp"
#include "robot_aiui/srv/navigate_to_office.hpp" // 导入自定义服务类型
#include "geometry_msgs/msg/pose2_d.hpp"
#include "kybot_msgs/srv/pick_and_place.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <thread>
#include <atomic>
#include "hpp/agent_bridge.h"
#include <regex>
#include <map>
#include <memory>
#include <exception>
#include <cmath>
#include <vector>
#include "hpp/agent_bridge.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

// 是否使用AIUI V2服务（交互大模型）
#define AIUI_V2

// 是否使用语义后合成。当在AIUI平台应用配置页面打开"语音合成"开关时，需要打开该宏
// #define USE_POST_SEMANTIC_TTS

using namespace std;
using namespace aiui_va;
using namespace aiui_v2;

// Forward declaration
class DemoListener;

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr image_pub;
std::weak_ptr<DemoListener> g_node_weak;

enum TaskState
{
    IDLE,
    NAVIGATING,
    PICKING,
    PLACE,
    NAVIGATING0
};
TaskState currentState = IDLE; // 状态管理

std::string gSyncSid;
std::string gVoiceCloneResId;

void startTTS(const string &text, const string &tag = "");
void startRecordAudio();
void stopRecordAudio();

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
}

// 新增：发布图片路径的函数
void publishImagePath()
{
    RCLCPP_INFO(rclcpp::get_logger("aiui"), "发布图片路径");

    std_msgs::msg::String img_msg;
    img_msg.data = "/home/sss/qwen_vision/src/qwen_vision/picture/2.jpg";
    image_pub->publish(img_msg);

    RCLCPP_INFO(rclcpp::get_logger("aiui"), "已发布图片地址: %s", img_msg.data.c_str());
}

// 抓取服务调用函数
void callYoloPick(int mode)
{
    auto node = std::make_shared<rclcpp::Node>("_pick_caller");
    auto pickClient = node->create_client<kybot_msgs::srv::PickAndPlace>("/yolo_pick");
    if (!pickClient->wait_for_service(std::chrono::seconds(3)))
    {
        RCLCPP_WARN(rclcpp::get_logger("aiui"), "YOLO pick service not available!");
        return;
    }

    auto srv = std::make_shared<kybot_msgs::srv::PickAndPlace::Request>();
    srv->mode = mode;

    auto future = pickClient->async_send_request(srv);
    rclcpp::spin_until_future_complete(node, future);
    auto result = future.get();
    if (result->success)
    {
        cout << "抓取指令已发送，模式: " << mode << endl;
        if (currentState == PICKING)
        {
            currentState = NAVIGATING0;
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
    auto node = std::make_shared<rclcpp::Node>("_place_caller");
    auto placeClient = node->create_client<kybot_msgs::srv::PickAndPlace>("/yolo_place");
    if (!placeClient->wait_for_service(std::chrono::seconds(3)))
    {
        RCLCPP_WARN(rclcpp::get_logger("aiui"), "YOLO place service not available!");
        return;
    }

    auto srv = std::make_shared<kybot_msgs::srv::PickAndPlace::Request>();
    srv->mode = mode;

    auto future = placeClient->async_send_request(srv);
    rclcpp::spin_until_future_complete(node, future);
    auto result = future.get();
    if (result->success)
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
    auto node = std::make_shared<rclcpp::Node>("_nav_caller");
    auto navGoalClient = node->create_client<robot_aiui::srv::NavigateToOffice>("/kybot_navgoali");

    auto srv = std::make_shared<robot_aiui::srv::NavigateToOffice::Request>();
    geometry_msgs::msg::Pose2D pose;
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
    srv->pose = pose;

    if (!navGoalClient->wait_for_service(std::chrono::seconds(3)))
    {
        RCLCPP_WARN(rclcpp::get_logger("aiui"), "Nav service /kybot_navgoali not available!");
        return false;
    }

    auto future = navGoalClient->async_send_request(srv);
    rclcpp::spin_until_future_complete(node, future);
    auto result = future.get();
    if (result->success)
    {
        RCLCPP_INFO(rclcpp::get_logger("aiui"), "导航请求发送成功: (%.3f, %.3f, %.3f)", x, y, theta);
        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("aiui"), "导航请求失败");
        return false;
    }
}

// 从IAIUIListener派生自己的结果监听器
class DemoListener : public IAIUIListener, public rclcpp::Node
{

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr image_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_capture_client_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr prompt_update_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_object_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr locate_sync_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_3DLocation_sub_;

    std::atomic<bool> task_running_{false};
    std::atomic<int> last_target_mode_{1};

    std::string description;

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
        if (target_key == "bottle" || target_key == "water bottle" || target_key == "water_bottle")
            return 5;
        return 1;
    }

    static std::string targetToVisionName(const std::string &target_key)
    {
        if (target_key == "bottle")
            return "water bottle";
        return target_key;
    }

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
        }
        string lastTtsText;
        void onFinish(const string &fullText) override
        {
            stopRecordAudio();
            if (fullText == lastTtsText)
            {
                return;
            }
            lastTtsText = fullText;
            startRecordAudio();
        }

        void onTtsData(const Json::Value &bizParamJson, const char *audio, int len) override
        {
            const Json::Value &data = (bizParamJson["data"])[0];
            const Json::Value &content = (data["content"])[0];
            int dts = content["dts"].asInt();
            int progress = content["text_percent"].asInt();

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
    Goal home_{0.013, 0.088, 0.132};

    bool loadGoalsFromParam()
    {
        auto param_names = this->list_parameters({"goals"}, 10).names;
        if (param_names.empty())
        {
            RCLCPP_WARN(this->get_logger(), "~goals not found. Using default hardcoded goals/home.");
            return false;
        }

        goals_.clear();
        // Collect goal keys (e.g., "goals.office.x", "goals.office.y", "goals.office.th")
        std::map<std::string, Goal> parsed;
        for (const auto &name : param_names)
        {
            // name = "goals.<key>.<field>"
            auto rest = name.substr(6); // skip "goals."
            auto dot = rest.find('.');
            if (dot == std::string::npos)
                continue;
            std::string key = rest.substr(0, dot);
            std::string field = rest.substr(dot + 1);
            double val = this->get_parameter(name).as_double();
            if (field == "x")
                parsed[key].x = val;
            else if (field == "y")
                parsed[key].y = val;
            else if (field == "th")
                parsed[key].th = val;
        }
        goals_ = parsed;

        // home（可选）
        auto home_names = this->list_parameters({"home"}, 10).names;
        if (!home_names.empty())
        {
            for (const auto &name : home_names)
            {
                auto field = name.substr(5); // skip "home."
                double val = this->get_parameter(name).as_double();
                if (field == "x")
                    home_.x = val;
                else if (field == "y")
                    home_.y = val;
                else if (field == "th")
                    home_.th = val;
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "~home not found or invalid, keep default home");
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu goals from YAML. home=(%.3f, %.3f, %.3f)",
                     goals_.size(), home_.x, home_.y, home_.th);
        return !goals_.empty();
    }

    // ===== [ADD] 采摘任务点位 =====
    std::map<std::string, Goal> harvest_points_;
    int harvest_max_per_point_ = 20;
    double same_pose_eps_ = 0.03;
    int same_pose_max_times_ = 3;

    bool loadHarvestPointsFromParam()
    {
        auto param_names = this->list_parameters({"harvest_points"}, 10).names;
        if (param_names.empty())
        {
            RCLCPP_WARN(this->get_logger(), "~harvest_points not found. Harvest task will NOT run until configured.");
            return false;
        }

        harvest_points_.clear();
        std::map<std::string, Goal> parsed;
        for (const auto &name : param_names)
        {
            // name = "harvest_points.<key>.<field>"
            auto rest = name.substr(15); // skip "harvest_points."
            auto dot = rest.find('.');
            if (dot == std::string::npos)
                continue;
            std::string key = rest.substr(0, dot);
            std::string field = rest.substr(dot + 1);
            double val = this->get_parameter(name).as_double();
            if (field == "x")
                parsed[key].x = val;
            else if (field == "y")
                parsed[key].y = val;
            else if (field == "th")
                parsed[key].th = val;
        }
        harvest_points_ = parsed;

        RCLCPP_INFO(this->get_logger(), "Loaded %zu harvest_points from YAML.", harvest_points_.size());
        return !harvest_points_.empty();
    }

    bool loadHarvestConfigFromParam()
    {
        this->declare_parameter("harvest_max_per_point", harvest_max_per_point_);
        this->declare_parameter("same_pose_eps", same_pose_eps_);
        this->declare_parameter("same_pose_max_times", same_pose_max_times_);

        harvest_max_per_point_ = this->get_parameter("harvest_max_per_point").as_int();
        same_pose_eps_ = this->get_parameter("same_pose_eps").as_double();
        same_pose_max_times_ = this->get_parameter("same_pose_max_times").as_int();

        if (harvest_max_per_point_ < 1)
            harvest_max_per_point_ = 1;
        if (same_pose_eps_ <= 0.0)
            same_pose_eps_ = 0.01;
        if (same_pose_max_times_ < 1)
            same_pose_max_times_ = 1;

        RCLCPP_INFO(this->get_logger(), "Harvest config: harvest_max_per_point=%d, same_pose_eps=%.4f, same_pose_max_times=%d",
                     harvest_max_per_point_, same_pose_eps_, same_pose_max_times_);
        return true;
    }

    static bool isHarvestIntent(const std::string &text)
    {
        return (text.find("开始采摘") != std::string::npos ||
                text.find("启动采摘") != std::string::npos ||
                text.find("开始摘") != std::string::npos ||
                text.find("采摘开始") != std::string::npos);
    }

    // 每个点位：循环"定位->抓->放(2)"直到没有目标
    void harvestAtPoint(const std::string &target_key, int mode)
    {
        std_msgs::msg::String tmsg;
        tmsg.data = targetToVisionName(target_key);
        target_object_pub_->publish(tmsg);

        last_target_mode_.store(mode);

        startTTS("开始识别并采摘");

        int picked = 0;

        double last_x = 1e9, last_y = 1e9, last_z = 1e9;
        int same_times = 0;

        for (int i = 0; i < harvest_max_per_point_; ++i)
        {
            target_object_pub_->publish(tmsg);

            std::string locateMsg;
            if (!callLocateObjectSync(locateMsg))
            {
                break;
            }

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
                    RCLCPP_WARN(this->get_logger(), "Harvest stuck: same target pose repeated.");
                    break;
                }
            }

            startTTS("发现目标，开始抓取");
            callYoloPick(mode);

            callYoloPlace(2);

            picked++;
            std::this_thread::sleep_for(std::chrono::milliseconds(600));
        }

        if (picked > 0)
            startTTS("该点采摘完成");
        else
            startTTS("未发现目标或目标已采摘完成");
    }

    void runHarvestTask()
    {
        const char *keys[4] = {"apple", "orange", "pear", "peach"};
        for (auto k : keys)
        {
            if (harvest_points_.find(k) == harvest_points_.end())
            {
                startTTS("采摘点位未配置，无法开始采摘任务");
                RCLCPP_ERROR(this->get_logger(), "Missing harvest_points key: %s", k);
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

        startTTS("采摘完成，返回起始点");
        navigateTo(home_.x, home_.y, home_.th);

        startTTS("任务完成");
    }

    // ===== 单例：禁止外部构造 =====
    DemoListener() : rclcpp::Node("aiui_ros_node")
    {
    }

public:
    void init()
    {
        image_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/vision_description", 10,
            std::bind(&DemoListener::VisionImageDescriptionCallback, this, std::placeholders::_1));

        trigger_capture_client_ = this->create_client<std_srvs::srv::Trigger>("/vision_trigger_capture");

        prompt_update_pub_ = this->create_publisher<std_msgs::msg::String>("/prompt_update", 10);

        target_object_pub_ = this->create_publisher<std_msgs::msg::String>("/target_object",
                                                                           rclcpp::QoS(1).transient_local());

        locate_sync_client_ = this->create_client<std_srvs::srv::Trigger>("/locate_object_sync");

        vision_3DLocation_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/object_position", 10,
            std::bind(&DemoListener::Vision3DLocationCallback, this, std::placeholders::_1));

        aiui_pcm_player_create();
        aiui_pcm_player_init();
        aiui_pcm_player_set_callbacks(
            onStarted, onPaused, onResumed, onStopped, onProgress, onError);
        aiui_pcm_player_start();

        std::shared_ptr<TtsHelperListener> listener = std::make_shared<TtsHelperListener>();
        m_pTtsHelper = std::make_shared<StreamNlpTtsHelper>(listener);
        m_pTtsHelper->setTextMinLimit(20);

        loadGoalsFromParam();
        loadHarvestPointsFromParam();
        loadHarvestConfigFromParam();
    }

    double last_3d_x_ = 0, last_3d_y_ = 0, last_3d_z_ = 0;

    // ===== 工厂方法 =====
    static std::shared_ptr<DemoListener> create()
    {
        auto instance = std::shared_ptr<DemoListener>(new DemoListener());
        instance->init();
        g_node_weak = instance;
        return instance;
    }

    DemoListener(const DemoListener &) = delete;
    DemoListener &operator=(const DemoListener &) = delete;

    void callTriggerCaptureService()
    {
        if (!trigger_capture_client_->wait_for_service(std::chrono::seconds(2)))
        {
            startTTS("视觉服务未就绪");
            std::cout << "视觉服务未就绪!" << endl;
            return;
        }
        auto srv = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = trigger_capture_client_->async_send_request(srv);
        rclcpp::spin_until_future_complete(this->shared_from_this(), future);
        auto result = future.get();
        if (result->success)
            startTTS("好的，我看到了");
        else
        {
            startTTS("拍照失败");
            std::cout << "拍照失败!" << endl;
        }
    }

    // ===== 返回图片描述（非阻塞，立即返回当前值）=====
    string getLatestImageDescription() const
    {
        return this->description;
    }

    // ===== 阻塞等待图片描述（用于 agent 模式同步获取视觉识别结果）=====
    std::string waitForImageDescription(double timeout_sec = 15.0)
    {
        this->description.clear();
        auto start = this->now();
        while (this->description.empty())
        {
            if ((this->now() - start).seconds() > timeout_sec)
            {
                RCLCPP_WARN(this->get_logger(), "waitForImageDescription: 超时（%.1f秒）未收到图像描述", timeout_sec);
                return "";
            }
            rclcpp::spin_some(this->shared_from_this());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO(this->get_logger(), "waitForImageDescription: 获取到描述，长度=%zu", this->description.size());
        return this->description;
    }

    // ===== 返回target_object_pub_指针（供 agent 模式直接发布目标）=====
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &getTargetObjectPublisher()
    {
        return target_object_pub_;
    }

    ~DemoListener()
    {
        aiui_pcm_player_destroy();
    }

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

    bool mMoreDetails = true;
    void processCommand(const string &commandText)
    {
        RCLCPP_INFO(this->get_logger(), "处理指令: %s", commandText.c_str());

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
        else if (commandText.find("观察") != string::npos ||
                 commandText.find("看看") != string::npos ||
                 commandText.find("查看") != string::npos)
        {
            callTriggerCaptureService();
        }
    }

    // ===== [ADD] 从用户话里提取目标物体 =====
    static std::string extractTarget(const std::string &text)
    {
        if (text.find("水蜜桃") != std::string::npos ||
            text.find("蜜桃") != std::string::npos ||
            text.find("桃子") != std::string::npos ||
            text.find("黄桃") != std::string::npos ||
            text.find("油桃") != std::string::npos)
            return "peach";

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

        if (text.find("水瓶") != std::string::npos ||
            text.find("矿泉水") != std::string::npos ||
            text.find("瓶装水") != std::string::npos ||
            text.find("饮用水") != std::string::npos ||
            text.find("一瓶水") != std::string::npos ||
            text.find("一瓶矿泉水") != std::string::npos)
            return "bottle";

        static const std::regex re_fetch_water(
            u8"(拿|取|抓|带|帮我拿|帮我取|帮我抓|给我|递给我|帮我带)[^，。！？\\s]{0,6}水(?!蜜桃|果)");

        if (std::regex_search(text, re_fetch_water))
            return "bottle";

        return "";
    }

    // ===== [ADD] 从用户话里提取目的地 =====
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

    // ===== [ADD] 是否是"取/拿/抓"的动作意图 =====
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
        if (!locate_sync_client_->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_WARN(this->get_logger(), "/locate_object_sync service not available!");
            return false;
        }

        auto srv = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = locate_sync_client_->async_send_request(srv);
        rclcpp::spin_until_future_complete(this->shared_from_this(), future);
        auto result = future.get();

        if (!result->success)
        {
            RCLCPP_WARN(this->get_logger(), "locate failed: %s", result->message.c_str());
            outJsonMsg = result->message;
            return false;
        }

        outJsonMsg = result->message;
        return true;
    }

    // ===== [ADD] 处理用户最终识别文本：入口函数 =====
    void handleUserUtterance(const std::string &userText)
    {
        if (agent::handleText(userText))
            return;

        std::string text = userText;
        RCLCPP_INFO(this->get_logger(), "用户指令(ASR最终): %s", text.c_str());
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
            startTTS("放置完成");
            return;
        }

        // 2) 先让"导航/观察/看看/查看"等非取物命令能工作
        if (!isFetchIntent(text))
        {
            std::cout << "=================没有检测到================" << endl;
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

        // 5) 不说目的地：就地"定位->抓取"
        if (dest.empty())
        {
            std::cout << "=================单独的拿起瓶子================" << endl;
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

        std_msgs::msg::String msg;
        msg.data = vision_target;
        target_object_pub_->publish(msg);

        startTTS("好的，我来找一下目标物体");

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

    void runFetchTask(const std::string &dest, const std::string &target_key)
    {
        int mode = targetToMode(target_key);
        last_target_mode_.store(mode);

        std::string vision_target = targetToVisionName(target_key);

        auto it = goals_.find(dest);
        if (it == goals_.end())
        {
            startTTS("我不知道这个目的地");
            RCLCPP_WARN(this->get_logger(), "Unknown destination key: %s", dest.c_str());
            return;
        }

        std_msgs::msg::String msg;
        msg.data = vision_target;
        target_object_pub_->publish(msg);

        startTTS("收到指令，开始前往目的地");

        const auto &g = it->second;
        if (!navigateTo(g.x, g.y, g.th))
        {
            startTTS("导航失败");
            return;
        }
        startTTS("已到达目的地，开始寻找目标物体");

        std::string locateMsg;
        if (!callLocateObjectSync(locateMsg))
        {
            startTTS("没有找到目标物体");
            return;
        }

        Json::Value root;
        Json::Reader reader;
        if (reader.parse(locateMsg, root, false))
        {
            std::string name = root.get("name", "").asString();
            std::string frame = root.get("frame_id", "").asString();
            double x = root.get("x", 0).asDouble();
            double y = root.get("y", 0).asDouble();
            double z = root.get("z", 0).asDouble();

            RCLCPP_INFO(this->get_logger(), "定位成功: name=%s frame=%s xyz=(%.3f,%.3f,%.3f)",
                         name.c_str(), frame.c_str(), x, y, z);
            startTTS("已定位到目标，开始抓取");
        }
        else
        {
            startTTS("已定位到目标，开始抓取");
        }

        callYoloPick(mode);

        startTTS("抓取完成，返回并放置");
        navigateTo(home_.x, home_.y, home_.th);
        callYoloPlace(mode);

        startTTS("任务完成");
    }

    //发布定位接口物体函数
    void publichTargetObject(const std::string &target_key)
    {
        std_msgs::msg::String msg;
        msg.data = targetToVisionName(target_key);
        target_object_pub_->publish(msg);
    }

private:
    fstream mFs;

    string mCurTtsSid;
    string mCurIatSid;
    string mIatTextBuffer;
    string mStreamNlpAnswerBuffer;
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

        if (shouldSynthesize && !answerText.empty())
        {
            startTTS(answerText);
        }
    }

    void VisionImageDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "收到图像描述: %s", msg->data.c_str());

        this->description = msg->data;
        if (this->description.length() > 1000)
        {
            this->description = this->description.substr(0, 1000) + "等";
        }

        startTTS(this->description, "image_description");
    }

    void Vision3DLocationCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        const auto &p = msg->pose.position;

        double dist = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        last_3d_x_ = p.x;
        last_3d_y_ = p.y;
        last_3d_z_ = p.z;

        RCLCPP_INFO(this->get_logger(), "Position [%s]: x=%.3f  y=%.3f  z=%.3f (dist=%.3f m)",
                     msg->header.frame_id.c_str(),
                     p.x, p.y, p.z,
                     dist);
    }

    void handleEvent(const IAIUIEvent &event)
    {
        switch (event.getEventType())
        {
        case AIUIConstant::EVENT_STATE:
        {
            switch (event.getArg1())
            {
            case AIUIConstant::STATE_IDLE:
            {
                cout << "EVENT_STATE: STATE_IDLE" << endl;
            }
            break;

            case AIUIConstant::STATE_READY:
            {
                cout << "EVENT_STATE: STATE_READY" << endl;
            }
            break;

            case AIUIConstant::STATE_WORKING:
            {
                cout << "EVENT_STATE: STATE_WORKING" << endl;
            }
            break;
            }
        }
        break;

        case AIUIConstant::EVENT_WAKEUP:
        {
            cout << "EVENT_WAKEUP: " << event.getInfo() << endl;
            aiui_pcm_player_stop();
            startTTS("我在");
        }
        break;

        case AIUIConstant::EVENT_SLEEP:
        {
            cout << "EVENT_SLEEP: arg1=" << event.getArg1() << endl;
        }
        break;

        case AIUIConstant::EVENT_VAD:
        {
            switch (event.getArg1())
            {
            case AIUIConstant::VAD_BOS_TIMEOUT:
            {
                cout << "EVENT_VAD: VAD_BOS_TIMEOUT" << endl;
            }
            break;

            case AIUIConstant::VAD_BOS:
            {
                cout << "EVENT_VAD: BOS" << endl;
            }
            break;

            case AIUIConstant::VAD_EOS:
            {
                cout << "EVENT_VAD: EOS" << endl;
            }
            break;

            case AIUIConstant::VAD_VOL:
            {
            }
            break;
            }
        }
        break;

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
            string sid = event.getData()->getString("sid", "");
            if (sub == "iat")
            {
                string cnt_id = content.get("cnt_id", "").asString();
                int dataLen = 0;
                const char *buffer = event.getData()->getBinary(cnt_id.c_str(), &dataLen);
                string resultStr = string(buffer, dataLen);
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
                        mIatTextBuffer.append(IatResultUtil::parseIatResult(textJson));
                    }
                    bool isLast = textJson["ls"].asBool();
                    if (isLast)
                    {
                        cout << "params: " << params.asString() << endl;
                        cout << "iat: " << mIatTextBuffer << endl;

                        DemoListener::getInstance().handleUserUtterance(mIatTextBuffer);

                        mIatTextBuffer.clear();
                    }
                }
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

            const char *buffer = event.getData()->getBinary(cnt_id.c_str(), &dataLen);

            if (sub == "tts")
            {
                Json::Value &&isUrl = content.get("url", empty);
                if (isUrl.asString() == "1")
                {
                    cout << "tts_url=" << string(buffer, dataLen) << endl;
                }
                else
                {
                    int progress = 0;
                    int dts = content["dts"].asInt();

                    string tag = event.getData()->getString("tag", "");
                    if (tag.find("stream_nlp_tts") == 0)
                    {
                        m_pTtsHelper->onOriginTtsData(tag, bizParamJson, buffer, dataLen);
                    }
                    else
                    {
                        if (dts == AIUIConstant::DTS_BLOCK_FIRST || dts == AIUIConstant::DTS_ONE_BLOCK)
                        {
                            if (aiui_pcm_player_get_state() != PCM_PLAYER_STATE_STARTED)
                            {
                                aiui_pcm_player_start();
                            }
                        }

                        aiui_pcm_player_write(0, buffer, dataLen, dts, progress);

#if 0
                            if (dts == AIUIConstant::DTS_BLOCK_FIRST || dts == AIUIConstant::DTS_ONE_BLOCK) {
                                mFs.open("tts.pcm", ios::binary | ios::out);
                            }

                            mFs.write(buffer, dataLen);

                            if (dts == AIUIConstant::DTS_BLOCK_LAST || dts == AIUIConstant::DTS_ONE_BLOCK) {
                                mFs.close();
                            }
#endif
                    }
                }
            }
            else if (sub == "nlp")
            {
                string resultStr = string(buffer, dataLen);

                long eosRsltTime = event.getData()->getLong("eos_rslt", -1);

                Json::Value resultJson;
                if (reader.parse(resultStr, resultJson, false))
                {
                    if (resultJson.isMember("intent") &&
                        resultJson["intent"].isMember("rc"))
                    {
                        Json::Value intentJson = resultJson["intent"];
                        processIntentJson(params, intentJson, resultStr, eosRsltTime, sid);
                    }
                    else if (resultJson.isMember("nlp"))
                    {
                        Json::Value nlpJson = resultJson["nlp"];
                        string text = nlpJson["text"].asString();

                        if (text.find("{\"intent\":") == 0)
                        {
                            Json::Value textJson;
                            if (reader.parse(text, textJson, false))
                            {
                                Json::Value intentJson = textJson["intent"];
                                processIntentJson(params, intentJson, resultStr, eosRsltTime, sid);
                            }
                        }
                        else
                        {
                            int seq = nlpJson["seq"].asInt();
                            int status = nlpJson["status"].asInt();

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
                            m_pTtsHelper->addText(text, seq, status);
#endif

                            if (status == 2)
                            {
                                mStreamNlpAnswerBuffer.clear();
                            }
                        }
                    }
                    else
                    {
                    }
                }
            }
            else if (sub == "cbm_intent_split")
            {
                string intentStr = string(buffer, dataLen);
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
                string resultStr = string(buffer, dataLen);

                cout << sub << ": " << event.getInfo() << endl
                     << resultStr << endl;
            }
        }
        break;

        case AIUIConstant::EVENT_CMD_RETURN:
        {
            if (AIUIConstant::CMD_BUILD_GRAMMAR == event.getArg1())
            {
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
                int dtype = event.getData()->getInt("sync_dtype", -1);
                int retCode = event.getArg2();
                string dataTypeStr;
                if (dtype == AIUIConstant::VOICE_CLONE_REG)
                {
                    dataTypeStr = "注册音频资源";
                }
                else if (dtype == AIUIConstant::VOICE_CLONE_DEL)
                {
                    dataTypeStr = "删除资源";
                }

                if (AIUIConstant::SUCCESS == retCode)
                {
                    string sid = event.getData()->getString("sid", "");
                    string tag = event.getData()->getString("tag", "");
                    long timeSpent = event.getData()->getLong("time_spent", -1);
                    cout << "声音复刻" << dataTypeStr << "成功"
                         << "，耗时：" << timeSpent
                         << "ms, sid=" + sid + "，tag=" + tag;
                    if (dtype == AIUIConstant::VOICE_CLONE_REG)
                    {
                        string resId = event.getData()->getString("res_id", "");
                        cout << "，res id = " << resId << endl;

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
                    gSyncSid = event.getData()->getString("sid", "");
                    string tag = event.getData()->getString("tag", "");
                    long timeSpent = event.getData()->getLong("time_spent", -1);
                    cout << "同步" << dataTypeStr << "成功"
                         << "，耗时：" << timeSpent
                         << "ms, sid=" + gSyncSid + "，tag=" + tag;
                    if (dtype == AIUIConstant::SYNC_DATA_UPLOAD)
                    {
                        cout << "，你可以试着说“打电话给刘德华”" << endl;
                    }
                    else
                    {
                        cout << endl;
                    }
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
                        gSyncSid = event.getData()->getString("sid", "");
                        string tag = event.getData()->getString("tag", "");
                        long timeSpent = event.getData()->getLong("time_spent", -1);

                        cout << "同步成功，"
                             << "耗时：" << timeSpent
                             << "ms, sid=" + gSyncSid + "，tag=" + tag +
                                    "，你可以试着说“打电话给刘德华”"
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
                int syncType = event.getData()->getInt("sync_dtype", -1);
                if (AIUIConstant::SYNC_DATA_QUERY == syncType)
                {
                    string result = event.getData()->getString("result", "");

                    cout << "查询结果：" << result << endl;
                }
            }
#endif
        }
        break;

        case AIUIConstant::EVENT_START_RECORD:
        {
            cout << "EVENT_START_RECORD " << endl;
        }
        break;

        case AIUIConstant::EVENT_STOP_RECORD:
        {
            cout << "EVENT_STOP_RECORD " << endl;
        }
        break;

        case AIUIConstant::EVENT_ERROR:
        {
            cout << "EVENT_ERROR: error=" << event.getArg1() << ", des=" << event.getInfo() << endl;
        }
        break;

        case AIUIConstant::EVENT_CONNECTED_TO_SERVER:
        {
            string uid = event.getData()->getString("uid", "");
            cout << "EVENT_CONNECTED_TO_SERVER, uid=" << uid << endl;
        }
        break;

        case AIUIConstant::EVENT_SERVER_DISCONNECTED:
        {
            cout << "EVENT_SERVER_DISCONNECTED " << endl;
        }
        break;
        }
    }

public:
    // 提供静态访问（兼容原有调用方式）
    static DemoListener &getInstance()
    {
        auto ptr = g_node_weak.lock();
        if (!ptr)
        {
            // 如果还没创建，创建一个（不应发生在正常流程中）
            return *create();
        }
        return *ptr;
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

void destroyAgent()
{
    if (g_pAgent)
    {
        g_pAgent->destroy();
        g_pAgent = nullptr;
    }
}

void wakeup()
{
    SEND_AIUIMESSAGE4(AIUIConstant::CMD_WAKEUP, 0, 0, "clear_data=true");
}

void resetWakeup()
{
    SEND_AIUIMESSAGE1(AIUIConstant::CMD_RESET_WAKEUP);
}

void start()
{
    SEND_AIUIMESSAGE1(AIUIConstant::CMD_START);
}

void stop()
{
    SEND_AIUIMESSAGE1(AIUIConstant::CMD_STOP);
}

void resetAIUI()
{
    SEND_AIUIMESSAGE(AIUIConstant::CMD_RESET, 0, 0, "", nullptr);
}

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

            AIUIBuffer frameData = aiui_create_buffer_from_data(buff, frameLen);
            SEND_AIUIMESSAGE(AIUIConstant::CMD_WRITE, 0, 0, "data_type=audio,tag=audio-tag", frameData);

            AIUI_SLEEP(40);
        }

        SEND_AIUIMESSAGE4(AIUIConstant::CMD_STOP_WRITE, 0, 0, "data_type=audio");

        delete[] audio;
    }
    else
    {
        cout << "open file failed, path=" << TEST_AUDIO_PATH << endl;
    }

    cout << "write finish" << endl;
}

void startRecordAudio()
{
    SEND_AIUIMESSAGE4(
        AIUIConstant::CMD_START_RECORD, 0, 0, "data_type=audio,pers_param={\"uid\":\"\"},tag=record-tag");
}

void stopRecordAudio()
{
    SEND_AIUIMESSAGE1(AIUIConstant::CMD_STOP_RECORD);
}

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

void triggerCaptureServiceAgent()
{
    DemoListener::getInstance().callTriggerCaptureService();
}

bool locateObjectSyncAgent(const std::string &target, std::string &locateMsg)
{
    DemoListener::getInstance().publichTargetObject(target);
    return DemoListener::getInstance().callLocateObjectSync(locateMsg);
}

std::string getLatestImageDescription()
{
    return DemoListener::getInstance().waitForImageDescription();
}

// ===== 发布检测目标 =====
void publishTargetObject(const std::string &target)
{
    DemoListener::getInstance().publichTargetObject(target);
}

// ===== 获取vision_3DLocation的最近一次坐标（供 agent 模式直接调用）=====
void getLast3DLocation(double &x, double &y, double &z)
{
    x = DemoListener::getInstance().last_3d_x_;
    y = DemoListener::getInstance().last_3d_y_;
    z = DemoListener::getInstance().last_3d_z_;
}

void startTTS(const string &text, const string &tag)
{
    if (!g_pAgent)
    {
        std::cerr << "[startTTS] ERROR: g_pAgent is NULL, TTS cannot proceed!" << std::endl;
        return;
    }

    AIUIBuffer textData = aiui_create_buffer_from_data(text.c_str(), text.length());
    string params = "voice_name=x5_lingxiaoyue_flow";
    if (!tag.empty())
    {
        params.append(",tag=").append(tag);
    }

    std::cout << "[startTTS] sending TTS: text=" << text << " params=" << params << std::endl;
    SEND_AIUIMESSAGE(AIUIConstant::CMD_TTS, AIUIConstant::START, 0, params.c_str(), textData);
}

void startHTS(const string &text, const string &tag)
{
    AIUIBuffer textData = aiui_create_buffer_from_data(text.c_str(), text.length());

    string params = "voice_name=x4_lingxiaoxuan_oral,scene=IFLYTEK.hts";
    if (!tag.empty())
    {
        params.append(",tag=").append(tag);
    }

    SEND_AIUIMESSAGE(AIUIConstant::CMD_TTS, AIUIConstant::START, 0, params.c_str(), textData);
}

void buildAsrGrammar()
{
    string grammar = readFileAsString("AIUI/asr/call.bnf");
    SEND_AIUIMESSAGE4(AIUIConstant::CMD_BUILD_GRAMMAR, 0, 0, grammar.c_str());
}

void buildEsrGrammar()
{
    string grammar = readFileAsString("AIUI/esr/message.fsa");
    SEND_AIUIMESSAGE4(AIUIConstant::CMD_BUILD_GRAMMAR, 0, 0, grammar.c_str());
}

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

void cleanDialogHistory()
{
    cout << "cleanDialogHistory" << endl;
    SEND_AIUIMESSAGE1(AIUIConstant::CMD_CLEAN_DIALOG_HISTORY);
}

void syncSchemaData(int type = AIUIConstant::SYNC_DATA_SCHEMA)
{
    string dataStrBase64 = Base64Util::encode(SYNC_CONTACT_CONTENT);

    Json::Value syncSchemaJson;
    Json::Value dataParamJson;

    dataParamJson["id_name"] = "uid";
    dataParamJson["res_name"] = "IFLYTEK.telephone_contact";

#ifdef AIUI_V2
    dataParamJson["name_space"] = "OS13360977719";
#endif

    syncSchemaJson["param"] = dataParamJson;
    if (AIUIConstant::SYNC_DATA_SCHEMA == type || AIUIConstant::SYNC_DATA_UPLOAD == type)
    {
        syncSchemaJson["data"] = dataStrBase64;
    }

    string jsonStr = syncSchemaJson.toString();

    AIUIBuffer syncData = aiui_create_buffer_from_data(jsonStr.c_str(), jsonStr.length());

    Json::Value paramJson;
    paramJson["tag"] = "sync-tag";

    SEND_AIUIMESSAGE(AIUIConstant::CMD_SYNC, type, 0,
                     paramJson.toString().c_str(), syncData);
}

#ifdef AIUI_V2
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

    cout << "see say content: " << contentJson.asString() << endl;
    string dataStrBase64 = Base64Util::encode(contentJson.asString());
    Json::Value syncSeeSayJson;
    syncSeeSayJson["data"] = dataStrBase64;
    string jsonStr = syncSeeSayJson.toString();
    AIUIBuffer syncData = aiui_create_buffer_from_data(jsonStr.c_str(), jsonStr.length());

    Json::Value paramJson;
    paramJson["tag"] = "sync_see_say_tag";

    SEND_AIUIMESSAGE(AIUIConstant::CMD_SYNC, AIUIConstant::SYNC_DATA_SEE_SAY, 0,
                     paramJson.toString().c_str(), syncData);
}

void voiceCloneReg(string resPath = VOICE_CLONE_AUDIO_PATH)
{
    Json::Value paramJson;
    paramJson["tag"] = "voice_clone_tag_0";
    paramJson["res_path"] = resPath;

    cout << "[func:" << __FUNCTION__ << " line:" << __LINE__ << "] "
         << "上传声音复刻的资源,资源路径: " << resPath << endl;

    SEND_AIUIMESSAGE(AIUIConstant::CMD_CLONE_VOICE, AIUIConstant::VOICE_CLONE_REG, 0,
                     paramJson.toString().c_str(),
                     nullptr);
}

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

    SEND_AIUIMESSAGE(AIUIConstant::CMD_CLONE_VOICE, AIUIConstant::VOICE_CLONE_DEL, 0,
                     paramJson.toString().c_str(),
                     nullptr);
}

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
    string params = "voice_name=x5_clone";
    params.append(",res_id=").append(resId);
    if (!tag.empty())
    {
        params.append(",tag=").append(tag);
    }

    SEND_AIUIMESSAGE(AIUIConstant::CMD_TTS, AIUIConstant::START, 0, params.c_str(), textData);
}

#endif

void querySyncSchemaStatus()
{
    Json::Value queryJson;
    queryJson["sid"] = gSyncSid;

    SEND_AIUIMESSAGE4(AIUIConstant::CMD_QUERY_SYNC_STATUS,
                      AIUIConstant::SYNC_DATA_SCHEMA,
                      0,
                      queryJson.toString().c_str());
}

void syncSpeakableData() {}

#if defined(__linux) || defined(__ANDROID__)
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>

static void GenerateMACAddress(char *mac)
{
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
            {
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
        adapterInfo = adapterInfo->Next;
    }

    free(adapterInfoTmp);

    adapterInfoTmp = NULL;
}
#endif

static void initSetting(bool log = true)
{
    AIUISetting::setAIUIDir(TEST_ROOT_DIR);
    AIUISetting::setMscDir(MSC_DIR);
    AIUISetting::setNetLogLevel(log ? aiui_debug : aiui_none);

    char mac[64] = {0};
    GenerateMACAddress(mac);

    AIUISetting::setSystemInfo(AIUI_KEY_SERIAL_NUM, mac);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = DemoListener::create();
    auto navGoalClient = node->create_client<robot_aiui::srv::NavigateToOffice>("/kybot_navgoali");
#ifdef WIN32
    system("chcp 65001 >nul");
#else
    freopen("/dev/null", "w", stderr);
#endif
    image_pub = node->create_publisher<std_msgs::msg::String>("image_file_path", 10);

    // 打印SDK版本
    std::cout << "Version: " << getVersion() << std::endl;

    initSetting();

    createAgent();

    startRecordAudio();
    cout << "----等待唤醒中----" << endl;
    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    return 0;
}
