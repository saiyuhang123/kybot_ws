#include "my_rviz_panel/mission_executor.hpp"
#include <chrono>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace my_rviz_panel {

MissionExecutor::MissionExecutor(const std::string& name)
    : Node(name)
{
    setupServices();
    setupClients();
    RCLCPP_INFO(get_logger(), "MissionExecutor ready");
}

MissionExecutor::~MissionExecutor()
{
    mission_cancel_ = true;
    if (mission_thread_.joinable())
        mission_thread_.join();
}

void MissionExecutor::registerAction(const std::string& name, ActionHandler handler)
{
    action_handlers_[name] = std::move(handler);
    RCLCPP_INFO(get_logger(), "Registered action handler: '%s'", name.c_str());
}

// ============================================================
// ROS 接口初始化
// ============================================================

void MissionExecutor::setupServices()
{
    run_mission_srv_ = create_service<hk_camera::srv::RunMission>(
        "/mission/run",
        std::bind(&MissionExecutor::onRunMission, this, _1, _2));

    cancel_mission_srv_ = create_service<std_srvs::srv::Trigger>(
        "/mission/cancel",
        std::bind(&MissionExecutor::onCancelMission, this, _1, _2));

    status_pub_ = create_publisher<hk_camera::msg::MissionStatus>(
        "/mission/status", 10);
}

void MissionExecutor::setupClients()
{
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
        this, "navigate_to_pose");

    ptz_client_ = create_client<hk_camera::srv::SetPTZPose>(
        "/hk_camera/set_ptz_pose");

    capture_client_ = create_client<hk_camera::srv::CapturePicture>(
        "/hk_camera/capture");

    login_client_ = create_client<std_srvs::srv::Trigger>(
        "/hk_camera/login");
}

// ============================================================
// 任务控制
// ============================================================

void MissionExecutor::onRunMission(
    const hk_camera::srv::RunMission::Request::SharedPtr req,
    hk_camera::srv::RunMission::Response::SharedPtr res)
{
    if (mission_running_)
    {
        res->accepted = false;
        res->message = "Mission already running";
        return;
    }

    waypoints_ = req->waypoints;
    mission_cancel_ = false;
    mission_running_ = true;

    // 在独立线程执行任务
    if (mission_thread_.joinable())
        mission_thread_.join();
    mission_thread_ = std::thread(&MissionExecutor::executeMission, this);

    res->accepted = true;
    res->message = "Mission accepted, " +
                   std::to_string(waypoints_.size()) + " waypoints";
    RCLCPP_INFO(get_logger(), "Mission started: %zu waypoints", waypoints_.size());
}

void MissionExecutor::onCancelMission(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
{
    if (!mission_running_)
    {
        res->success = false;
        res->message = "No mission running";
        return;
    }

    mission_cancel_ = true;
    // 取消当前导航
    if (nav_client_)
        nav_client_->async_cancel_all_goals();

    res->success = true;
    res->message = "Mission cancel requested";
    RCLCPP_INFO(get_logger(), "Mission cancel requested");
}

// ============================================================
// 任务执行主循环
// ============================================================

void MissionExecutor::executeMission()
{
    const size_t total = waypoints_.size();

    // ---- 自动登录相机 ----
    RCLCPP_INFO(get_logger(), "Checking camera login...");
    if (login_client_->wait_for_service(3s))
    {
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = login_client_->async_send_request(req);
        if (future.wait_for(10s) == std::future_status::ready)
        {
            auto res = future.get();
            RCLCPP_INFO(get_logger(), "Camera login: %s (success=%d)",
                        res->message.c_str(), res->success);
        }
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Login service not available, PTZ/capture may fail");
    }

    for (size_t i = 0; i < total; i++)
    {
        if (mission_cancel_)
        {
            publishStatus(hk_camera::msg::MissionStatus::STATE_CANCELED,
                          i, "Mission canceled");
            break;
        }

        const auto& wp = waypoints_[i];
        char buf[128];

        // ---- Step 1: 导航 ----
        snprintf(buf, sizeof(buf), "Navigating to point %zu/%zu", i + 1, total);
        publishStatus(hk_camera::msg::MissionStatus::STATE_NAVIGATING, i, buf);
        RCLCPP_INFO(get_logger(), "[%zu/%zu] Navigating...", i + 1, total);

        if (!navigateTo(wp.nav_pose))
        {
            if (mission_cancel_) break;
            snprintf(buf, sizeof(buf), "Navigation failed at point %zu", i + 1);
            publishStatus(hk_camera::msg::MissionStatus::STATE_FAILED, i, buf);
            RCLCPP_ERROR(get_logger(), "%s", buf);
            break;
        }
        RCLCPP_INFO(get_logger(), "[%zu/%zu] Arrived", i + 1, total);

        // ---- Step 2: 设置云台 ----
        RCLCPP_INFO(get_logger(), "[%zu/%zu] PTZ check: pan=%.0f",
                    i + 1, total, wp.pan);
        if (wp.pan > 0)
        {
            snprintf(buf, sizeof(buf), "Setting PTZ at point %zu/%zu", i + 1, total);
            publishStatus(hk_camera::msg::MissionStatus::STATE_PTZ_MOVING, i, buf);
            RCLCPP_INFO(get_logger(), "[%zu/%zu] PTZ → preset %.0f",
                        i + 1, total, wp.pan);

            if (!setPTZPose(wp.pan, wp.tilt, wp.zoom))
            {
                RCLCPP_WARN(get_logger(), "[%zu/%zu] PTZ failed, continuing",
                            i + 1, total);
            }
            else
            {
                RCLCPP_INFO(get_logger(), "[%zu/%zu] PTZ OK", i + 1, total);
            }
            // 等待云台到位
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }

        // ---- Step 3: 拍照 ----
        RCLCPP_INFO(get_logger(), "[%zu/%zu] Capture check: do_capture=%d",
                    i + 1, total, wp.do_capture);
        if (wp.do_capture)
        {
            snprintf(buf, sizeof(buf), "Capturing at point %zu/%zu", i + 1, total);
            publishStatus(hk_camera::msg::MissionStatus::STATE_CAPTURING, i, buf);
            RCLCPP_INFO(get_logger(), "[%zu/%zu] Capturing...", i + 1, total);

            if (!capturePicture())
            {
                RCLCPP_WARN(get_logger(), "[%zu/%zu] Capture failed, continuing",
                            i + 1, total);
            }
            else
            {
                RCLCPP_INFO(get_logger(), "[%zu/%zu] Capture OK", i + 1, total);
            }
        }

        // ---- Step 4: 扩展动作 ----
        if (!wp.extra_action.empty())
        {
            auto it = action_handlers_.find(wp.extra_action);
            if (it != action_handlers_.end())
            {
                RCLCPP_INFO(get_logger(), "[%zu/%zu] Executing action: '%s'",
                            i + 1, total, wp.extra_action.c_str());
                it->second(wp, wp.extra_action);
            }
            else
            {
                RCLCPP_WARN(get_logger(), "[%zu/%zu] Unknown action: '%s'",
                            i + 1, total, wp.extra_action.c_str());
            }
        }
    }

    if (!mission_cancel_)
    {
        publishStatus(hk_camera::msg::MissionStatus::STATE_COMPLETED,
                      total, "Mission completed");
        RCLCPP_INFO(get_logger(), "Mission completed: %zu waypoints", total);
    }

    mission_running_ = false;
}

// ============================================================
// 底层调用
// ============================================================

bool MissionExecutor::navigateTo(const geometry_msgs::msg::PoseStamped& pose)
{
    // 等待 action server
    if (!nav_client_->wait_for_action_server(5s))
    {
        RCLCPP_ERROR(get_logger(), "NavigateToPose action server not available");
        return false;
    }

    auto goal = NavigateToPose::Goal();
    goal.pose = pose;

    auto goal_handle_future = nav_client_->async_send_goal(goal);
    if (goal_handle_future.wait_for(10s) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "Failed to send nav goal");
        return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(get_logger(), "Nav goal rejected");
        return false;
    }

    // 等待结果
    auto result_future = nav_client_->async_get_result(goal_handle);
    auto status = result_future.wait_for(600s);  // 最多等 10 分钟
    if (status != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "Nav timeout");
        nav_client_->async_cancel_goal(goal_handle);
        return false;
    }

    auto result = result_future.get();
    return result.code == rclcpp_action::ResultCode::SUCCEEDED;
}

bool MissionExecutor::setPTZPose(float pan, float tilt, float zoom)
{
    RCLCPP_INFO(get_logger(), "setPTZPose: checking service...");
    if (!ptz_client_->wait_for_service(3s))
    {
        RCLCPP_ERROR(get_logger(), "SetPTZPose service NOT available at /hk_camera/set_ptz_pose");
        return false;
    }
    RCLCPP_INFO(get_logger(), "setPTZPose: service available, sending request pan=%.0f", pan);

    auto request = std::make_shared<hk_camera::srv::SetPTZPose::Request>();
    request->pan = pan;
    request->tilt = tilt;
    request->zoom = zoom;

    auto future = ptz_client_->async_send_request(request);
    if (future.wait_for(10s) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "SetPTZPose timeout");
        return false;
    }

    auto response = future.get();
    RCLCPP_INFO(get_logger(), "setPTZPose result: success=%d, msg=%s",
                response->success, response->message.c_str());
    return response->success;
}

bool MissionExecutor::capturePicture()
{
    RCLCPP_INFO(get_logger(), "capturePicture: checking service...");
    if (!capture_client_->wait_for_service(3s))
    {
        RCLCPP_ERROR(get_logger(), "CapturePicture service NOT available at /hk_camera/capture");
        return false;
    }
    RCLCPP_INFO(get_logger(), "capturePicture: service available, sending request");

    auto request = std::make_shared<hk_camera::srv::CapturePicture::Request>();
    request->quality = 0;
    request->save_path = "";

    auto future = capture_client_->async_send_request(request);
    if (future.wait_for(10s) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "CapturePicture timeout");
        return false;
    }

    auto response = future.get();
    RCLCPP_INFO(get_logger(), "capturePicture result: success=%d, msg=%s",
                response->success, response->message.c_str());
    return response->success;
}

void MissionExecutor::publishStatus(uint8_t state, uint8_t index,
                                     const std::string& msg)
{
    auto status = hk_camera::msg::MissionStatus();
    status.state = state;
    status.current_index = index;
    status.total_count = static_cast<uint8_t>(waypoints_.size());
    status.message = msg;
    status_pub_->publish(status);
}

} // namespace my_rviz_panel

// ============================================================
// 独立节点入口
// ============================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<my_rviz_panel::MissionExecutor>();

    // 注册示例扩展动作 (可选)
    node->registerAction("log", [](const hk_camera::msg::MissionWaypoint& wp,
                                    const std::string& name) {
        RCLCPP_INFO(rclcpp::get_logger("mission_executor"),
                    "Custom action '%s' at waypoint", name.c_str());
        return true;
    });

    // ---- 机械臂抓取/放置动作 (yolo_grasp.py 提供的 Trigger 服务) ----
    // extra_action = "grasp": 视觉引导抓取 (YOLO 当前目标)
    // extra_action = "place": 移动到示教放置位姿并张手
    // 注意: client 必须建在 mission_executor 节点上,
    // 由主线程 rclcpp::spin(node) 处理响应, 任务线程只管等 future
    auto make_trigger_action = [node](const std::string& srv_name) {
        auto client = node->create_client<std_srvs::srv::Trigger>(srv_name);
        return [client, srv_name](const hk_camera::msg::MissionWaypoint&,
                                  const std::string&) {
            if (!client->wait_for_service(3s))
            {
                RCLCPP_ERROR(rclcpp::get_logger("mission_executor"),
                             "%s NOT available (yolo_grasp.py running?)",
                             srv_name.c_str());
                return false;
            }
            auto future = client->async_send_request(
                std::make_shared<std_srvs::srv::Trigger::Request>());
            // 抓取/放置含多段 movej/movel，耗时长，超时给足
            if (future.wait_for(120s) != std::future_status::ready)
            {
                RCLCPP_ERROR(rclcpp::get_logger("mission_executor"),
                             "%s timeout", srv_name.c_str());
                return false;
            }
            auto res = future.get();
            RCLCPP_INFO(rclcpp::get_logger("mission_executor"),
                        "%s result: success=%d, msg=%s",
                        srv_name.c_str(), res->success, res->message.c_str());
            return res->success;
        };
    };
    node->registerAction("grasp", make_trigger_action("/yolo_grasp/grasp"));
    node->registerAction("place", make_trigger_action("/yolo_grasp/place"));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
