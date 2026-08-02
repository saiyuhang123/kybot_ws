#include "my_rviz_panel/mission_executor.hpp"
#include <nav2_msgs/action/drive_on_heading.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include <chrono>
#include <limits>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace my_rviz_panel {

MissionExecutor::MissionExecutor(const std::string& name)
    : Node(name)
{
    declare_parameter("front_stop_distance", 0.60);
    declare_parameter("scan_timeout", 0.5);
    declare_parameter("odom_timeout", 0.30);
    declare_parameter("front_scan_min_angle", -10.0);
    declare_parameter("front_scan_max_angle", 10.0);

    setupServices();
    setupClients();

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&MissionExecutor::onOdom, this, _1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan_fe", 10, std::bind(&MissionExecutor::onScan, this, _1));

    RCLCPP_INFO(get_logger(), "MissionExecutor ready");
}

MissionExecutor::~MissionExecutor()
{
    mission_cancel_ = true;
    stopBase();
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

    arm_home2_client_ = create_client<std_srvs::srv::Trigger>(
        "/yolo_grasp/home2");
}

void MissionExecutor::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    latest_odom_ = *msg;
    latest_odom_time_ = now();
    have_odom_ = true;
}

void MissionExecutor::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    latest_scan_ = *msg;
    latest_scan_time_ = now();
    have_scan_ = true;
}

void MissionExecutor::publishVelocity(double linear_x)
{
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
}

void MissionExecutor::stopBase()
{
    // Publish several zero commands so the bridge receives the stop even if
    // one cycle is delayed. The bridge will convert this to neutral/disabled.
    for (int i = 0; i < 3; ++i)
    {
        publishVelocity(0.0);
        rclcpp::sleep_for(20ms);
    }
}

bool MissionExecutor::getFrontObstacleDistance(double& distance) const
{
    const double min_angle = get_parameter("front_scan_min_angle").as_double() * M_PI / 180.0;
    const double max_angle = get_parameter("front_scan_max_angle").as_double() * M_PI / 180.0;

    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (!have_scan_ || latest_scan_.ranges.empty())
        return false;

    double minimum = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < latest_scan_.ranges.size(); ++i)
    {
        const double angle = latest_scan_.angle_min +
                             static_cast<double>(i) * latest_scan_.angle_increment;
        if (angle < min_angle || angle > max_angle)
            continue;

        const double range = latest_scan_.ranges[i];
        if (std::isfinite(range) && range >= latest_scan_.range_min &&
            range <= latest_scan_.range_max)
        {
            minimum = std::min(minimum, range);
        }
    }

    if (!std::isfinite(minimum))
        return false;

    distance = minimum;
    return true;
}

bool MissionExecutor::driveDistance(double distance, double speed,
                                    bool forward, double& traveled_distance)
{
    traveled_distance = 0.0;
    if (distance <= 0.0 || speed <= 0.0)
    {
        stopBase();
        return distance <= 0.0;
    }

    const auto wait_start = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            if (have_odom_)
                break;
        }
        if (mission_cancel_ ||
            std::chrono::steady_clock::now() - wait_start > 2s)
        {
            stopBase();
            RCLCPP_ERROR(get_logger(), "No fresh odometry for base motion");
            return false;
        }
        rclcpp::sleep_for(20ms);
    }

    if (!rclcpp::ok())
    {
        stopBase();
        return false;
    }

    double start_x;
    double start_y;
    double start_yaw;
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        start_x = latest_odom_.pose.pose.position.x;
        start_y = latest_odom_.pose.pose.position.y;
        const auto& q = latest_odom_.pose.pose.orientation;
        start_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    const double direction = forward ? 1.0 : -1.0;
    const double command = direction * speed;
    const double front_stop_distance =
        get_parameter("front_stop_distance").as_double();
    const double scan_timeout = get_parameter("scan_timeout").as_double();
    const double odom_timeout = get_parameter("odom_timeout").as_double();
    const auto motion_start = std::chrono::steady_clock::now();
    const auto max_motion_time =
        std::chrono::duration<double>(distance / speed + 5.0);

    RCLCPP_INFO(get_logger(), "%s %.2f m at %.2f m/s",
                forward ? "Approaching" : "Retreating", distance, speed);

    while (rclcpp::ok())
    {
        if (mission_cancel_)
        {
            stopBase();
            return false;
        }

        double x;
        double y;
        rclcpp::Time odom_time;
        {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            if (!have_odom_)
            {
                stopBase();
                return false;
            }
            x = latest_odom_.pose.pose.position.x;
            y = latest_odom_.pose.pose.position.y;
            odom_time = latest_odom_time_;
        }

        const double age = (now() - odom_time).seconds();
        if (age > odom_timeout)
        {
            stopBase();
            RCLCPP_ERROR(get_logger(), "Odometry timeout during base motion");
            return false;
        }

        const double projection = (x - start_x) * std::cos(start_yaw) +
                                  (y - start_y) * std::sin(start_yaw);
        traveled_distance = std::max(0.0, direction * projection);
        if (traveled_distance >= distance)
        {
            stopBase();
            RCLCPP_WARN(get_logger(),
                        "Approach completed full distance (%.2fm) without "
                        "obstacle stop; front laser may not be seeing the target",
                        traveled_distance);
            return true;
        }

        if (forward)
        {
            rclcpp::Time scan_time;
            bool have_scan = false;
            {
                std::lock_guard<std::mutex> lock(sensor_mutex_);
                scan_time = latest_scan_time_;
                have_scan = have_scan_;
            }
            if (!have_scan || (now() - scan_time).seconds() > scan_timeout)
            {
                stopBase();
                RCLCPP_ERROR(get_logger(), "Laser scan timeout during approach");
                return false;
            }

            double obstacle_distance = 0.0;
            if (!getFrontObstacleDistance(obstacle_distance))
            {
                stopBase();
                RCLCPP_ERROR(get_logger(), "No valid forward laser range");
                return false;
            }
            if (obstacle_distance <= front_stop_distance)
            {
                stopBase();
                RCLCPP_WARN(get_logger(),
                            "Approach stopped by obstacle at %.2f m; traveled %.2f m",
                            obstacle_distance, traveled_distance);
                // A controlled obstacle stop is a valid approach endpoint.
                return true;
            }
        }

        if (std::chrono::steady_clock::now() - motion_start > max_motion_time)
        {
            stopBase();
            RCLCPP_ERROR(get_logger(), "Base motion timeout");
            return false;
        }

        publishVelocity(command);
        rclcpp::sleep_for(50ms);
    }

    stopBase();
    return false;
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

    // ---- 任务开始: 机械臂收拢到 Home2 (导航期间安全姿态, 无机械臂时跳过) ----
    if (arm_home2_client_->wait_for_service(2s))
    {
        RCLCPP_INFO(get_logger(), "Stowing arm to Home2 before navigation...");
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = arm_home2_client_->async_send_request(req);
        // home2 服务同步执行, 返回时已到位 (movej 约几秒, 超时给 30s)
        if (future.wait_for(30s) == std::future_status::ready)
        {
            auto res = future.get();
            RCLCPP_INFO(get_logger(), "Arm Home2: %s (success=%d)",
                        res->message.c_str(), res->success);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Arm Home2 timeout, continuing anyway");
        }
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Arm home2 service not available, skipping stow");
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
// 直线行驶原语 (Nav2 behavior: drive_on_heading / backup)
// 从任务线程同步调用, 带 Nav2 局部代价地图碰撞检查
// ============================================================
template<typename ActionT>
bool driveStraight(
    typename rclcpp_action::Client<ActionT>::SharedPtr client,
    double distance, double speed, const char* name,
    double* traveled_out = nullptr)
{
    if (!client->wait_for_action_server(3s))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_executor"),
                     "%s action server NOT available (behavior_server running?)",
                     name);
        return false;
    }

    typename ActionT::Goal goal;
    goal.target.x = distance;  // BackUp 内部自动取负, 传正值即可
    goal.speed = speed;
    goal.time_allowance = rclcpp::Duration(30, 0);

    // 用 feedback 记录实际行驶距离 (碰撞检查提前停止时退回不会过冲)
    auto opts = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    if (traveled_out)
    {
        *traveled_out = 0.0;
        opts.feedback_callback =
            [traveled_out](
                typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
                const std::shared_ptr<const typename ActionT::Feedback> fb) {
                *traveled_out = fb->distance_traveled;
            };
    }

    auto gh_future = client->async_send_goal(goal, opts);
    if (gh_future.wait_for(5s) != std::future_status::ready)
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_executor"),
                     "%s: send goal timeout", name);
        return false;
    }
    auto goal_handle = gh_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_executor"),
                     "%s: goal rejected", name);
        return false;
    }

    auto result_future = client->async_get_result(goal_handle);
    if (result_future.wait_for(60s) != std::future_status::ready)
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_executor"),
                     "%s: result timeout", name);
        client->async_cancel_goal(goal_handle);
        return false;
    }

    auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_WARN(rclcpp::get_logger("mission_executor"),
                    "%s: finished with code %d (可能因碰撞检查提前停止)",
                    name, static_cast<int>(result.code));
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("mission_executor"),
                "%s: done (%.2fm @ %.2fm/s)", name, distance, speed);
    return true;
}

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

    // ---- 机械臂放置/位姿动作 (yolo_grasp.py 提供的 Trigger 服务) ----
    // extra_action = "place": 移动到示教放置位姿并张手
    // extra_action = "home2"/"ready": 机械臂收拢/预备位姿 (调试用)
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
    // ---- 抓取组合动作: 直线逼近 → 视觉抓取 → 直线退回 ----
    // YAML 路点写 standoff 位姿 (远离障碍的自由空间, yaw 对准目标),
    // 到位后由 behavior_server 做带碰撞检查的直线逼近, 抓取后原路退回
    node->declare_parameter("approach_distance", 1.5);  // 逼近距离 (m)
    node->declare_parameter("approach_speed", 0.1);     // 逼近速度 (m/s)

    auto grasp_trigger =
        node->create_client<std_srvs::srv::Trigger>("/yolo_grasp/grasp");

    node->registerAction("grasp", [=](const hk_camera::msg::MissionWaypoint&,
                                      const std::string&) {
        const double dist = node->get_parameter("approach_distance").as_double();
        const double speed = node->get_parameter("approach_speed").as_double();

        // 1. 直线逼近 (碰撞检查提前停止只警告, 是否够得着由抓取服务判断)
        double approached = 0.0;
        const bool approach_ok =
            node->driveDistance(dist, speed, true, approached);
        if (!approach_ok)
        {
            RCLCPP_WARN(rclcpp::get_logger("mission_executor"),
                        "Approach failed (%.2f/%.2fm), skipping grasp",
                        approached, dist);
        }

        // 2. 视觉抓取
        bool ok = false;
        if (approach_ok && !grasp_trigger->wait_for_service(3s))
        {
            RCLCPP_ERROR(rclcpp::get_logger("mission_executor"),
                         "/yolo_grasp/grasp NOT available (yolo_grasp.py running?)");
        }
        else if (approach_ok)
        {
            auto future = grasp_trigger->async_send_request(
                std::make_shared<std_srvs::srv::Trigger::Request>());
            if (future.wait_for(120s) != std::future_status::ready)
            {
                RCLCPP_ERROR(rclcpp::get_logger("mission_executor"),
                             "/yolo_grasp/grasp timeout");
            }
            else
            {
                auto res = future.get();
                ok = res->success;
                RCLCPP_INFO(rclcpp::get_logger("mission_executor"),
                            "grasp result: success=%d, msg=%s",
                            res->success, res->message.c_str());
            }
        }

        // 3. 原路退回 (无论抓取成败; 只退实际逼近的距离, 防过冲)
        if (approached > 0.05)
        {
            double retreated = 0.0;
            if (!node->driveDistance(approached, speed, false, retreated))
            {
                RCLCPP_ERROR(rclcpp::get_logger("mission_executor"),
                             "Retreat FAILED, 底盘仍停在逼近位置, 请人工确认!");
            }
        }
        return ok;
    });

    node->registerAction("place", make_trigger_action("/yolo_grasp/place"));
    node->registerAction("home2", make_trigger_action("/yolo_grasp/home2"));
    node->registerAction("ready", make_trigger_action("/yolo_grasp/ready"));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
