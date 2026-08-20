#include "my_rviz_panel/mission_executor.hpp"
#include <nav2_msgs/action/drive_on_heading.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include <chrono>
#include <limits>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace my_rviz_panel {

namespace {

bool actionAllowedForMode(const std::string& mode, const std::string& action)
{
    if (action.empty() || action == "log")
        return true;
    if (mode == "polish")
        return action == "polish";
    if (mode == "twofinger" || mode == "softtouch" || mode == "linkerhand")
        return action == "grasp" || action == "place" ||
               action == "home2" || action == "ready";
    return false;
}

bool isValidEndEffectorMode(const std::string& mode)
{
    return mode == "twofinger" || mode == "softtouch" ||
           mode == "linkerhand" || mode == "polish";
}

}  // namespace

MissionExecutor::MissionExecutor(const std::string& name)
    : Node(name)
{
    declare_parameter("front_stop_distance", 0.70);
    // 柔触专用前向停车保护距离 (m)：仅 end_effector_mode=softtouch 时生效，
    // 其他末端仍用 front_stop_distance
    declare_parameter("softtouch_front_stop_distance", 0.50);
    declare_parameter("scan_timeout", 0.5);
    declare_parameter("odom_timeout", 0.30);
    declare_parameter("front_scan_min_angle", -10.0);
    declare_parameter("front_scan_max_angle", 10.0);
    declare_parameter("bottle_stop_distance", 0.60);
    declare_parameter("bottle_approach_speed", 0.15);
    declare_parameter("bottle_confirm_timeout", 5.0);
    declare_parameter("bottle_interrupt_distance", 2.0);
    declare_parameter("max_bottle_interrupts_per_waypoint", 10);
    declare_parameter("bottle_candidate_frames", 3);
    declare_parameter("use_map_goal_approach", true);
    declare_parameter("approach_goal_timeout", 3.0);
    end_effector_mode_ = declare_parameter<std::string>(
        "end_effector_mode", "twofinger");
    declare_parameter("polish_timeout_sec", 1020.0);
    if (!isValidEndEffectorMode(end_effector_mode_))
    {
        RCLCPP_ERROR(get_logger(),
                     "Invalid end_effector_mode='%s'; all end-effector actions will be rejected",
                     end_effector_mode_.c_str());
    }

    // 必须在所有 declare_parameter 之后注册: declare 自身也会触发本回调。
    // 注册后即可在不重启本节点 (即不打断定位/Nav2) 的前提下热切换末端:
    //   ros2 param set /mission_executor end_effector_mode softtouch
    mode_param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
            return onSetParameters(params);
        });

    setupServices();
    setupClients();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&MissionExecutor::onOdom, this, _1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan_fe", 10, std::bind(&MissionExecutor::onScan, this, _1));
    trash_sub_ =
        create_subscription<trash_mission_interfaces::msg::TrashTarget>(
            "/trash/target", 10,
            std::bind(&MissionExecutor::onTrashTarget, this, _1));
    approach_goal_sub_ =
        create_subscription<geometry_msgs::msg::PoseStamped>(
            "/trash/approach_goal", 10,
            std::bind(&MissionExecutor::onApproachGoal, this, _1));

    RCLCPP_INFO(get_logger(), "MissionExecutor ready (M3), end effector=%s",
                end_effector_mode_.c_str());
}

MissionExecutor::~MissionExecutor()
{
    mission_cancel_ = true;
    stopBase();
    if (mission_thread_.joinable())
        mission_thread_.join();
}

std::string MissionExecutor::endEffectorMode() const
{
    std::lock_guard<std::mutex> lock(mode_mutex_);
    return end_effector_mode_;
}

// 运行时热切换末端。定位/Nav2/EKF/FAST_LIO 全程不受影响, 只有本节点的
// 动作白名单、前向停车距离和收臂服务选择会跟着变。
rcl_interfaces::msg::SetParametersResult MissionExecutor::onSetParameters(
    const std::vector<rclcpp::Parameter>& params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& p : params)
    {
        if (p.get_name() != "end_effector_mode")
            continue;

        if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
        {
            result.successful = false;
            result.reason = "end_effector_mode must be a string";
            return result;
        }

        const std::string next = p.as_string();
        if (!isValidEndEffectorMode(next))
        {
            result.successful = false;
            result.reason = "invalid end_effector_mode '" + next +
                "'; expected twofinger/softtouch/linkerhand/polish";
            return result;
        }

        // 任务执行中一律拒绝: 动作白名单、停车距离、收臂服务都会中途改变
        // 语义, 半程换末端等于让机器人用错误的物理假设继续跑。
        if (mission_running_.load())
        {
            result.successful = false;
            result.reason =
                "mission is running; cancel the mission before switching "
                "end effector";
            return result;
        }

        std::string prev;
        {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            prev = end_effector_mode_;
            end_effector_mode_ = next;
        }
        if (prev != next)
        {
            RCLCPP_WARN(get_logger(),
                        "End effector mode hot-switched: %s -> %s",
                        prev.c_str(), next.c_str());
        }
    }
    return result;
}

void MissionExecutor::registerAction(const std::string& name, ActionHandler handler)
{
    action_handlers_[name] = std::move(handler);
    RCLCPP_INFO(get_logger(), "Registered action handler: '%s'", name.c_str());
}

void MissionExecutor::setActionDetail(const std::string& detail)
{
    action_detail_ = detail;
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

    grasp_client_ = create_client<std_srvs::srv::Trigger>(
        "/yolo_grasp/grasp_hold");
    place_client_ = create_client<std_srvs::srv::Trigger>(
        "/yolo_grasp/place");
    polish_run_client_ = create_client<std_srvs::srv::Trigger>(
        "/elite_polish/run");
    polish_cancel_client_ = create_client<std_srvs::srv::Trigger>(
        "/elite_polish/cancel");
    polish_home_client_ = create_client<std_srvs::srv::Trigger>(
        "/elite_polish/home");
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

void MissionExecutor::onTrashTarget(
    const trash_mission_interfaces::msg::TrashTarget::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(trash_mutex_);
        latest_trash_ = *msg;
        have_trash_ = true;
        trash_stamp_ = now();
        // 缓存最近一次有效距离（停车静态确认 + 开环逼近用）
        if (msg->distance_valid && msg->distance > 0.05)
        {
            last_good_bottle_dist_ = msg->distance;
            last_good_bottle_time_ = now();
        }
    }

    if (msg->stationary_confirm)
    {
        bottle_confirmed_ = true;
    }

    if (!mission_running_ || mission_cancel_)
    {
        return;
    }

    // 运动中粗检：连续多帧 detected && moving，且 D435 距离进入
    // bottle_interrupt_distance（默认 2m）才中断导航停车。
    // 识别到不等于停车，避免车停得太远够不着。
    const double interrupt_dist =
        get_parameter("bottle_interrupt_distance").as_double();
    const double interrupt_hyst = interrupt_dist + 0.3;
    if (!msg->detected || !msg->moving || !msg->distance_valid)
    {
        bottle_candidate_count_ = 0;
        return;
    }
    if (msg->distance <= interrupt_dist)
    {
        bottle_candidate_count_++;
    }
    else if (msg->distance > interrupt_hyst)
    {
        // 距离还在 2.0~2.3m 区间时保持计数（防抖动），超过 2.3m 才清零
        bottle_candidate_count_ = 0;
    }
    const int frames = get_parameter("bottle_candidate_frames").as_int();
    if (!bottle_interrupt_.load() && bottle_candidate_count_ >= frames)
    {
        bottle_interrupt_ = true;
        RCLCPP_INFO(get_logger(),
                    "Bottle candidate confirmed (%d frames, dist %.2fm), "
                    "interrupt nav",
                    frames, msg->distance);
    }
}

void MissionExecutor::onApproachGoal(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!msg || msg->header.frame_id != "map")
    {
        return;
    }
    std::lock_guard<std::mutex> lock(trash_mutex_);
    latest_approach_goal_ = *msg;
    latest_approach_goal_time_ = now();
    have_approach_goal_ = true;
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
    // 柔触模式用专用停车保护距离，其他末端保持 front_stop_distance 不变
    const double front_stop_distance =
        endEffectorMode() == "softtouch"
            ? get_parameter("softtouch_front_stop_distance").as_double()
            : get_parameter("front_stop_distance").as_double();
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

    if (req->waypoints.empty())
    {
        res->accepted = false;
        res->message = "Mission has no waypoints";
        return;
    }

    // 在底盘移动前完成整条路线的末端动作预检，防止装着打磨头却执行抓取，
    // 或装着夹爪却在到点后才发现 polish 不可用。
    // 整段任务只在这里读一次末端模式, 避免任务中途被热切换改变语义
    const std::string mission_mode = endEffectorMode();
    for (size_t i = 0; i < req->waypoints.size(); ++i)
    {
        const auto& action = req->waypoints[i].extra_action;
        if (!actionAllowedForMode(mission_mode, action))
        {
            res->accepted = false;
            res->message = "Waypoint " + std::to_string(i + 1) +
                " action '" + action + "' is incompatible with end effector '" +
                mission_mode + "'";
            RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
            return;
        }
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
    if (endEffectorMode() == "polish" &&
        polish_cancel_client_ && polish_cancel_client_->service_is_ready())
    {
        polish_cancel_client_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>());
        RCLCPP_WARN(get_logger(), "Requested safe polish cancel");
    }

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
    bool mission_failed = false;

    // ---- 自动登录相机 ----
    RCLCPP_INFO(get_logger(), "Checking camera login...");
    if (login_client_->wait_for_service(5s))
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
    const std::string stow_mode = endEffectorMode();
    const bool stow_required = stow_mode == "polish";
    auto stow_client = stow_required ? polish_home_client_ : arm_home2_client_;
    const auto stow_timeout = stow_required ? 90s : 30s;
    const char* stow_name = stow_required
        ? "/elite_polish/home" : "/yolo_grasp/home2";
    if (stow_client->wait_for_service(5s))
    {
        RCLCPP_INFO(get_logger(), "Stowing arm to Home2 via %s before navigation...",
                    stow_name);
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = stow_client->async_send_request(req);
        // 服务同步执行，返回时才算已到 Home2；打磨系统启动较慢，给足 90s。
        if (future.wait_for(stow_timeout) == std::future_status::ready)
        {
            auto res = future.get();
            RCLCPP_INFO(get_logger(), "Arm Home2: %s (success=%d)",
                        res->message.c_str(), res->success);
            if (stow_required && !res->success && !mission_cancel_)
            {
                publishStatus(hk_camera::msg::MissionStatus::STATE_FAILED, 0,
                              "Polish arm failed to reach Home2; mission not started");
                mission_failed = true;
            }
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Arm Home2 timeout via %s", stow_name);
            if (stow_required && !mission_cancel_)
            {
                publishStatus(hk_camera::msg::MissionStatus::STATE_FAILED, 0,
                              "Polish arm Home2 timeout; mission not started");
                mission_failed = true;
            }
        }
    }
    else
    {
        if (stow_required)
        {
            RCLCPP_ERROR(get_logger(), "%s not available; mission not started", stow_name);
            publishStatus(hk_camera::msg::MissionStatus::STATE_FAILED, 0,
                          "Polish arm service unavailable; mission not started");
            mission_failed = true;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Arm home2 service not available, skipping stow");
        }
    }

    if (mission_failed || mission_cancel_)
    {
        if (mission_cancel_)
        {
            publishStatus(hk_camera::msg::MissionStatus::STATE_CANCELED, 0,
                          "Mission canceled before navigation");
        }
        mission_running_ = false;
        return;
    }

    size_t current_index = 0;
    for (size_t i = 0; i < total; i++)
    {
        current_index = i;
        if (mission_cancel_)
        {
            break;
        }

        const auto& wp = waypoints_[i];
        char buf[128];

        // ---- Step 1: 导航（途中可被瓶子中断后自动捡瓶并恢复）----
        NavResult nav_res = NavResult::FAILED;
        int bottle_interrupts = 0;
        const int max_interrupts =
            get_parameter("max_bottle_interrupts_per_waypoint").as_int();

        while (true)
        {
            if (mission_cancel_) break;

            // 每个导航周期重置瓶子中断状态
            bottle_interrupt_ = false;
            bottle_confirmed_ = false;
            bottle_candidate_count_ = 0;

            snprintf(buf, sizeof(buf), "Navigating to point %zu/%zu",
                     i + 1, total);
            publishStatus(hk_camera::msg::MissionStatus::STATE_NAVIGATING,
                          i, buf);
            RCLCPP_INFO(get_logger(), "[%zu/%zu] Navigating...", i + 1, total);

            nav_res = navigateTo(wp.nav_pose);
            if (nav_res != NavResult::INTERRUPTED)
            {
                break;
            }

            // 导航被瓶子中断：停车确认 → 接近 → 抓取 → 放置 → 退回
            bottle_interrupts++;
            RCLCPP_WARN(get_logger(),
                        "[%zu/%zu] Bottle detected, pickup attempt %d/%d",
                        i + 1, total, bottle_interrupts, max_interrupts);
            const bool picked = pickupBottle(i);
            RCLCPP_INFO(get_logger(), "[%zu/%zu] Pickup finished: %s",
                        i + 1, total, picked ? "ok" : "failed");

            if (bottle_interrupts >= max_interrupts)
            {
                RCLCPP_WARN(get_logger(),
                            "[%zu/%zu] Too many interrupts, skip to next waypoint",
                            i + 1, total);
                break;
            }
            // 未超限：继续重新导航当前路点
        }

        if (mission_cancel_)
        {
            break;
        }

        if (nav_res == NavResult::FAILED)
        {
            snprintf(buf, sizeof(buf), "Navigation failed at point %zu", i + 1);
            publishStatus(hk_camera::msg::MissionStatus::STATE_FAILED, i, buf);
            RCLCPP_ERROR(get_logger(), "%s", buf);
            mission_failed = true;
            break;
        }

        if (nav_res == NavResult::INTERRUPTED)
        {
            // 中断次数超限，跳过当前点继续下一个
            RCLCPP_WARN(get_logger(),
                        "[%zu/%zu] Skip waypoint after pickup attempts",
                        i + 1, total);
            continue;
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
                if (wp.extra_action == "polish")
                {
                    snprintf(buf, sizeof(buf), "Polishing at point %zu/%zu",
                             i + 1, total);
                    publishStatus(hk_camera::msg::MissionStatus::STATE_POLISHING,
                                  i, buf);
                }
                action_detail_.clear();
                const bool action_ok = it->second(wp, wp.extra_action);
                if (!action_ok)
                {
                    if (!mission_cancel_)
                    {
                        snprintf(buf, sizeof(buf),
                                 "Action '%s' failed at point %zu/%zu",
                                 wp.extra_action.c_str(), i + 1, total);
                        std::string failure_message(buf);
                        if (!action_detail_.empty())
                            failure_message += ": " + action_detail_;
                        publishStatus(hk_camera::msg::MissionStatus::STATE_FAILED,
                                      i, failure_message);
                        RCLCPP_ERROR(get_logger(), "%s", failure_message.c_str());
                        mission_failed = true;
                    }
                    break;
                }
            }
            else
            {
                snprintf(buf, sizeof(buf), "Unknown action '%s' at point %zu/%zu",
                         wp.extra_action.c_str(), i + 1, total);
                publishStatus(hk_camera::msg::MissionStatus::STATE_FAILED, i, buf);
                RCLCPP_ERROR(get_logger(), "%s", buf);
                mission_failed = true;
                break;
            }
        }
    }

    if (mission_cancel_)
    {
        // 任务线程退出前保证最后一次状态一定是 CANCELED。
        // 取消可能发生在 NAVIGATING/APPROACHING/RETREATING 等阶段，
        // 若不在收尾统一发布，launcher 会一直停留在最后一个 busy 状态。
        publishStatus(hk_camera::msg::MissionStatus::STATE_CANCELED,
                      static_cast<uint8_t>(current_index), "Mission canceled");
        RCLCPP_INFO(get_logger(), "Mission canceled at waypoint %zu",
                    current_index);
    }
    else if (!mission_failed)
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

MissionExecutor::NavResult MissionExecutor::navigateTo(
    const geometry_msgs::msg::PoseStamped& pose,
    bool allow_bottle_interrupt)
{
    // 等待 action server
    if (!nav_client_->wait_for_action_server(5s))
    {
        RCLCPP_ERROR(get_logger(), "NavigateToPose action server not available");
        return NavResult::FAILED;
    }

    auto goal = NavigateToPose::Goal();
    goal.pose = pose;

    auto goal_handle_future = nav_client_->async_send_goal(goal);
    if (goal_handle_future.wait_for(10s) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "Failed to send nav goal");
        return NavResult::FAILED;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(get_logger(), "Nav goal rejected");
        return NavResult::FAILED;
    }

    // 等待结果，同时监听瓶子中断
    auto result_future = nav_client_->async_get_result(goal_handle);
    while (rclcpp::ok())
    {
        if (mission_cancel_)
        {
            nav_client_->async_cancel_goal(goal_handle);
            return NavResult::FAILED;
        }
        if (allow_bottle_interrupt && bottle_interrupt_.load())
        {
            RCLCPP_INFO(get_logger(),
                        "Bottle candidate detected, interrupting navigation");
            nav_client_->async_cancel_goal(goal_handle);
            return NavResult::INTERRUPTED;
        }
        auto status = result_future.wait_for(100ms);
        if (status == std::future_status::ready)
        {
            break;
        }
    }

    auto result = result_future.get();
    return result.code == rclcpp_action::ResultCode::SUCCEEDED
               ? NavResult::OK
               : NavResult::FAILED;
}

// ============================================================
// M3：自动捡瓶流程
// ============================================================

bool MissionExecutor::callTriggerService(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
    const std::string& name, double timeout_sec)
{
    if (!client->wait_for_service(5s))
    {
        RCLCPP_ERROR(get_logger(), "%s NOT available (yolo_grasp.py running?)",
                     name.c_str());
        return false;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(req);
    const auto timeout =
        std::chrono::duration<double, std::ratio<1>>(timeout_sec);
    if (future.wait_for(timeout) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "%s timeout", name.c_str());
        return false;
    }
    auto res = future.get();
    RCLCPP_INFO(get_logger(), "%s result: success=%d, msg=%s",
                name.c_str(), res->success, res->message.c_str());
    return res->success;
}

bool MissionExecutor::pickupBottle(size_t waypoint_index)
{
    const double confirm_timeout =
        get_parameter("bottle_confirm_timeout").as_double();
    const double approach_speed =
        get_parameter("bottle_approach_speed").as_double();

    // 1. 停车确认
    publishStatus(hk_camera::msg::MissionStatus::STATE_BOTTLE_CONFIRMING,
                  waypoint_index, "Confirming bottle (stopped)");
    const auto confirm_start = std::chrono::steady_clock::now();
    bool confirmed = false;
    while (rclcpp::ok() && !mission_cancel_)
    {
        if (bottle_confirmed_.load())
        {
            confirmed = true;
            break;
        }
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          confirm_start)
                .count();
        if (elapsed > confirm_timeout)
        {
            // 未完成静态确认：若还有新鲜的最近距离，仍按“静态距离→开环逼近”
            // 策略继续，成败交给机械臂拍照位验证（D435 近距离本来就不稳定）
            bool fresh_dist = false;
            {
                std::lock_guard<std::mutex> lock(trash_mutex_);
                fresh_dist = last_good_bottle_dist_ > 0.0 &&
                             (now() - last_good_bottle_time_).seconds() <= 2.0;
            }
            if (fresh_dist)
            {
                RCLCPP_WARN(get_logger(),
                            "Bottle confirm timeout after %.1fs, but fresh "
                            "distance exists, continue approach anyway", elapsed);
                confirmed = true;
                break;
            }
            RCLCPP_WARN(get_logger(),
                        "Bottle confirm timeout after %.1fs, treat as false alarm",
                        elapsed);
            stopBase();
            return false;
        }
        rclcpp::sleep_for(100ms);
    }
    if (mission_cancel_)
    {
        return false;
    }
    if (!confirmed)
    {
        RCLCPP_WARN(get_logger(), "Bottle not confirmed, resume navigation");
        stopBase();
        return false;
    }
    RCLCPP_INFO(get_logger(), "Bottle confirmed, start approach");

    // 2. 接近瓶子：优先使用感知节点算好的 map goal（0.55m + 朝向），
    //    不可用时回退原来的 D435 距离直线逼近。
    publishStatus(hk_camera::msg::MissionStatus::STATE_APPROACHING,
                  waypoint_index, "Approaching bottle");
    double approached = 0.0;
    geometry_msgs::msg::PoseStamped pre_approach_pose;
    bool map_goal_available = false;
    const bool map_goal_ok =
        approachToBottleMapGoal(pre_approach_pose, map_goal_available);
    const bool used_map_goal = map_goal_available;
    bool approach_ok = map_goal_ok;
    if (!map_goal_available)
    {
        RCLCPP_INFO(get_logger(),
                    "Map-goal approach unavailable, fallback to straight D435 approach");
        approach_ok = approachToBottle(approached);
    }
    else if (!map_goal_ok)
    {
        RCLCPP_WARN(get_logger(),
                    "Map-goal approach failed, will not fallback blindly");
    }
    if (!approach_ok)
    {
        RCLCPP_WARN(get_logger(), "Approach failed");
    }

    // 3. 抓取
    bool grasped = false;
    if (approach_ok)
    {
        publishStatus(hk_camera::msg::MissionStatus::STATE_GRASPING,
                      waypoint_index, "Grasping bottle");
        grasped = callTriggerService(
            grasp_client_, "/yolo_grasp/grasp_hold", 120.0);
    }

    // 4. 放置
    if (grasped)
    {
        publishStatus(hk_camera::msg::MissionStatus::STATE_PLACING,
                      waypoint_index, "Placing bottle");
        callTriggerService(place_client_, "/yolo_grasp/place", 120.0);
    }

    // 5. 退回：map goal 方案用 Nav2 回到逼近前位姿；旧方案按行进距离原路倒回。
    publishStatus(hk_camera::msg::MissionStatus::STATE_RETREATING,
                  waypoint_index, "Retreating");
    if (used_map_goal)
    {
        RCLCPP_INFO(get_logger(),
                    "Return to pre-approach pose via Nav2");
        if (navigateTo(pre_approach_pose, false) != NavResult::OK)
        {
            RCLCPP_ERROR(get_logger(),
                         "Retreat via Nav2 FAILED, please check manually");
        }
    }
    else if (approached > 0.05)
    {
        double retreated = 0.0;
        if (!driveDistance(approached, approach_speed, false, retreated))
        {
            RCLCPP_ERROR(get_logger(),
                         "Retreat FAILED, car still at approach position, "
                         "please check manually");
        }
    }
    return grasped;
}

bool MissionExecutor::approachToBottle(double& traveled_distance)
{
    traveled_distance = 0.0;
    const double d_stop =
        get_parameter("bottle_stop_distance").as_double();
    const double approach_speed =
        get_parameter("bottle_approach_speed").as_double();
    const double front_stop =
        endEffectorMode() == "softtouch"
            ? get_parameter("softtouch_front_stop_distance").as_double()
            : get_parameter("front_stop_distance").as_double();
    const double scan_timeout =
        get_parameter("scan_timeout").as_double();
    const double odom_timeout =
        get_parameter("odom_timeout").as_double();

    // 两段式逼近：
    //   远段（距离 > 0.9m）：D435 实时距离可靠，闭环边走边看；
    //   近段（距离 <= 0.9m）：D435 近距离不可靠，改为里程计开环补到 d_stop；
    //   中途丢距离：用“最后有效距离 - 已行驶距离”开环补足（限幅 max_blind）。
    const double far_target = 0.90;
    const double max_blind = 0.80;

    // 等待里程计
    const auto wait_start = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            if (have_odom_)
            {
                break;
            }
        }
        if (mission_cancel_ ||
            std::chrono::steady_clock::now() - wait_start > 2s)
        {
            stopBase();
            RCLCPP_ERROR(get_logger(), "No fresh odometry for bottle approach");
            return false;
        }
        rclcpp::sleep_for(20ms);
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

    // ---------- 远段：闭环逼近 ----------
    while (rclcpp::ok())
    {
        if (mission_cancel_)
        {
            stopBase();
            return false;
        }

        // 里程计新鲜度 + 已行驶距离
        double x, y;
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
        if ((now() - odom_time).seconds() > odom_timeout)
        {
            stopBase();
            RCLCPP_ERROR(get_logger(), "Odometry timeout during bottle approach");
            return false;
        }
        const double projection =
            (x - start_x) * std::cos(start_yaw) +
            (y - start_y) * std::sin(start_yaw);
        traveled_distance = std::max(0.0, projection);
        if (traveled_distance > 1.5)
        {
            stopBase();
            RCLCPP_ERROR(get_logger(),
                         "Approach traveled %.2fm without reaching bottle, abort",
                         traveled_distance);
            return false;
        }

        // 前激光硬保护
        {
            rclcpp::Time scan_time;
            bool have_scan = false;
            {
                std::lock_guard<std::mutex> lock(sensor_mutex_);
                scan_time = latest_scan_time_;
                have_scan = have_scan_;
            }
            if (have_scan &&
                (now() - scan_time).seconds() <= scan_timeout)
            {
                double obstacle = 0.0;
                if (getFrontObstacleDistance(obstacle) &&
                    obstacle <= front_stop)
                {
                    stopBase();
                    RCLCPP_WARN(get_logger(),
                                "Front laser stop at %.2fm, arrived",
                                obstacle);
                    return true;
                }
            }
        }

        // 取最近一次有效距离
        double dist = -1.0;
        rclcpp::Time dist_time;
        {
            std::lock_guard<std::mutex> lock(trash_mutex_);
            if (last_good_bottle_dist_ > 0.0)
            {
                dist = last_good_bottle_dist_;
                dist_time = last_good_bottle_time_;
            }
        }
        const double age = (now() - dist_time).seconds();

        if (dist > 0.0 && age <= 2.0)
        {
            if (dist <= far_target)
            {
                break;  // 已进入近段，转开环
            }
            // 还远：按距离定速前进
            double speed = dist > 1.2 ? 0.30 : 0.15;
            speed = std::min(speed, approach_speed);
            publishVelocity(speed);
            rclcpp::sleep_for(50ms);
            continue;
        }

        // ---------- 距离丢失/过期：用最后距离 - 已行驶开环补足 ----------
        const double need = std::max(
            0.0, dist - traveled_distance - d_stop);
        stopBase();
        if (need <= 0.02)
        {
            RCLCPP_INFO(get_logger(),
                        "Bottle distance lost but already in range");
            return true;
        }
        if (need > max_blind)
        {
            RCLCPP_WARN(get_logger(),
                        "Bottle distance lost (last %.2fm, traveled %.2fm), "
                        "remaining %.2fm too far to drive blind",
                        dist, traveled_distance, need);
            return false;
        }
        RCLCPP_INFO(get_logger(),
                    "Bottle distance lost, open-loop remaining %.2fm",
                    need);
        double driven = 0.0;
        driveDistance(need, 0.08, true, driven);
        traveled_distance += driven;
        return true;
    }

    // ---------- 近段：开环收尾到 d_stop ----------
    double dist = -1.0;
    {
        std::lock_guard<std::mutex> lock(trash_mutex_);
        if (last_good_bottle_dist_ > 0.0)
        {
            dist = last_good_bottle_dist_;
        }
    }
    const double need = std::max(0.0, dist - traveled_distance - d_stop);
    stopBase();
    RCLCPP_INFO(get_logger(),
                "Static dist %.2fm, open-loop %.2fm to d_stop=%.2fm",
                dist, need, d_stop);
    if (need > 0.02)
    {
        double driven = 0.0;
        driveDistance(need, approach_speed, true, driven);
        traveled_distance += driven;
    }
    return true;
}

bool MissionExecutor::getRobotPoseMap(
    geometry_msgs::msg::PoseStamped& pose)
{
    try
    {
        auto tf = tf_buffer_->lookupTransform(
            "map", "base_link", tf2::TimePointZero);
        pose.header = tf.header;
        pose.pose.position.x = tf.transform.translation.x;
        pose.pose.position.y = tf.transform.translation.y;
        pose.pose.position.z = tf.transform.translation.z;
        pose.pose.orientation = tf.transform.rotation;
        return true;
    }
    catch (const tf2::TransformException& exc)
    {
        RCLCPP_WARN(get_logger(), "map -> base_link 不可用: %s", exc.what());
        return false;
    }
}

bool MissionExecutor::getLatestApproachGoal(
    geometry_msgs::msg::PoseStamped& goal, bool warn)
{
    std::lock_guard<std::mutex> lock(trash_mutex_);
    if (!have_approach_goal_)
    {
        return false;
    }
    const double age = (now() - latest_approach_goal_time_).seconds();
    if (age > get_parameter("approach_goal_timeout").as_double())
    {
        if (warn)
        {
            RCLCPP_WARN(get_logger(),
                        "Latest /trash/approach_goal is %.1fs old, ignore",
                        age);
        }
        return false;
    }
    goal = latest_approach_goal_;
    return true;
}

bool MissionExecutor::waitForFreshApproachGoal(
    geometry_msgs::msg::PoseStamped& goal, double timeout_sec)
{
    // front_perception 在 stationary_confirm 后还要稳定 goal_settle_sec
    // 才发布 approach_goal，所以这里不能只检查一次，必须等它到齐。
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(timeout_sec);
    while (rclcpp::ok() && !mission_cancel_)
    {
        if (getLatestApproachGoal(goal, false))
        {
            return true;
        }
        if (std::chrono::steady_clock::now() >= deadline)
        {
            break;
        }
        rclcpp::sleep_for(100ms);
    }
    return false;
}

bool MissionExecutor::approachToBottleMapGoal(
    geometry_msgs::msg::PoseStamped& retreat_pose,
    bool& goal_available)
{
    goal_available = false;
    if (!get_parameter("use_map_goal_approach").as_bool())
    {
        return false;
    }
    if (!getRobotPoseMap(retreat_pose))
    {
        return false;
    }
    geometry_msgs::msg::PoseStamped goal;
    if (!waitForFreshApproachGoal(goal, 5.0))
    {
        RCLCPP_WARN(get_logger(),
                    "/trash/approach_goal 不可用，回退直线逼近");
        return false;
    }
    goal_available = true;
    RCLCPP_INFO(get_logger(),
                "Approach using map goal: [%.3f, %.3f, yaw from quaternion]",
                goal.pose.position.x, goal.pose.position.y);
    return navigateTo(goal, false) == NavResult::OK;
}

bool MissionExecutor::setPTZPose(float pan, float tilt, float zoom)
{
    RCLCPP_INFO(get_logger(), "setPTZPose: checking service...");
    if (!ptz_client_->wait_for_service(5s))
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
    if (!capture_client_->wait_for_service(5s))
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
    auto make_trigger_action = [node](const std::string& srv_name,
                                      double timeout_sec = 120.0) {
        auto client = node->create_client<std_srvs::srv::Trigger>(srv_name);
        return [node, client, srv_name, timeout_sec](
                   const hk_camera::msg::MissionWaypoint&,
                   const std::string&) {
            if (!client->wait_for_service(5s))
            {
                node->setActionDetail(srv_name + " service unavailable");
                RCLCPP_ERROR(rclcpp::get_logger("mission_executor"),
                             "%s NOT available",
                             srv_name.c_str());
                return false;
            }
            auto future = client->async_send_request(
                std::make_shared<std_srvs::srv::Trigger::Request>());
            // 抓取/放置含多段 movej/movel，耗时长，超时给足
            if (future.wait_for(std::chrono::duration<double>(timeout_sec)) !=
                std::future_status::ready)
            {
                node->setActionDetail(srv_name + " timed out");
                RCLCPP_ERROR(rclcpp::get_logger("mission_executor"),
                             "%s timeout", srv_name.c_str());
                return false;
            }
            auto res = future.get();
            RCLCPP_INFO(rclcpp::get_logger("mission_executor"),
                        "%s result: success=%d, msg=%s",
                        srv_name.c_str(), res->success, res->message.c_str());
            if (!res->success)
                node->setActionDetail(res->message);
            return res->success;
        };
    };
    // ---- 抓取组合动作: 直线逼近 → 视觉抓取 → 直线退回 ----
    // YAML 路点写 standoff 位姿 (远离障碍的自由空间, yaw 对准目标),
    // 到位后由 behavior_server 做带碰撞检查的直线逼近, 抓取后原路退回
    node->declare_parameter("approach_distance", 1.5);  // 逼近距离 (m)
    node->declare_parameter("approach_speed", 0.1);     // 逼近速度 (m/s)

    auto grasp_trigger =
        node->create_client<std_srvs::srv::Trigger>("/yolo_grasp/grasp_hold");

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
        if (approach_ok && !grasp_trigger->wait_for_service(5s))
        {
            RCLCPP_ERROR(rclcpp::get_logger("mission_executor"),
                         "/yolo_grasp/grasp_hold NOT available "
                         "(yolo_grasp.py running?)");
        }
        else if (approach_ok)
        {
            auto future = grasp_trigger->async_send_request(
                std::make_shared<std_srvs::srv::Trigger::Request>());
            if (future.wait_for(120s) != std::future_status::ready)
            {
                RCLCPP_ERROR(rclcpp::get_logger("mission_executor"),
                             "/yolo_grasp/grasp_hold timeout");
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
    node->registerAction(
        "polish",
        make_trigger_action(
            "/elite_polish/run",
            node->get_parameter("polish_timeout_sec").as_double()));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
