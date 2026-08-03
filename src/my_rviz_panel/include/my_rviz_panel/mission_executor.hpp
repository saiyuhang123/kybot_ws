#ifndef MY_RVIZ_PANEL_MISSION_EXECUTOR_HPP_
#define MY_RVIZ_PANEL_MISSION_EXECUTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "hk_camera/msg/mission_waypoint.hpp"
#include "hk_camera/msg/mission_status.hpp"
#include "hk_camera/srv/run_mission.hpp"
#include "hk_camera/srv/set_ptz_pose.hpp"
#include "hk_camera/srv/capture_picture.hpp"

#include <atomic>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace my_rviz_panel {

/**
 * @brief 巡检任务执行器节点
 *
 * 流程: 导航到点 → 设置云台 → 拍照 → 下一个点
 *
 * 接口:
 *   - Sub:  /mission/run      (RunMission)    启动任务
 *   - Sub:  /mission/cancel   (Trigger)       取消任务
 *   - Pub:  /mission/status   (MissionStatus) 实时状态
 *
 * 扩展机制:
 *   registerAction(name, handler) 注册自定义动作处理器
 *   extra_action 字段匹配时调用
 */
class MissionExecutor : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    using ActionHandler = std::function<bool(
        const hk_camera::msg::MissionWaypoint& wp,
        const std::string& action_name)>;

    explicit MissionExecutor(const std::string& name = "mission_executor");
    ~MissionExecutor();

    /// 注册扩展动作处理器
    void registerAction(const std::string& name, ActionHandler handler);

    bool driveDistance(double distance, double speed, bool forward,
                       double& traveled_distance);

private:
    // ---- ROS 接口 ----
    void setupServices();
    void setupClients();

    // ---- 任务控制 ----
    void onRunMission(
        const hk_camera::srv::RunMission::Request::SharedPtr req,
        hk_camera::srv::RunMission::Response::SharedPtr res);
    void onCancelMission(
        const std_srvs::srv::Trigger::Request::SharedPtr req,
        std_srvs::srv::Trigger::Response::SharedPtr res);

    // ---- 任务执行 (独立线程) ----
    void executeMission();
    bool navigateTo(const geometry_msgs::msg::PoseStamped& pose);
    bool setPTZPose(float pan, float tilt, float zoom);
    bool capturePicture();
    void publishStatus(uint8_t state, uint8_t index, const std::string& msg);
    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void publishVelocity(double linear_x);
    void stopBase();
    bool getFrontObstacleDistance(double& distance) const;

    // ---- 状态 ----
    std::vector<hk_camera::msg::MissionWaypoint> waypoints_;
    std::atomic<bool> mission_running_{false};
    std::atomic<bool> mission_cancel_{false};
    std::thread mission_thread_;

    // ---- 扩展动作 ----
    std::map<std::string, ActionHandler> action_handlers_;

    // ---- ROS 接口 ----
    rclcpp::Service<hk_camera::srv::RunMission>::SharedPtr run_mission_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_mission_srv_;
    rclcpp::Publisher<hk_camera::msg::MissionStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    mutable std::mutex sensor_mutex_;
    nav_msgs::msg::Odometry latest_odom_;
    sensor_msgs::msg::LaserScan latest_scan_;
    rclcpp::Time latest_odom_time_;
    rclcpp::Time latest_scan_time_;
    bool have_odom_{false};
    bool have_scan_{false};

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Client<hk_camera::srv::SetPTZPose>::SharedPtr ptz_client_;
    rclcpp::Client<hk_camera::srv::CapturePicture>::SharedPtr capture_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr login_client_;
    // 机械臂收拢 (yolo_grasp home2): 任务开始前调用, 可选
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_home2_client_;
};

} // namespace my_rviz_panel

#endif // MY_RVIZ_PANEL_MISSION_EXECUTOR_HPP_
