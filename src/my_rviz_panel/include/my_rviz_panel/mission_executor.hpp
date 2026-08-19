#ifndef MY_RVIZ_PANEL_MISSION_EXECUTOR_HPP_
#define MY_RVIZ_PANEL_MISSION_EXECUTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "trash_mission_interfaces/msg/trash_target.hpp"
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

    /// 导航结果（M3：支持被瓶子中断）
    enum class NavResult {
        OK,
        FAILED,
        INTERRUPTED,
    };

    explicit MissionExecutor(const std::string& name = "mission_executor");
    ~MissionExecutor();

    /// 注册扩展动作处理器
    void registerAction(const std::string& name, ActionHandler handler);

    /// 保存动作服务返回的详细结果，供任务状态透传
    void setActionDetail(const std::string& detail);

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
    NavResult navigateTo(const geometry_msgs::msg::PoseStamped& pose,
                         bool allow_bottle_interrupt = true);
    bool pickupBottle(size_t waypoint_index);
    bool approachToBottle(double& traveled_distance);
    bool approachToBottleMapGoal(
        geometry_msgs::msg::PoseStamped& retreat_pose,
        bool& goal_available);
    bool getRobotPoseMap(geometry_msgs::msg::PoseStamped& pose);
    bool getLatestApproachGoal(geometry_msgs::msg::PoseStamped& goal,
                               bool warn = true);
    bool waitForFreshApproachGoal(geometry_msgs::msg::PoseStamped& goal,
                                  double timeout_sec);
    bool callTriggerService(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
        const std::string& name, double timeout_sec);
    bool setPTZPose(float pan, float tilt, float zoom);
    bool capturePicture();
    void publishStatus(uint8_t state, uint8_t index, const std::string& msg);
    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void onTrashTarget(
        const trash_mission_interfaces::msg::TrashTarget::SharedPtr msg);
    void onApproachGoal(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg);
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
    std::string action_detail_;

    // ---- ROS 接口 ----
    rclcpp::Service<hk_camera::srv::RunMission>::SharedPtr run_mission_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_mission_srv_;
    rclcpp::Publisher<hk_camera::msg::MissionStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<trash_mission_interfaces::msg::TrashTarget>::SharedPtr
        trash_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        approach_goal_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    mutable std::mutex sensor_mutex_;
    nav_msgs::msg::Odometry latest_odom_;
    sensor_msgs::msg::LaserScan latest_scan_;
    rclcpp::Time latest_odom_time_;
    rclcpp::Time latest_scan_time_;
    bool have_odom_{false};
    bool have_scan_{false};

    // ---- 瓶子中断状态 (M3) ----
    mutable std::mutex trash_mutex_;
    trash_mission_interfaces::msg::TrashTarget latest_trash_;
    bool have_trash_{false};
    rclcpp::Time trash_stamp_;
    // 最近一次有效瓶子距离（停车静态确认后开环逼近用）
    double last_good_bottle_dist_{-1.0};
    rclcpp::Time last_good_bottle_time_;
    // /trash/approach_goal: 感知节点算好的地图系停车位姿
    geometry_msgs::msg::PoseStamped latest_approach_goal_;
    rclcpp::Time latest_approach_goal_time_;
    bool have_approach_goal_{false};
    std::atomic<bool> bottle_interrupt_{false};
    std::atomic<bool> bottle_confirmed_{false};
    int bottle_candidate_count_{0};

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Client<hk_camera::srv::SetPTZPose>::SharedPtr ptz_client_;
    rclcpp::Client<hk_camera::srv::CapturePicture>::SharedPtr capture_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr login_client_;
    // 机械臂收拢 (yolo_grasp home2): 任务开始前调用, 可选
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_home2_client_;
    // 抓取/放置 (M3 自动捡瓶)
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr grasp_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr place_client_;
    // 打磨服务桥接（仅 end_effector_mode=polish 时使用）
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr polish_run_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr polish_cancel_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr polish_home_client_;

    std::string end_effector_mode_{"twofinger"};
};

} // namespace my_rviz_panel

#endif // MY_RVIZ_PANEL_MISSION_EXECUTOR_HPP_
