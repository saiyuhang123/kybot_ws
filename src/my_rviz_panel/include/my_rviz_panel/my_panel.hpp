#ifndef MY_RVIZ_PANEL_MY_PANEL_HPP_
#define MY_RVIZ_PANEL_MY_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "hk_camera/msg/mission_waypoint.hpp"
#include "hk_camera/srv/run_mission.hpp"

// Qt 界面控件
#include <QPushButton>
#include <QListWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSpinBox>
#include <QCheckBox>
#include <QGroupBox>
#include <QLineEdit>

namespace my_rviz_panel {

/// 单个任务点位 (导航位姿 + PTZ + 动作)
struct MissionPoint {
    geometry_msgs::msg::PoseStamped nav_pose;
    int ptz_preset = 0;      // 0 = 不设置云台
    bool do_capture = true;
    std::string extra_action;
};

class MyPanel : public rviz_common::Panel {
    Q_OBJECT
public:
    explicit MyPanel(QWidget* parent = nullptr);
    virtual ~MyPanel() = default;

    void onInitialize() override;

protected Q_SLOTS:
    // 点位管理
    void onRecordClicked();
    void onDeleteClicked();
    void onClearClicked();
    void onSaveClicked();
    void onLoadClicked();

    // 导航 (纯导航, 不拍照)
    void onStartNavClicked();
    void onCancelNavClicked();

    // 巡检任务 (导航 + PTZ + 拍照)
    void onStartMissionClicked();
    void onCancelMissionClicked();

    // 编辑选中点的 PTZ
    void onSelectionChanged();

private:
    // ROS 2
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;
    rclcpp::Client<hk_camera::srv::RunMission>::SharedPtr mission_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_mission_client_;

    // 数据
    std::vector<MissionPoint> mission_points_;

    // ---- UI 控件 ----
    QListWidget* list_widget_;

    // 点位管理按钮
    QPushButton* btn_record_;
    QPushButton* btn_delete_;
    QPushButton* btn_clear_;
    QPushButton* btn_save_;
    QPushButton* btn_load_;

    // PTZ 编辑区
    QGroupBox* ptz_edit_group_;
    QSpinBox*  ptz_preset_spin_;
    QCheckBox* do_capture_check_;
    QLineEdit* extra_action_edit_;
    QPushButton* btn_apply_ptz_;

    // 导航控制
    QPushButton* btn_start_nav_;
    QPushButton* btn_cancel_nav_;

    // 巡检任务
    QPushButton* btn_start_mission_;
    QPushButton* btn_cancel_mission_;

    QLabel* status_label_;

    // 辅助
    void updateListWidget();
    void updatePtzEditFromSelection();
};

} // namespace my_rviz_panel

#endif // MY_RVIZ_PANEL_MY_PANEL_HPP_
