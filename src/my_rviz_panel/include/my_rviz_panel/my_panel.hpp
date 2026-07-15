#ifndef MY_RVIZ_PANEL_MY_PANEL_HPP_
#define MY_RVIZ_PANEL_MY_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>

// Qt 界面控件
#include <QPushButton>
#include <QListWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

namespace my_rviz_panel {

class MyPanel : public rviz_common::Panel {
    Q_OBJECT
public:
    explicit MyPanel(QWidget* parent = nullptr);
    virtual ~MyPanel() = default;

    // 重写 RViz Panel 的初始化函数
    void onInitialize() override;

protected Q_SLOTS:
    void onRecordClicked();   // 记录当前位置
    void onDeleteClicked();   // 删除选中的点
    void onClearClicked();    // 清空列表
    void onSaveClicked();     // 保存点位到文件
    void onLoadClicked();     // 从文件载入点位
    void onStartNavClicked(); // 开始多点导航
    void onCancelNavClicked();// 取消导航

private:
    // ROS 2 相关的变量
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;

    // 内存中保存的点位数据列表
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;

    // Qt UI 控件
    QListWidget* list_widget_;
    QPushButton* btn_record_;
    QPushButton* btn_delete_;
    QPushButton* btn_clear_;
    QPushButton* btn_save_;
    QPushButton* btn_load_;
    QPushButton* btn_start_nav_;
    QPushButton* btn_cancel_nav_;
    QLabel* status_label_;

    // 辅助函数：更新 QListWidget 的显示
    void updateListWidget();
};

} // namespace my_rviz_panel

#endif // MY_RVIZ_PANEL_MY_PANEL_HPP_