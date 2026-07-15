#include "my_rviz_panel/my_panel.hpp"
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <QFileDialog>
#include <QMessageBox>
#include <QMetaObject>
#include <fstream>
#include <sstream>

namespace my_rviz_panel {

MyPanel::MyPanel(QWidget* parent) : rviz_common::Panel(parent) {
    // 1. 创建 UI 布局
    auto* main_layout = new QVBoxLayout(this);

    // 点位显示列表
    list_widget_ = new QListWidget(this);
    main_layout->addWidget(new QLabel("Recorded Waypoints:", this));
    main_layout->addWidget(list_widget_);

    // 按钮布局 - 点位管理
    auto* btn_layout_1 = new QHBoxLayout();
    btn_record_ = new QPushButton("Record Pose", this);
    btn_delete_ = new QPushButton("Delete Selected", this);
    btn_clear_ = new QPushButton("Clear All", this);
    btn_layout_1->addWidget(btn_record_);
    btn_layout_1->addWidget(btn_delete_);
    btn_layout_1->addWidget(btn_clear_);
    main_layout->addLayout(btn_layout_1);

    // 按钮布局 - 文件存取
    auto* btn_layout_2 = new QHBoxLayout();
    btn_save_ = new QPushButton("Save to File", this);
    btn_load_ = new QPushButton("Load from File", this);
    btn_layout_2->addWidget(btn_save_);
    btn_layout_2->addWidget(btn_load_);
    main_layout->addLayout(btn_layout_2);

    // 状态提示
    status_label_ = new QLabel("Status: Idle", this);
    status_label_->setStyleSheet("font-weight: bold; color: blue;");
    main_layout->addWidget(status_label_);

    // 按钮布局 - 导航控制
    auto* btn_layout_3 = new QHBoxLayout();
    btn_start_nav_ = new QPushButton("Start Waypoint Nav", this);
    btn_start_nav_->setStyleSheet("background-color: green; color: white;");
    btn_cancel_nav_ = new QPushButton("Cancel Nav", this);
    btn_cancel_nav_->setStyleSheet("background-color: red; color: white;");
    btn_layout_3->addWidget(btn_start_nav_);
    btn_layout_3->addWidget(btn_cancel_nav_);
    main_layout->addLayout(btn_layout_3);

    // 2. 绑定槽函数
    connect(btn_record_, &QPushButton::clicked, this, &MyPanel::onRecordClicked);
    connect(btn_delete_, &QPushButton::clicked, this, &MyPanel::onDeleteClicked);
    connect(btn_clear_, &QPushButton::clicked, this, &MyPanel::onClearClicked);
    connect(btn_save_, &QPushButton::clicked, this, &MyPanel::onSaveClicked);
    connect(btn_load_, &QPushButton::clicked, this, &MyPanel::onLoadClicked);
    connect(btn_start_nav_, &QPushButton::clicked, this, &MyPanel::onStartNavClicked);
    connect(btn_cancel_nav_, &QPushButton::clicked, this, &MyPanel::onCancelNavClicked);
}

void MyPanel::onInitialize() {
    // 获取 RViz2 的 ROS 2 原始节点
    node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    // 初始化 TF2 监听器
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 初始化 Nav2 Action 客户端
    action_client_ = rclcpp_action::create_client<FollowWaypoints>(node_, "follow_waypoints");
}

// 槽：记录小车当前位姿
void MyPanel::onRecordClicked() {
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        // 查找最新一帧从 map 到 base_link 的坐标转换
        transformStamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = node_->get_clock()->now();
        pose.pose.position.x = transformStamped.transform.translation.x;
        pose.pose.position.y = transformStamped.transform.translation.y;
        pose.pose.position.z = transformStamped.transform.translation.z;
        pose.pose.orientation = transformStamped.transform.rotation;

        waypoints_.push_back(pose);
        updateListWidget();
        status_label_->setText("Status: Position Recorded.");
    } catch (const tf2::TransformException &ex) {
        QMessageBox::warning(this, "TF Error", QString("Could not find coordinate transform: ") + ex.what());
    }
}

// 槽：删除选中的点
void MyPanel::onDeleteClicked() {
    int curr_row = list_widget_->currentRow();
    if (curr_row >= 0 && curr_row < static_cast<int>(waypoints_.size())) {
        waypoints_.erase(waypoints_.begin() + curr_row);
        updateListWidget();
        status_label_->setText("Status: Waypoint Deleted.");
    }
}

// 槽：清空列表
void MyPanel::onClearClicked() {
    waypoints_.clear();
    updateListWidget();
    status_label_->setText("Status: List Cleared.");
}

// 槽：保存到 TXT
void MyPanel::onSaveClicked() {
    if (waypoints_.empty()) {
        QMessageBox::warning(this, "Save Warning", "Waypoint list is empty.");
        return;
    }

    QString filename = QFileDialog::getSaveFileName(this, "Save Waypoints", "", "Text Files (*.txt);;All Files (*)");
    if (filename.isEmpty()) return;

    std::ofstream outfile(filename.toStdString());
    if (outfile.is_open()) {
        outfile << "# x, y, z, qx, qy, qz, qw\n";
        for (const auto& wp : waypoints_) {
            outfile << wp.pose.position.x << ","
                    << wp.pose.position.y << ","
                    << wp.pose.position.z << ","
                    << wp.pose.orientation.x << ","
                    << wp.pose.orientation.y << ","
                    << wp.pose.orientation.z << ","
                    << wp.pose.orientation.w << "\n";
        }
        outfile.close();
        status_label_->setText("Status: Saved to file.");
    }
}

// 槽：从 TXT 载入
void MyPanel::onLoadClicked() {
    QString filename = QFileDialog::getOpenFileName(this, "Load Waypoints", "", "Text Files (*.txt);;All Files (*)");
    if (filename.isEmpty()) return;

    std::ifstream infile(filename.toStdString());
    if (!infile.is_open()) return;

    waypoints_.clear();
    std::string line;
    while (std::getline(infile, line)) {
        if (line.empty() || line[0] == '#') continue; // 跳过空行和注释
        
        std::stringstream ss(line);
        std::string val;
        std::vector<double> coords;
        while (std::getline(ss, val, ',')) {
            coords.push_back(std::stod(val));
        }

        if (coords.size() == 7) {
            geometry_msgs::msg::PoseStamped wp;
            wp.header.frame_id = "map";
            wp.pose.position.x = coords[0];
            wp.pose.position.y = coords[1];
            wp.pose.position.z = coords[2];
            wp.pose.orientation.x = coords[3];
            wp.pose.orientation.y = coords[4];
            wp.pose.orientation.z = coords[5];
            wp.pose.orientation.w = coords[6];
            waypoints_.push_back(wp);
        }
    }
    infile.close();
    updateListWidget();
    status_label_->setText("Status: Loaded from file.");
}

// 槽：启动 Nav2 多点导航
void MyPanel::onStartNavClicked() {
    if (waypoints_.empty()) {
        QMessageBox::warning(this, "Nav Warning", "Please record or load waypoints first.");
        return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        QMessageBox::critical(this, "Nav Error", "Nav2 '/follow_waypoints' action server not available!");
        return;
    }

    auto goal_msg = FollowWaypoints::Goal();
    // 赋值给 Goal 前刷新时间戳
    for (auto& wp : waypoints_) {
        wp.header.stamp = node_->get_clock()->now();
    }
    goal_msg.poses = waypoints_;

    auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
    
    // 反馈回调：更新状态标签（使用 invokeMethod 确保 Qt 线程安全）
    send_goal_options.feedback_callback =
        [this](rclcpp_action::ClientGoalHandle<FollowWaypoints>::SharedPtr,
               const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
            int idx = feedback->current_waypoint;
            QString status_text = QString("Status: Navigating to Point [%1]").arg(idx + 1);
            QMetaObject::invokeMethod(status_label_, "setText", Qt::QueuedConnection, Q_ARG(QString, status_text));
        };

    // 结束回调：更新状态标签
    send_goal_options.result_callback = 
        [this](const rclcpp_action::ClientGoalHandle<FollowWaypoints>::WrappedResult& result) {
            QString res_text;
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                res_text = "Status: Navigation Succeeded!";
            } else {
                res_text = "Status: Navigation Failed/Canceled.";
            }
            QMetaObject::invokeMethod(status_label_, "setText", Qt::QueuedConnection, Q_ARG(QString, res_text));
        };

    status_label_->setText("Status: Starting Navigation...");
    action_client_->async_send_goal(goal_msg, send_goal_options);
}

// 槽：取消当前导航
void MyPanel::onCancelNavClicked() {
    if (action_client_) {
        action_client_->async_cancel_all_goals();
        status_label_->setText("Status: Navigation Canceled.");
    }
}

// 辅助：更新列表框
void MyPanel::updateListWidget() {
    list_widget_->clear();
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        const auto& pos = waypoints_[i].pose.position;
        QString item_str = QString("Point %1: [X: %2, Y: %3]")
                           .arg(i + 1)
                           .arg(pos.x, 0, 'f', 2)
                           .arg(pos.y, 0, 'f', 2);
        list_widget_->addItem(item_str);
    }
}

} // namespace my_rviz_panel

// 注册插件
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(my_rviz_panel::MyPanel, rviz_common::Panel)