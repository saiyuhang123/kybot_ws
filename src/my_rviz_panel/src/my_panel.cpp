#include "my_rviz_panel/my_panel.hpp"
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <QFileDialog>
#include <QMessageBox>
#include <QMetaObject>
#include <QTimer>
#include <fstream>
#include <sstream>

namespace my_rviz_panel {

MyPanel::MyPanel(QWidget* parent) : rviz_common::Panel(parent) {
    auto* main_layout = new QVBoxLayout(this);

    // ---- 点位列表 ----
    list_widget_ = new QListWidget(this);
    main_layout->addWidget(new QLabel("Waypoints:", this));
    main_layout->addWidget(list_widget_);

    // ---- 点位管理按钮 ----
    auto* btn_layout_1 = new QHBoxLayout();
    btn_record_ = new QPushButton("Record Pose", this);
    btn_delete_ = new QPushButton("Delete", this);
    btn_clear_  = new QPushButton("Clear All", this);
    btn_layout_1->addWidget(btn_record_);
    btn_layout_1->addWidget(btn_delete_);
    btn_layout_1->addWidget(btn_clear_);
    main_layout->addLayout(btn_layout_1);

    // ---- 文件存取 ----
    auto* btn_layout_2 = new QHBoxLayout();
    btn_save_ = new QPushButton("Save", this);
    btn_load_ = new QPushButton("Load", this);
    btn_layout_2->addWidget(btn_save_);
    btn_layout_2->addWidget(btn_load_);
    main_layout->addLayout(btn_layout_2);

    // ---- PTZ 编辑区 ----
    ptz_edit_group_ = new QGroupBox("Selected Point - PTZ & Action", this);
    auto* ptz_layout = new QHBoxLayout(ptz_edit_group_);
    ptz_layout->setSpacing(4);

    ptz_layout->addWidget(new QLabel("Preset:"));
    ptz_preset_spin_ = new QSpinBox(this);
    ptz_preset_spin_->setRange(0, 256);
    ptz_preset_spin_->setValue(0);
    ptz_preset_spin_->setToolTip("0 = no PTZ, 1~256 = preset number");
    ptz_preset_spin_->setMaximumWidth(60);
    ptz_layout->addWidget(ptz_preset_spin_);

    do_capture_check_ = new QCheckBox("Capture", this);
    do_capture_check_->setChecked(true);
    ptz_layout->addWidget(do_capture_check_);

    ptz_layout->addWidget(new QLabel("Action:"));
    extra_action_edit_ = new QLineEdit(this);
    extra_action_edit_->setPlaceholderText("(optional)");
    extra_action_edit_->setMaximumWidth(80);
    ptz_layout->addWidget(extra_action_edit_);

    btn_apply_ptz_ = new QPushButton("Apply", this);
    btn_apply_ptz_->setToolTip("Apply PTZ settings to selected point");
    ptz_layout->addWidget(btn_apply_ptz_);

    main_layout->addWidget(ptz_edit_group_);

    // ---- 状态 ----
    status_label_ = new QLabel("Status: Idle", this);
    status_label_->setStyleSheet("font-weight: bold; color: blue;");
    main_layout->addWidget(status_label_);

    // ---- 导航控制 ----
    auto* btn_layout_3 = new QHBoxLayout();
    btn_start_nav_ = new QPushButton("Nav Only", this);
    btn_start_nav_->setStyleSheet("background-color: #1565c0; color: white;");
    btn_cancel_nav_ = new QPushButton("Cancel Nav", this);
    btn_cancel_nav_->setStyleSheet("background-color: #c62828; color: white;");
    btn_layout_3->addWidget(btn_start_nav_);
    btn_layout_3->addWidget(btn_cancel_nav_);
    main_layout->addLayout(btn_layout_3);

    // ---- 巡检任务 ----
    auto* btn_layout_4 = new QHBoxLayout();
    btn_start_mission_ = new QPushButton("Start Mission", this);
    btn_start_mission_->setStyleSheet(
        "QPushButton { background-color: #2e7d32; color: white; font-weight: bold; font-size: 13px; }"
        "QPushButton:hover { background-color: #388e3c; }");
    btn_cancel_mission_ = new QPushButton("Cancel Mission", this);
    btn_cancel_mission_->setStyleSheet("background-color: #e65100; color: white;");
    btn_layout_4->addWidget(btn_start_mission_);
    btn_layout_4->addWidget(btn_cancel_mission_);
    main_layout->addLayout(btn_layout_4);

    // ---- 信号槽 ----
    connect(btn_record_, &QPushButton::clicked, this, &MyPanel::onRecordClicked);
    connect(btn_delete_, &QPushButton::clicked, this, &MyPanel::onDeleteClicked);
    connect(btn_clear_,  &QPushButton::clicked, this, &MyPanel::onClearClicked);
    connect(btn_save_,   &QPushButton::clicked, this, &MyPanel::onSaveClicked);
    connect(btn_load_,   &QPushButton::clicked, this, &MyPanel::onLoadClicked);

    connect(btn_apply_ptz_, &QPushButton::clicked, this, [this]() {
        int row = list_widget_->currentRow();
        if (row >= 0 && row < static_cast<int>(mission_points_.size())) {
            mission_points_[row].ptz_preset = ptz_preset_spin_->value();
            mission_points_[row].do_capture = do_capture_check_->isChecked();
            mission_points_[row].extra_action = extra_action_edit_->text().toStdString();
            updateListWidget();
            status_label_->setText(QString("Status: PTZ applied to Point %1").arg(row + 1));
        }
    });

    connect(btn_start_nav_,  &QPushButton::clicked, this, &MyPanel::onStartNavClicked);
    connect(btn_cancel_nav_, &QPushButton::clicked, this, &MyPanel::onCancelNavClicked);
    connect(btn_start_mission_,  &QPushButton::clicked, this, &MyPanel::onStartMissionClicked);
    connect(btn_cancel_mission_, &QPushButton::clicked, this, &MyPanel::onCancelMissionClicked);

    connect(list_widget_, &QListWidget::currentRowChanged, this, &MyPanel::onSelectionChanged);
}

void MyPanel::onInitialize() {
    node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    action_client_ = rclcpp_action::create_client<FollowWaypoints>(node_, "follow_waypoints");
    mission_client_ = node_->create_client<hk_camera::srv::RunMission>("/mission/run");
    cancel_mission_client_ = node_->create_client<std_srvs::srv::Trigger>("/mission/cancel");
}

// ============================================================
// 点位管理
// ============================================================

void MyPanel::onRecordClicked() {
    try {
        auto ts = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

        MissionPoint mp;
        mp.nav_pose.header.frame_id = "map";
        mp.nav_pose.header.stamp = node_->get_clock()->now();
        mp.nav_pose.pose.position.x = ts.transform.translation.x;
        mp.nav_pose.pose.position.y = ts.transform.translation.y;
        mp.nav_pose.pose.position.z = ts.transform.translation.z;
        mp.nav_pose.pose.orientation = ts.transform.rotation;
        mp.ptz_preset = 0;
        mp.do_capture = true;

        mission_points_.push_back(mp);
        updateListWidget();
        status_label_->setText(QString("Status: Point %1 recorded")
                               .arg(mission_points_.size()));
    } catch (const tf2::TransformException& ex) {
        QMessageBox::warning(this, "TF Error", ex.what());
    }
}

void MyPanel::onDeleteClicked() {
    int row = list_widget_->currentRow();
    if (row >= 0 && row < static_cast<int>(mission_points_.size())) {
        mission_points_.erase(mission_points_.begin() + row);
        updateListWidget();
        status_label_->setText("Status: Point deleted.");
    }
}

void MyPanel::onClearClicked() {
    mission_points_.clear();
    updateListWidget();
    status_label_->setText("Status: Cleared.");
}

void MyPanel::onSaveClicked() {
    if (mission_points_.empty()) {
        QMessageBox::warning(this, "Save", "No points to save.");
        return;
    }
    QString filename = QFileDialog::getSaveFileName(this, "Save", "", "Text (*.txt)");
    if (filename.isEmpty()) return;

    std::ofstream f(filename.toStdString());
    f << "# x,y,z,qx,qy,qz,qw,ptz_preset,do_capture,extra_action\n";
    for (const auto& mp : mission_points_) {
        const auto& p = mp.nav_pose.pose;
        f << p.position.x << "," << p.position.y << "," << p.position.z << ","
          << p.orientation.x << "," << p.orientation.y << ","
          << p.orientation.z << "," << p.orientation.w << ","
          << mp.ptz_preset << "," << (mp.do_capture ? 1 : 0) << ","
          << mp.extra_action << "\n";
    }
    status_label_->setText("Status: Saved.");
}

void MyPanel::onLoadClicked() {
    QString filename = QFileDialog::getOpenFileName(this, "Load", "", "Text (*.txt)");
    if (filename.isEmpty()) return;

    std::ifstream f(filename.toStdString());
    if (!f.is_open()) return;

    mission_points_.clear();
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        std::string val;
        std::vector<std::string> tokens;
        while (std::getline(ss, val, ',')) tokens.push_back(val);

        if (tokens.size() >= 7) {
            MissionPoint mp;
            mp.nav_pose.header.frame_id = "map";
            mp.nav_pose.pose.position.x = std::stod(tokens[0]);
            mp.nav_pose.pose.position.y = std::stod(tokens[1]);
            mp.nav_pose.pose.position.z = std::stod(tokens[2]);
            mp.nav_pose.pose.orientation.x = std::stod(tokens[3]);
            mp.nav_pose.pose.orientation.y = std::stod(tokens[4]);
            mp.nav_pose.pose.orientation.z = std::stod(tokens[5]);
            mp.nav_pose.pose.orientation.w = std::stod(tokens[6]);
            if (tokens.size() >= 8) mp.ptz_preset = std::stoi(tokens[7]);
            if (tokens.size() >= 9) mp.do_capture = (std::stoi(tokens[8]) != 0);
            if (tokens.size() >= 10) mp.extra_action = tokens[9];
            mission_points_.push_back(mp);
        }
    }
    updateListWidget();
    status_label_->setText(QString("Status: Loaded %1 points").arg(mission_points_.size()));
}

// ============================================================
// 纯导航 (原功能)
// ============================================================

void MyPanel::onStartNavClicked() {
    if (mission_points_.empty()) {
        QMessageBox::warning(this, "Nav", "No waypoints.");
        return;
    }
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        QMessageBox::critical(this, "Nav", "Nav2 follow_waypoints not available!");
        return;
    }

    auto goal = FollowWaypoints::Goal();
    for (auto& mp : mission_points_) {
        mp.nav_pose.header.stamp = node_->get_clock()->now();
        goal.poses.push_back(mp.nav_pose);
    }

    auto opts = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
    opts.feedback_callback =
        [this](auto, const std::shared_ptr<const FollowWaypoints::Feedback> fb) {
            QString s = QString("Nav: Point [%1]").arg(fb->current_waypoint + 1);
            QMetaObject::invokeMethod(status_label_, "setText", Qt::QueuedConnection, Q_ARG(QString, s));
        };
    opts.result_callback =
        [this](const auto& result) {
            QString s = (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                        ? "Nav: Done!" : "Nav: Failed/Canceled.";
            QMetaObject::invokeMethod(status_label_, "setText", Qt::QueuedConnection, Q_ARG(QString, s));
        };

    status_label_->setText("Status: Navigating...");
    action_client_->async_send_goal(goal, opts);
}

void MyPanel::onCancelNavClicked() {
    if (action_client_) {
        action_client_->async_cancel_all_goals();
        status_label_->setText("Status: Nav canceled.");
    }
}

// ============================================================
// 巡检任务 (导航 + PTZ + 拍照)
// ============================================================

void MyPanel::onStartMissionClicked() {
    if (mission_points_.empty()) {
        QMessageBox::warning(this, "Mission", "No waypoints.");
        return;
    }
    if (!mission_client_->wait_for_service(std::chrono::seconds(3))) {
        QMessageBox::critical(this, "Mission",
            "Mission Executor not available!\n"
            "Run: ros2 launch my_rviz_panel mission_executor.launch.py");
        return;
    }

    // 先取消可能还在跑的旧导航
    if (action_client_)
        action_client_->async_cancel_all_goals();

    btn_start_mission_->setEnabled(false);

    auto request = std::make_shared<hk_camera::srv::RunMission::Request>();
    for (const auto& mp : mission_points_) {
        hk_camera::msg::MissionWaypoint wp;
        wp.nav_pose = mp.nav_pose;
        wp.pan = static_cast<float>(mp.ptz_preset);
        wp.tilt = 0;
        wp.zoom = 0;
        wp.do_capture = mp.do_capture;
        wp.extra_action = mp.extra_action;
        request->waypoints.push_back(wp);
    }

    auto future = mission_client_->async_send_request(request);
    status_label_->setText("Status: Starting mission...");

    // 延迟重新启用按钮
    QTimer::singleShot(3000, this, [this]() {
        btn_start_mission_->setEnabled(true);
    });
}

void MyPanel::onCancelMissionClicked() {
    if (!cancel_mission_client_->wait_for_service(std::chrono::seconds(2))) {
        status_label_->setText("Status: Cancel service not available");
        return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    cancel_mission_client_->async_send_request(request);
    status_label_->setText("Status: Mission cancel sent.");
}

// ============================================================
// PTZ 编辑
// ============================================================

void MyPanel::onSelectionChanged() {
    updatePtzEditFromSelection();
}

void MyPanel::updatePtzEditFromSelection() {
    int row = list_widget_->currentRow();
    if (row >= 0 && row < static_cast<int>(mission_points_.size())) {
        const auto& mp = mission_points_[row];
        ptz_preset_spin_->setValue(mp.ptz_preset);
        do_capture_check_->setChecked(mp.do_capture);
        extra_action_edit_->setText(QString::fromStdString(mp.extra_action));
    }
}

// ============================================================
// 列表显示
// ============================================================

void MyPanel::updateListWidget() {
    list_widget_->clear();
    for (size_t i = 0; i < mission_points_.size(); ++i) {
        const auto& pos = mission_points_[i].nav_pose.pose.position;
        const auto& mp = mission_points_[i];

        QString s = QString("P%1: [%.2f, %.2f]")
                    .arg(i + 1).arg(pos.x).arg(pos.y);
        if (mp.ptz_preset > 0)
            s += QString(" PTZ:%1").arg(mp.ptz_preset);
        if (mp.do_capture)
            s += " 📷";
        if (!mp.extra_action.empty())
            s += QString(" [%1]").arg(QString::fromStdString(mp.extra_action));

        list_widget_->addItem(s);
    }
}

} // namespace my_rviz_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(my_rviz_panel::MyPanel, rviz_common::Panel)
