#include "main_window.h"
#include <QDateTime>
#include <QMessageBox>

namespace hk_camera
{

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    camera_  = HKCamera::instance();
    decoder_ = new HKDecoder(this);

    setupUI();
    setupConnections();

    setWindowTitle("海康相机控制台 — ROS2 + Qt5");
    resize(960, 680);
}

MainWindow::~MainWindow()
{
    decoder_->stop();
    if (user_id_ >= 0)      camera_->logout(user_id_);
}

// ============================================================
// UI 搭建
// ============================================================

void MainWindow::setupUI()
{
    auto* central = new QWidget(this);
    setCentralWidget(central);
    auto* main_layout = new QHBoxLayout(central);

    // ---- 左侧: 预览 + 状态 ----
    auto* left_widget = new QWidget();
    auto* left_layout  = new QVBoxLayout(left_widget);

    // 视频预览区
    video_label_ = new QLabel("等待视频流...");
    video_label_->setMinimumSize(480, 360);
    video_label_->setAlignment(Qt::AlignCenter);
    video_label_->setStyleSheet(
        "QLabel { background-color: #1a1a1a; color: #888; "
        "border: 2px solid #555; border-radius: 4px; }");
    left_layout->addWidget(video_label_);

    // 状态栏
    status_label_ = new QLabel("设备状态: 未连接");
    status_label_->setStyleSheet("QLabel { color: #ff6600; font-weight: bold; }");
    left_layout->addWidget(status_label_);

    main_layout->addWidget(left_widget, 2);

    // ---- 右侧: 控制面板 (两列布局节省纵向空间) ----
    auto* right_widget = new QWidget();
    auto* right_layout  = new QGridLayout(right_widget);
    right_widget->setMaximumWidth(500);

    // 登录组
    login_group_ = new QGroupBox("设备登录");
    auto* login_layout = new QVBoxLayout(login_group_);

    ip_edit_   = new QLineEdit("192.168.1.64");
    port_edit_ = new QLineEdit("8000");
    user_edit_ = new QLineEdit("admin");
    pass_edit_ = new QLineEdit("a1234567");
    pass_edit_->setEchoMode(QLineEdit::Password);

    login_btn_  = new QPushButton("登 录");
    logout_btn_ = new QPushButton("登 出");
    logout_btn_->setEnabled(false);

    login_layout->addWidget(new QLabel("IP地址:"));
    login_layout->addWidget(ip_edit_);
    login_layout->addWidget(new QLabel("端口:"));
    login_layout->addWidget(port_edit_);
    login_layout->addWidget(new QLabel("用户名:"));
    login_layout->addWidget(user_edit_);
    login_layout->addWidget(new QLabel("密码:"));
    login_layout->addWidget(pass_edit_);
    login_layout->addWidget(login_btn_);
    login_layout->addWidget(logout_btn_);

    right_layout->addWidget(login_group_, 0, 0);

    // 取流组
    stream_group_ = new QGroupBox("视频控制");
    auto* stream_layout = new QVBoxLayout(stream_group_);

    channel_combo_ = new QComboBox();
    for (int i = 1; i <= 16; i++)
        channel_combo_->addItem(QString("通道 %1").arg(i), i);

    start_stream_btn_ = new QPushButton("开始取流");
    stop_stream_btn_  = new QPushButton("停止取流");
    snapshot_btn_     = new QPushButton("快照模式 (1Hz 低CPU)");
    capture_btn_      = new QPushButton("抓 图");
    start_stream_btn_->setEnabled(false);
    stop_stream_btn_->setEnabled(false);
    snapshot_btn_->setEnabled(false);
    capture_btn_->setEnabled(false);
    snapshot_btn_->setCheckable(true);

    stream_layout->addWidget(new QLabel("通道选择:"));
    stream_layout->addWidget(channel_combo_);
    stream_layout->addWidget(start_stream_btn_);
    stream_layout->addWidget(stop_stream_btn_);
    stream_layout->addWidget(snapshot_btn_);
    stream_layout->addWidget(capture_btn_);

    right_layout->addWidget(stream_group_, 0, 1);

    // 云台控制组
    ptz_group_ = new QGroupBox("云台控制");
    auto* ptz_layout = new QGridLayout(ptz_group_);

    ptz_up_btn_    = new QPushButton("▲ 上");
    ptz_down_btn_  = new QPushButton("▼ 下");
    ptz_left_btn_  = new QPushButton("◄ 左");
    ptz_right_btn_ = new QPushButton("右 ►");
    ptz_stop_btn_  = new QPushButton("■ 停");

    ptz_layout->addWidget(ptz_up_btn_,    0, 1);
    ptz_layout->addWidget(ptz_left_btn_,   1, 0);
    ptz_layout->addWidget(ptz_stop_btn_,   1, 1);
    ptz_layout->addWidget(ptz_right_btn_,  1, 2);
    ptz_layout->addWidget(ptz_down_btn_,   2, 1);

    // 变倍
    auto* zoom_layout = new QHBoxLayout();
    zoom_in_btn_  = new QPushButton("+ 放大");
    zoom_out_btn_ = new QPushButton("- 缩小");
    zoom_layout->addWidget(zoom_in_btn_);
    zoom_layout->addWidget(zoom_out_btn_);
    ptz_layout->addLayout(zoom_layout, 3, 0, 1, 3);

    // 预置点
    auto* preset_layout = new QHBoxLayout();
    preset_spin_      = new QSpinBox();
    preset_set_btn_   = new QPushButton("设置");
    preset_goto_btn_  = new QPushButton("调用");
    preset_clear_btn_ = new QPushButton("清除");
    preset_spin_->setRange(1, 256);
    preset_spin_->setValue(1);
    preset_spin_->setToolTip("预置点编号");
    preset_layout->addWidget(new QLabel("预置点:"));
    preset_layout->addWidget(preset_spin_);
    preset_layout->addWidget(preset_set_btn_);
    preset_layout->addWidget(preset_goto_btn_);
    preset_layout->addWidget(preset_clear_btn_);
    ptz_layout->addLayout(preset_layout, 4, 0, 1, 3);

    ptz_group_->setEnabled(false);
    right_layout->addWidget(ptz_group_, 1, 0);

    // 巡航控制组
    cruise_group_ = new QGroupBox("预置点巡航");
    auto* cruise_layout = new QVBoxLayout(cruise_group_);

    auto* presets_row = new QHBoxLayout();
    presets_row->addWidget(new QLabel("预置点序列:"));
    cruise_presets_edit_ = new QLineEdit("1");
    cruise_presets_edit_->setToolTip("逗号分隔, 例如: 1,3,5,7");
    presets_row->addWidget(cruise_presets_edit_);
    cruise_layout->addLayout(presets_row);

    auto* dwell_row = new QHBoxLayout();
    dwell_row->addWidget(new QLabel("驻留时间:"));
    cruise_dwell_spin_ = new QSpinBox();
    cruise_dwell_spin_->setRange(1, 60);
    cruise_dwell_spin_->setValue(5);
    cruise_dwell_spin_->setSuffix(" 秒");
    dwell_row->addWidget(cruise_dwell_spin_);
    dwell_row->addStretch();
    cruise_layout->addLayout(dwell_row);

    auto* cruise_btn_row = new QHBoxLayout();
    cruise_start_btn_ = new QPushButton("▶ 开始巡航");
    cruise_stop_btn_  = new QPushButton("■ 停止");
    cruise_stop_btn_->setEnabled(false);
    cruise_btn_row->addWidget(cruise_start_btn_);
    cruise_btn_row->addWidget(cruise_stop_btn_);
    cruise_layout->addLayout(cruise_btn_row);

    cruise_group_->setEnabled(false);
    right_layout->addWidget(cruise_group_, 1, 1);

    // 日志组
    auto* log_group = new QGroupBox("运行日志");
    auto* log_layout = new QVBoxLayout(log_group);
    log_edit_ = new QTextEdit();
    log_edit_->setReadOnly(true);
    log_edit_->setMaximumHeight(150);
    log_layout->addWidget(log_edit_);

    right_layout->addWidget(log_group, 2, 0, 1, 2);  // 跨两列
    main_layout->addWidget(right_widget);
}

// ============================================================
// 信号槽连接
// ============================================================

void MainWindow::setupConnections()
{
    // 按钮
    connect(login_btn_,  &QPushButton::clicked, this, &MainWindow::onLogin);
    connect(logout_btn_, &QPushButton::clicked, this, &MainWindow::onLogout);
    connect(start_stream_btn_, &QPushButton::clicked,
            this, &MainWindow::onStartStream);
    connect(stop_stream_btn_, &QPushButton::clicked,
            this, &MainWindow::onStopStream);
    connect(snapshot_btn_, &QPushButton::clicked,
            this, &MainWindow::onToggleSnapshot);
    connect(capture_btn_, &QPushButton::clicked,
            this, &MainWindow::onCapture);

    // RTSP 解码器 → 显示
    connect(decoder_, &HKDecoder::frameDecoded,
            this, &MainWindow::onDecodedImage);

    // ---- 云台方向 (按下=动, 松开=停) ----
    connect(ptz_up_btn_,    &QPushButton::pressed,  this, &MainWindow::onPtzUpPressed);
    connect(ptz_up_btn_,    &QPushButton::released, this, &MainWindow::onPtzStop);
    connect(ptz_down_btn_,  &QPushButton::pressed,  this, &MainWindow::onPtzDownPressed);
    connect(ptz_down_btn_,  &QPushButton::released, this, &MainWindow::onPtzStop);
    connect(ptz_left_btn_,  &QPushButton::pressed,  this, &MainWindow::onPtzLeftPressed);
    connect(ptz_left_btn_,  &QPushButton::released, this, &MainWindow::onPtzStop);
    connect(ptz_right_btn_, &QPushButton::pressed,  this, &MainWindow::onPtzRightPressed);
    connect(ptz_right_btn_, &QPushButton::released, this, &MainWindow::onPtzStop);
    // 变倍
    connect(zoom_in_btn_,   &QPushButton::pressed,  this, &MainWindow::onZoomInPressed);
    connect(zoom_in_btn_,   &QPushButton::released, this, &MainWindow::onZoomStop);
    connect(zoom_out_btn_,  &QPushButton::pressed,  this, &MainWindow::onZoomOutPressed);
    connect(zoom_out_btn_,  &QPushButton::released, this, &MainWindow::onZoomStop);
    // 预置点
    connect(preset_set_btn_,   &QPushButton::clicked, this, &MainWindow::onPtzSetPreset);
    connect(preset_goto_btn_,  &QPushButton::clicked, this, &MainWindow::onPtzGotoPreset);
    connect(preset_clear_btn_, &QPushButton::clicked, this, &MainWindow::onPtzClearPreset);
    // 巡航
    connect(cruise_start_btn_, &QPushButton::clicked, this, &MainWindow::onCruiseStart);
    connect(cruise_stop_btn_,  &QPushButton::clicked, this, &MainWindow::onCruiseStop);

    // 报警 + 错误
    connect(camera_, &HKCamera::alarmReceived,
            this, &MainWindow::onAlarm);
    connect(camera_, &HKCamera::errorOccurred,
            this, &MainWindow::onError);

    // 定时器刷新状态
    status_timer_ = new QTimer(this);
    connect(status_timer_, &QTimer::timeout, this, &MainWindow::updateStatus);
    status_timer_->start(1000);
}

// ============================================================
// 槽函数实现
// ============================================================

void MainWindow::onLogin()
{
    camera_->init();

    CameraLoginInfo info;
    info.ip       = ip_edit_->text().toStdString();
    info.port     = static_cast<uint16_t>(port_edit_->text().toInt());
    info.username = user_edit_->text().toStdString();
    info.password = pass_edit_->text().toStdString();

    log(QString("正在连接 %1:%2 ...")
        .arg(info.ip.c_str()).arg(info.port));

    user_id_ = camera_->login(info);

    if (user_id_ >= 0)
    {
        log(QString("✓ 登录成功, user_id=%1").arg(user_id_));
        setControlsEnabled(true);
    }
    else
    {
        log(QString("✗ 登录失败, 错误码=%1").arg(camera_->getLastError()));
    }
}

void MainWindow::onLogout()
{
    // 停掉所有取流模式
    decoder_->stop();
    if (snapshot_mode_)
    {
        snapshot_mode_ = false;
        snapshot_btn_->setChecked(false);
        snapshot_btn_->setText("快照模式 (1Hz 低CPU)");
        if (snapshot_timer_) { snapshot_timer_->stop(); delete snapshot_timer_; snapshot_timer_ = nullptr; }
    }
    if (cruise_running_) onCruiseStop();
    if (user_id_ >= 0)     { camera_->logout(user_id_);           user_id_     = -1; }

    setControlsEnabled(false);
    login_btn_->setEnabled(true);
    log("已登出");
}

void MainWindow::onStartStream()
{
    if (user_id_ < 0) return;

    // 如果快照模式开着, 先关掉
    if (snapshot_mode_)
    {
        snapshot_mode_ = false;
        snapshot_btn_->setChecked(false);
        snapshot_btn_->setText("快照模式 (1Hz 低CPU)");
        if (snapshot_timer_) { snapshot_timer_->stop(); delete snapshot_timer_; snapshot_timer_ = nullptr; }
    }

    int channel = channel_combo_->currentData().toInt();
    log(QString("开始取流 通道%1 (RTSP)...").arg(channel));

    CameraLoginInfo info;
    info.ip       = ip_edit_->text().toStdString();
    info.port     = static_cast<uint16_t>(port_edit_->text().toInt());
    info.username = user_edit_->text().toStdString();
    info.password = pass_edit_->text().toStdString();

    if (decoder_->start(info, channel))
    {
        log(QString("✓ RTSP 取流已启动"));
        start_stream_btn_->setEnabled(false);
        stop_stream_btn_->setEnabled(true);
    }
    else
    {
        log(QString("✗ RTSP 取流启动失败"));
    }
}

void MainWindow::onStopStream()
{
    decoder_->stop();

    log("取流已停止");
    start_stream_btn_->setEnabled(true);
    stop_stream_btn_->setEnabled(false);
}

void MainWindow::onCapture()
{
    if (user_id_ < 0) return;

    int  channel = channel_combo_->currentData().toInt();
    auto path    = QString("/home/nvidia/kybot_ws/src/hk_camera/pic_capture/%1.jpg")
                   .arg(QDateTime::currentSecsSinceEpoch());

    bool ok = camera_->captureJPEG(user_id_, channel, 0, path.toStdString());
    if (ok)
        log(QString("✓ 抓图成功: %1").arg(path));
    else
        log(QString("✗ 抓图失败, 错误码=%1").arg(camera_->getLastError()));
}

void MainWindow::onToggleSnapshot()
{
    if (snapshot_mode_)
    {
        // 关闭快照模式
        snapshot_mode_ = false;
        snapshot_btn_->setChecked(false);
        snapshot_btn_->setText("快照模式 (1Hz 低CPU)");
        if (snapshot_timer_)
        {
            snapshot_timer_->stop();
            delete snapshot_timer_;
            snapshot_timer_ = nullptr;
        }
        log("快照模式已关闭");
    }
    else
    {
        if (user_id_ < 0)
        {
            log("请先登录");
            snapshot_btn_->setChecked(false);
            return;
        }

        // 如果正在取流, 先停掉
        if (decoder_->isRunning())
        {
            decoder_->stop();
            start_stream_btn_->setEnabled(true);
            stop_stream_btn_->setEnabled(false);
        }

        snapshot_mode_ = true;
        snapshot_btn_->setText("快照模式 (运行中...)");
        log("快照模式已开启 (1Hz, 相机端编码 JPEG, CPU 低负载)");

        // 立即抓一张
        onSnapshotTick();

        // 启动 1Hz 定时器
        snapshot_timer_ = new QTimer(this);
        connect(snapshot_timer_, &QTimer::timeout, this, &MainWindow::onSnapshotTick);
        snapshot_timer_->start(1000);
    }
}

void MainWindow::onSnapshotTick()
{
    if (!snapshot_mode_ || user_id_ < 0) return;

    int  channel = channel_combo_->currentData().toInt();
    // 用固定临时路径, 避免磁盘堆积
    auto path = QString("/tmp/hk_snapshot.jpg");

    if (camera_->captureJPEG(user_id_, channel, 0, path.toStdString()))
    {
        QPixmap pix(path);
        if (!pix.isNull())
        {
            video_label_->setPixmap(
                pix.scaled(video_label_->size(),
                           Qt::KeepAspectRatio,
                           Qt::SmoothTransformation));
        }
    }
}

// ============================================================
// 云台控制槽
// ============================================================

void MainWindow::ptzDo(DWORD cmd)
{
    if (user_id_ < 0) return;
    int ch = channel_combo_->currentData().toInt();
    camera_->ptzControl(user_id_, ch, cmd, 0);  // 0=start
}

void MainWindow::onPtzUpPressed()    { ptzDo(TILT_UP); }
void MainWindow::onPtzDownPressed()  { ptzDo(TILT_DOWN); }
void MainWindow::onPtzLeftPressed()  { ptzDo(PAN_LEFT); }
void MainWindow::onPtzRightPressed() { ptzDo(PAN_RIGHT); }
void MainWindow::onZoomInPressed()   { ptzDo(ZOOM_IN); }
void MainWindow::onZoomOutPressed()  { ptzDo(ZOOM_OUT); }

void MainWindow::onPtzStop()
{
    if (user_id_ < 0) return;
    int ch = channel_combo_->currentData().toInt();
    // 停所有方向 + 变倍 (0=stop, 但对全部方向发 stop 比较暴力但安全)
    camera_->ptzControl(user_id_, ch, TILT_UP, 1);
    camera_->ptzControl(user_id_, ch, PAN_LEFT, 1);
    camera_->ptzControl(user_id_, ch, ZOOM_IN, 1);
}

void MainWindow::onZoomStop() { onPtzStop(); }

void MainWindow::onPtzSetPreset()
{
    if (user_id_ < 0) return;
    int ch = channel_combo_->currentData().toInt();
    int idx = preset_spin_->value();
    camera_->ptzPreset(user_id_, ch, SET_PRESET, idx);
    log(QString("设置预置点 %1").arg(idx));
}

void MainWindow::onPtzGotoPreset()
{
    if (user_id_ < 0) return;
    int ch = channel_combo_->currentData().toInt();
    int idx = preset_spin_->value();
    camera_->ptzPreset(user_id_, ch, GOTO_PRESET, idx);
    log(QString("转到预置点 %1").arg(idx));
}

void MainWindow::onPtzClearPreset()
{
    if (user_id_ < 0) return;
    int ch = channel_combo_->currentData().toInt();
    int idx = preset_spin_->value();
    camera_->ptzPreset(user_id_, ch, CLE_PRESET, idx);
    log(QString("清除预置点 %1").arg(idx));
}

// ============================================================
// 巡航控制
// ============================================================

void MainWindow::onCruiseStart()
{
    if (user_id_ < 0 || cruise_running_) return;

    // 解析预置点序列
    QString text = cruise_presets_edit_->text();
    QStringList parts = text.split(",", Qt::SkipEmptyParts);
    std::vector<int> presets;
    for (const auto& p : parts)
    {
        bool ok;
        int v = p.trimmed().toInt(&ok);
        if (ok && v >= 1 && v <= 256)
            presets.push_back(v);
    }
    if (presets.empty())
    {
        log("无效的预置点序列");
        return;
    }

    cruise_running_ = true;
    cruise_index_   = 0;
    cruise_start_btn_->setEnabled(false);
    cruise_stop_btn_->setEnabled(true);
    cruise_presets_edit_->setEnabled(false);
    cruise_dwell_spin_->setEnabled(false);

    log(QString("巡航开始, %1个预置点, 驻留%2秒")
        .arg(presets.size()).arg(cruise_dwell_spin_->value()));

    // 立即转到第一个点
    int ch = channel_combo_->currentData().toInt();
    camera_->ptzPreset(user_id_, ch, GOTO_PRESET, presets[0]);
    log(QString("  → 预置点 %1").arg(presets[0]));

    // 启动定时器
    cruise_timer_ = new QTimer(this);
    connect(cruise_timer_, &QTimer::timeout, this, [this]() {
        onCruiseTick();
    });
    cruise_timer_->start(cruise_dwell_spin_->value() * 1000);
}

void MainWindow::onCruiseTick()
{
    if (!cruise_running_ || user_id_ < 0) return;

    // 解析当前序列
    QString text = cruise_presets_edit_->text();
    QStringList parts = text.split(",", Qt::SkipEmptyParts);
    std::vector<int> presets;
    for (const auto& p : parts)
    {
        bool ok;
        int v = p.trimmed().toInt(&ok);
        if (ok && v >= 1 && v <= 256)
            presets.push_back(v);
    }
    if (presets.empty()) { onCruiseStop(); return; }

    // 下一个预置点 (循环)
    cruise_index_ = (cruise_index_ + 1) % presets.size();
    int ch = channel_combo_->currentData().toInt();
    camera_->ptzPreset(user_id_, ch, GOTO_PRESET, presets[cruise_index_]);
    log(QString("  → 预置点 %1").arg(presets[cruise_index_]));
}

void MainWindow::onCruiseStop()
{
    cruise_running_ = false;
    cruise_index_   = 0;
    if (cruise_timer_) { cruise_timer_->stop(); delete cruise_timer_; cruise_timer_ = nullptr; }
    cruise_start_btn_->setEnabled(true);
    cruise_stop_btn_->setEnabled(false);
    cruise_presets_edit_->setEnabled(true);
    cruise_dwell_spin_->setEnabled(true);
    log("巡航已停止");
}

void MainWindow::onDecodedImage(const QImage& image)
{
    if (image.isNull()) return;
    video_label_->setPixmap(
        QPixmap::fromImage(image).scaled(
            video_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void MainWindow::onAlarm(const AlarmEvent& alarm)
{
    log(QString("[告警] type=%1, channels detected")
        .arg(alarm.alarm_type));
}

void MainWindow::onError(int code, const QString& msg)
{
    log(QString("[错误 %1] %2").arg(code).arg(msg));
}

void MainWindow::updateStatus()
{
    if (user_id_ >= 0 && decoder_->isRunning())
    {
        status_label_->setText("设备状态: ● 取流中");
        status_label_->setStyleSheet("QLabel { color: #00cc66; font-weight: bold; }");
    }
    else if (user_id_ >= 0)
    {
        status_label_->setText("设备状态: ● 已连接 (未取流)");
        status_label_->setStyleSheet("QLabel { color: #ffaa00; font-weight: bold; }");
    }
    else
    {
        status_label_->setText("设备状态: ● 未连接");
        status_label_->setStyleSheet("QLabel { color: #ff6600; font-weight: bold; }");
    }
}

// ============================================================
// 辅助
// ============================================================

void MainWindow::log(const QString& msg)
{
    QString ts = QDateTime::currentDateTime().toString("hh:mm:ss");
    log_edit_->append(QString("[%1] %2").arg(ts, msg));
}

void MainWindow::setControlsEnabled(bool enabled)
{
    login_btn_->setEnabled(!enabled);
    logout_btn_->setEnabled(enabled);
    start_stream_btn_->setEnabled(enabled);
    stop_stream_btn_->setEnabled(false);
    snapshot_btn_->setEnabled(enabled);
    capture_btn_->setEnabled(enabled);
    ptz_group_->setEnabled(enabled);
    cruise_group_->setEnabled(enabled);
}

} // namespace hk_camera
