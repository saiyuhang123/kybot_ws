#!/usr/bin/env python3
"""KYBOT 一键启动面板 (PyQt5, 单文件, 双击可运行)

分组管理: 定位Nav / 机械臂抓取(灵巧手|二指, 单选互斥) / 语音调度 / 语音前端 / RViz
- 状态灯查 ROS 图(节点/服务在线), 不查进程
- 停止杀整个进程组 (setsid + killpg), 杜绝 ros2 launch 留僵尸
- 日志分组 tab, 每组上限 5000 行循环覆盖
- 桌面快捷方式: Exec=python3 /home/nvidia/kybot_ws/tools/launcher.py
"""

import os
import signal
import subprocess
import sys
import threading
import time

KYBOT_WS = '/home/nvidia/kybot_ws'
ELITE_WS = '/home/nvidia/Documents/elite_robot_ws'

# ---------- 环境自举: 保证双击/任意终端可用 ----------
def _need_bootstrap():
    if 'ROS_DISTRO' not in os.environ:
        return True
    try:  # 只有 ROS 主环境没 source 工作区时, 探针 import 会全灭
        import hk_camera.msg  # noqa: F401
        return False
    except ImportError:
        return True

if _need_bootstrap() and os.environ.get('KYBOT_LAUNCHER_BOOT') != '1':
    # PYTHONPATH/LD_LIBRARY_PATH 对已启动进程无效, 注入环境后重启自身
    try:
        out = subprocess.check_output(
            ['bash', '-c',
             'source /opt/ros/humble/setup.bash && '
             'source %s/install/setup.bash && env' % KYBOT_WS],
            text=True, stderr=subprocess.DEVNULL)
        env = dict(os.environ)
        for line in out.splitlines():
            k, _, v = line.partition('=')
            if k:
                env[k] = v
        env['ROS_DOMAIN_ID'] = '42'
        env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        env['KYBOT_LAUNCHER_BOOT'] = '1'
        os.execvpe(sys.executable, [sys.executable] + sys.argv, env)
    except Exception:
        pass  # 失败则降级为"仅进程管理"模式继续
os.environ['ROS_DOMAIN_ID'] = '42'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

import yaml  # noqa: E402

from PyQt5.QtCore import QProcess, Qt, QTimer, pyqtSignal  # noqa: E402
from PyQt5.QtWidgets import (QApplication, QButtonGroup, QCheckBox,  # noqa: E402
                             QComboBox, QFileDialog, QFormLayout,
                             QGroupBox, QHBoxLayout, QLabel, QLineEdit,
                             QListWidget, QMainWindow, QMessageBox,
                             QPlainTextEdit, QPushButton, QRadioButton,
                             QSpinBox, QTabWidget, QVBoxLayout, QWidget)

SETUP_ENV = ('source /opt/ros/humble/setup.bash && '
             'export ROS_DOMAIN_ID=42 && '
             'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && ')
KYBOT_SETUP = SETUP_ENV + 'source %s/install/setup.bash && ' % KYBOT_WS
ELITE_SETUP = SETUP_ENV + 'source %s/install/setup.bash && ' % ELITE_WS

# ---------- 两种机械臂模式的 launch 配对 (按 A/B 流程文档) ----------
MODES = {
    'twofinger': {
        'title': '二指',
        'nav_cmd': KYBOT_SETUP + 'ros2 launch kybot_bringup '
                   'trash_pipeline.launch.py use_ocr:=false use_rviz:=true',
        'arm_cmds': [
            ('arm_main', '机械臂驱动(二指)',
             ELITE_SETUP + 'ros2 launch %s/biaoding/yolo_grasp_two_finger.launch.py '
             'run_grasp_main:=false' % ELITE_WS),
            ('arm_grasp', '抓取服务(二指)',
             ELITE_SETUP + 'cd %s/biaoding && python3 yolo_grasp.py '
             '--gripper two_finger --target-class bottle --headless' % ELITE_WS),
        ],
        'arm_grasp_delay_s': 10,  # 主 launch 起完再起 headless 抓取服务
    },
    'linkerhand': {
        'title': '灵巧手',
        'nav_cmd': KYBOT_SETUP + 'ros2 launch kybot_bringup bringup.launch.py',
        'arm_cmds': [
            ('arm_main', '机械臂抓取(灵巧手)',
             ELITE_SETUP + 'ros2 launch %s/biaoding/yolo_grasp.launch.py' % ELITE_WS),
        ],
        'arm_grasp_delay_s': 0,
    },
}

# 其他固定组
FIXED_CMDS = {
    'brain': ('语音调度', KYBOT_SETUP + 'ros2 launch kybot_brain kybot_brain.launch.py'),
    'aiui': ('语音前端', KYBOT_SETUP + 'ros2 launch robot_aiui robot_aiui.launch.py'),
    'camera': ('海康相机', KYBOT_SETUP + 'ros2 run hk_camera hk_camera_node'),
    'ocr': ('OCR识别', KYBOT_SETUP + 'ros2 launch ocr_node ocr_node.launch.py'),
    'rviz': ('RViz', KYBOT_SETUP + 'rviz2'),
}

# 建图组 (对应 ~/Documents/start_fastlioMapping.sh 的四个组件,
# 直接以子进程方式管理, 不弹 gnome-terminal, killpg 即可全停;
# delay 沿用原脚本的错峰秒数)
DOC = '/home/nvidia/Documents'
MAPPING_CMDS = [
    ('mapping_imu', '建图-IMU',
     SETUP_ENV + 'source %s/wit_ros2_imu_src/install/setup.bash && '
     'ros2 run wit_ros2_imu wit_ros2_imu --ros-args '
     '-p port:=/dev/ttyCH341USB0 -p baudrate:=921600' % DOC, 0),
    ('mapping_lidar', '建图-雷达',
     SETUP_ENV + 'source %s/rslidar_ros2_ws/install/setup.bash && '
     'ros2 launch rslidar_sdk start.py' % DOC, 5),
    ('mapping_rs', '建图-点云转换',
     SETUP_ENV + 'source %s/rs_to_velodyne-master/install/setup.bash && '
     'ros2 run rs_to_velodyne rs_to_velodyne' % DOC, 10),
    ('mapping_fastlio', '建图-FAST_LIO2',
     SETUP_ENV + 'source %s/fast_lio2_ws/install/setup.bash && '
     'ros2 launch fast_lio mapping.launch.py config_file:=rslidar_wit.yaml' % DOC, 18),
]
# 与建图互斥的组 (建图开启时这些必须停用)
MUTEX_WITH_MAPPING = ('nav', 'arm', 'brain', 'aiui')

# 任务点位页
MISSION_STATE_NAMES = {
    0: '空闲', 1: '导航中', 2: '云台运动中', 3: '拍照中', 4: '已完成',
    5: '失败', 6: '已取消', 7: '目标确认中', 8: '逼近目标中', 9: '抓取中',
    10: '放置中', 11: '退回中',
}
MISSION_FREE_STATES = {0, 4, 5, 6}
DEFAULT_WP_FILE = KYBOT_WS + '/location/location.yaml'  # 与 brain 共用

# 任务页的依赖服务 (与 nav 组互斥: bringup/trash_pipeline 也会拉起它们)
DEP_CMDS = {
    'dep_exec': ('任务调度 (mission_executor)',
                 KYBOT_SETUP + 'ros2 launch my_rviz_panel mission_executor.launch.py'),
    'dep_camera': ('海康相机 (hk_camera)',
                   KYBOT_SETUP + 'ros2 run hk_camera hk_camera_node'),
    'dep_ocr': ('OCR识别 (ocr_node)',
                KYBOT_SETUP + 'ros2 launch ocr_node ocr_node.launch.py'),
}
DEP_NODE_CHECKS = {
    'dep_exec': ('mission_executor',),
    'dep_camera': ('hk_camera_node',),
    'dep_ocr': ('ocr_node',),
}

LOG_LINE_LIMIT = 5000


class RosProbe:
    """内嵌 rclpy 探针: 查 ROS 图判断各组是否真的在线. rclpy 不可用时全部返回 False."""

    def __init__(self):
        self.node = None
        self.tf_buffer = None
        self.mission_status = None   # 最近一次 /mission/status
        self.capture_hook = None     # CAPTURING 状态跳变回调 (MainWindow 赋值)
        self.ocr_feedback_hook = None  # /ocr_feedback 话题回调
        self.on_error = None           # 探针异常上报 (MainWindow 赋值)
        self.init_log = []             # 初始化阶段的异常留底
        self._prev_status = None
        self._init_error = None
        # 第一阶段: 核心 (rclpy + 节点 + spin) —— 失败则探针全灭
        try:
            import rclpy
            rclpy.init(args=None)
            self.node = rclpy.create_node('kybot_launcher_probe')
            self._rclpy = rclpy
            self._thread = threading.Thread(target=self._spin, daemon=True)
            self._thread.start()
        except Exception as exc:
            self._init_error = 'rclpy 初始化失败: %s' % exc
            self.node = None
            return
        # 第二阶段: 各功能独立挂载, 单个失败只降级对应功能
        self._optional_init()

    def _optional_init(self):
        try:
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self.node)
        except Exception as exc:
            self._report_error('TF 监听不可用(录点功能降级): %s' % exc)
        try:
            from std_srvs.srv import Trigger
            from hk_camera.msg import MissionStatus
            from hk_camera.srv import RunMission
            self.run_cli = self.node.create_client(RunMission, '/mission/run')
            self.cancel_cli = self.node.create_client(Trigger, '/mission/cancel')
            self.node.create_subscription(
                MissionStatus, '/mission/status', self._on_mission_status, 10)
            self.login_cli = self.node.create_client(Trigger, '/hk_camera/login')
            self.stream_cli = self.node.create_client(Trigger,
                                                      '/hk_camera/start_stream')
        except Exception as exc:
            self._report_error('任务/相机接口不可用(需 source 工作区环境): %s' % exc)
        try:
            from std_msgs.msg import String
            from ocr_interfaces.srv import RecognizeText
            self.ocr_cli = self.node.create_client(RecognizeText,
                                                   '/ocr/recognize')
            self.node.create_subscription(
                String, '/ocr_feedback', self._on_ocr_feedback, 10)
        except Exception as exc:
            self._report_error('OCR 接口不可用: %s' % exc)

    def _on_mission_status(self, msg):
        self.mission_status = msg
        # CAPTURING 跳变 = 任务流程里拍照发生了, 触发 OCR
        prev, self._prev_status = self._prev_status, msg
        if (msg.state == 3 and self.capture_hook is not None
                and (prev is None or prev.state != 3)):
            try:
                self.capture_hook()
            except Exception:
                pass

    def _on_ocr_feedback(self, msg):
        if self.ocr_feedback_hook is not None:
            try:
                self.ocr_feedback_hook(msg.data)
            except Exception:
                pass

    def lookup_pose(self, target='map', source='base_link'):
        """查当前位姿 (录点用). 返回 dict(x..qw) 或 None."""
        if self.tf_buffer is None:
            return None
        try:
            from rclpy.time import Time
            tf = self.tf_buffer.lookup_transform(target, source, Time())
            t, q = tf.transform.translation, tf.transform.rotation
            return {'x': t.x, 'y': t.y, 'z': t.z,
                    'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w}
        except Exception:
            return None

    def _report_error(self, text):
        self.init_log.append(text)  # 留底, MainWindow 启动后冲进系统日志
        if self.on_error is not None:
            try:
                self.on_error(text)
            except Exception:
                pass

    def _spin(self):
        # 打不死: 任何异常重建 executor 继续; 线程死亡 = ROS 图永久过期
        from rclpy.executors import SingleThreadedExecutor
        while self.node is not None:
            try:
                ex = SingleThreadedExecutor()
                ex.add_node(self.node)
                while True:
                    ex.spin_once(timeout_sec=0.5)
            except Exception as exc:
                if self.node is None:
                    return
                self._report_error('探针 spin 异常(已自动恢复): %s' % exc)
                time.sleep(1.0)

    def available(self):
        return self.node is not None

    def has_node(self, name):
        try:
            return name in self.node.get_node_names()
        except Exception:
            return False

    def has_service(self, name):
        try:
            return name in [n for n, _t in
                            self.node.get_service_names_and_types()]
        except Exception:
            return False

    def has_publisher(self, topic):
        try:
            return self.node.count_publishers(topic) > 0
        except Exception:
            return False

    def shutdown(self):
        node = self.node
        self.node = None  # 先置空让 spin 循环退出
        if node is not None:
            try:
                node.destroy_node()
                self._rclpy.shutdown()
            except Exception:
                pass


class Proc:
    """一个被管进程: setsid 启动使 PGID==PID, 停止时 killpg 杀整组."""

    def __init__(self, key, title, cmd, log_widget, on_state_change):
        self.key = key
        self.title = title
        self.cmd = cmd
        self._log = log_widget
        self._on_state_change = on_state_change
        self.qp = QProcess()
        self.qp.setProcessChannelMode(QProcess.MergedChannels)
        self.qp.readyReadStandardOutput.connect(self._on_read)
        self.qp.finished.connect(self._on_finished)
        self._force_kill_timer = None

    def start(self):
        if self.running():
            return
        self._append('—— 启动: %s ——' % self.cmd.split('&&')[-1].strip()[:120])
        self.qp.start('setsid', ['bash', '-c', self.cmd])
        self._on_state_change()

    def running(self):
        return self.qp.state() != QProcess.NotRunning

    def stop(self):
        if not self.running():
            return
        pid = int(self.qp.processId())
        try:
            os.killpg(pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError, OSError):
            self.qp.terminate()
        self._append('—— 已发送停止(SIGTERM 进程组) ——')
        # 5 秒不死则 SIGKILL
        self._force_kill_timer = QTimer()
        self._force_kill_timer.setSingleShot(True)
        self._force_kill_timer.timeout.connect(lambda: self._force_kill(pid))
        self._force_kill_timer.start(5000)

    def _force_kill(self, pid):
        if self.running():
            try:
                os.killpg(pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError, OSError):
                self.qp.kill()
            self._append('—— 强制结束(SIGKILL) ——')

    def _on_read(self):
        text = bytes(self.qp.readAllStandardOutput()).decode('utf-8', 'replace')
        self._append(text.rstrip('\n'))

    def _on_finished(self):
        self._append('—— 进程已退出 ——')
        self._on_state_change()

    def _append(self, text):
        for line in text.split('\n'):
            if line.strip():
                self._log.appendPlainText(line)


class MainWindow(QMainWindow):
    # 跨线程信号: rclpy spin 线程 → GUI 线程的安全投递
    sig_sys = pyqtSignal(str)
    sig_ocr = pyqtSignal(str)
    sig_mission = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.setWindowTitle('KYBOT 启动面板')
        self.resize(1020, 660)
        self._procs = {}          # key -> Proc
        self._mode = 'twofinger'  # 默认二指
        self._seq_steps = []      # 一键启动的待执行步骤
        self._seq_wait_until = 0.0
        self._seq_ready_fn = None
        self._oneshots = []       # 一次性命令的 QProcess (防 GC)
        self._last_ocr_trigger = 0.0  # OCR 触发去抖 (monotonic)
        self._ocr_inflight = False    # OCR 调用在飞标志 (防并发)
        self._prev_ms_state = None    # 任务状态边沿检测 (循环重发用)
        self._mission_from_here = False  # 当前任务是否由本页发起 (循环重发用)

        self.probe = RosProbe()
        self._build_ui()
        self._apply_mode('twofinger', force=True)
        # OCR: 巡检拍照跳变触发 + brain 的 /ocr_feedback, 统一进结果区
        self.probe.capture_hook = self._on_capture_edge
        self.probe.ocr_feedback_hook = self._on_ocr_feedback
        # 跨线程信号槽
        self.sig_sys.connect(self._sys_log)
        self.sig_ocr.connect(self._ocr_append)
        self.sig_mission.connect(self._mission_started)
        # 探针异常上系统日志 (含初始化阶段留底的)
        self.probe.on_error = lambda msg: self.sig_sys.emit(msg)
        if not self.probe.available():
            self.sig_sys.emit('⚠ 探针初始化失败, 状态灯/任务功能不可用: %s'
                              % (self.probe._init_error or '未知'))
            self._status_label.setText('探针初始化失败, 请从终端启动查看报错')
        for msg in self.probe.init_log:
            self.sig_sys.emit('探针: %s' % msg)

        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._refresh_status)
        self._status_timer.start(2000)
        self._seq_timer = QTimer(self)
        self._seq_timer.timeout.connect(self._seq_tick)

    # ---------- UI ----------

    def _build_ui(self):
        self._pages = QTabWidget()
        self.setCentralWidget(self._pages)

        page1 = QWidget()
        root = QHBoxLayout(page1)
        self._pages.addTab(page1, '启动管理')

        left = QVBoxLayout()
        left.setSpacing(6)

        # 机械臂模式单选 (互斥: 同一末端只能装一种手)
        mode_box = QGroupBox('机械臂模式 (同一末端, 互斥)')
        mh = QHBoxLayout(mode_box)
        self._radio_two = QRadioButton('二指')
        self._radio_two.setChecked(True)
        self._radio_hand = QRadioButton('灵巧手')
        self._mode_group = QButtonGroup(self)
        self._mode_group.addButton(self._radio_two)
        self._mode_group.addButton(self._radio_hand)
        mh.addWidget(self._radio_two)
        mh.addWidget(self._radio_hand)
        left.addWidget(mode_box)
        self._radio_two.toggled.connect(
            lambda c: c and self._apply_mode('twofinger'))
        self._radio_hand.toggled.connect(
            lambda c: c and self._apply_mode('linkerhand'))

        # 分组行
        self._rows = {}
        groups_box = QGroupBox('功能组')
        gv = QVBoxLayout(groups_box)
        for key, title in [('nav', '定位 + Nav'), ('arm', '机械臂抓取'),
                           ('brain', '语音调度'), ('aiui', '语音前端'),
                           ('camera', '海康相机'), ('ocr', 'OCR识别'),
                           ('rviz', 'RViz'), ('mapping', '建图 (FAST_LIO2)')]:
            row = self._make_group_row(key, title, gv)
            self._rows[key] = row
        # 建图行加"保存建图"按钮 (调用 /map_save)
        self._btn_save_map = QPushButton('保存')
        self._btn_save_map.setFixedWidth(48)
        self._btn_save_map.clicked.connect(self._save_map)
        mh = self._rows['mapping']['btn_log'].parent().layout()
        mh.addWidget(self._btn_save_map)
        left.addWidget(groups_box)

        # 全局按钮
        self._btn_can = QPushButton('配置 CAN (can1/can2)')
        self._btn_can.clicked.connect(self._setup_can)
        left.addWidget(self._btn_can)

        self._chk_rviz = QCheckBox('一键启动时包含 RViz')
        left.addWidget(self._chk_rviz)

        self._btn_all = QPushButton('一键全部启动')
        self._btn_all.setStyleSheet('font-weight: bold; padding: 8px;')
        self._btn_all.clicked.connect(self._start_all)
        left.addWidget(self._btn_all)

        self._btn_stop_all = QPushButton('全部停止')
        self._btn_stop_all.clicked.connect(self._stop_all)
        left.addWidget(self._btn_stop_all)

        self._status_label = QLabel('就绪')
        self._status_label.setWordWrap(True)
        left.addWidget(self._status_label)
        left.addStretch(1)

        left_widget = QWidget()
        left_widget.setLayout(left)
        left_widget.setFixedWidth(330)
        root.addWidget(left_widget)

        # 右侧日志 tab
        self._tabs = QTabWidget()
        root.addWidget(self._tabs, 1)
        self._logs = {}
        self._add_log_tab('system', '系统')

        # 第二页: 任务点位
        self._mission = self._build_mission_page()
        self._pages.addTab(self._mission['widget'], '任务点位')

    def _make_group_row(self, key, title, parent_layout):
        w = QWidget()
        h = QHBoxLayout(w)
        h.setContentsMargins(2, 2, 2, 2)
        btn = QPushButton('启动')
        btn.setFixedWidth(64)
        btn.clicked.connect(lambda: self._toggle_group(key))
        name = QLabel(title)
        led = QLabel('●')
        led.setFixedWidth(20)
        self._set_led(led, 'off')
        btn_log = QPushButton('日志')
        btn_log.setFixedWidth(48)
        h.addWidget(btn)
        h.addWidget(name, 1)
        h.addWidget(led)
        h.addWidget(btn_log)
        parent_layout.addWidget(w)
        return {'btn': btn, 'led': led, 'btn_log': btn_log,
                'name': name, 'title': title}

    def _set_led(self, led, state):
        color = {'off': '#888888', 'run': '#888888',
                 'starting': '#e6a23c', 'online': '#2eb85c'}[state]
        led.setStyleSheet('color: %s; font-size: 16px;' % color)

    def _add_log_tab(self, key, title):
        edit = QPlainTextEdit()
        edit.setReadOnly(True)
        edit.document().setMaximumBlockCount(LOG_LINE_LIMIT)
        self._tabs.addTab(edit, title)
        self._logs[key] = edit
        return edit

    def _sys_log(self, text):
        self._logs['system'].appendPlainText('[%s] %s'
                                             % (time.strftime('%H:%M:%S'), text))

    # ---------- 任务点位页 ----------

    def _build_mission_page(self):
        w = QWidget()
        root = QHBoxLayout(w)

        # 左列: 工具条 + 点位列表
        left = QVBoxLayout()
        btns = QHBoxLayout()
        for text, fn in [('录当前点', self._wp_record),
                         ('删除', self._wp_delete),
                         ('清空', self._wp_clear),
                         ('加载', self._wp_load),
                         ('保存', self._wp_save)]:
            b = QPushButton(text)
            b.clicked.connect(fn)
            btns.addWidget(b)
        left.addLayout(btns)
        self._wp_list = QListWidget()
        self._wp_list.currentRowChanged.connect(self._wp_select)
        left.addWidget(self._wp_list, 1)
        root.addLayout(left, 3)

        # 右列: 编辑表单 + 任务控制
        right = QVBoxLayout()
        form_box = QGroupBox('选中点编辑 (名称供语音按名导航)')
        form = QFormLayout(form_box)
        self._wp_name = QLineEdit()
        self._wp_ptz = QSpinBox()
        self._wp_ptz.setRange(0, 256)
        self._wp_ptz.setToolTip('海康预置位号, 0 = 不动云台')
        self._wp_capture = QCheckBox('到达后拍照')
        self._wp_action = QComboBox()
        # (显示名, extra_action 值); 与 executor 注册的动作一致
        for label, val in [('无', ''), ('抓取 (grasp)', 'grasp'),
                           ('放置 (place)', 'place'),
                           ('收臂 (home2)', 'home2'),
                           ('预备 (ready)', 'ready')]:
            self._wp_action.addItem(label, val)
        form.addRow('名称:', self._wp_name)
        form.addRow('预置位:', self._wp_ptz)
        form.addRow('拍照:', self._wp_capture)
        form.addRow('动作:', self._wp_action)
        right.addWidget(form_box)
        btn_apply = QPushButton('应用到选中点')
        btn_apply.clicked.connect(self._wp_apply)
        right.addWidget(btn_apply)

        self._wp_file_label = QLabel('文件: %s' % DEFAULT_WP_FILE)
        self._wp_file_label.setWordWrap(True)
        right.addWidget(self._wp_file_label)

        # 依赖服务: 任务调度 / 海康相机 (与"定位+Nav"组互斥, 那边会拉起同款)
        dep_box = QGroupBox('依赖服务 (与"定位+Nav"组互斥)')
        dv = QVBoxLayout(dep_box)
        self._deps = {}
        for key, (title, cmd) in DEP_CMDS.items():
            row = QHBoxLayout()
            name = QLabel(title)
            status = QLabel('离线')
            status.setFixedWidth(40)
            btn = QPushButton('启动')
            btn.setFixedWidth(56)
            btn.clicked.connect(lambda _c=False, k=key: self._toggle_dep(k))
            btn_log = QPushButton('日志')
            btn_log.setFixedWidth(48)
            row.addWidget(name, 1)
            row.addWidget(status)
            row.addWidget(btn)
            row.addWidget(btn_log)
            dv.addLayout(row)
            self._deps[key] = {'status': status, 'btn': btn,
                               'btn_log': btn_log, 'title': title}
        right.addWidget(dep_box)

        task_box = QGroupBox('任务')
        tv = QVBoxLayout(task_box)
        mrow = QHBoxLayout()
        self._btn_mission_start = QPushButton('开始任务')
        self._btn_mission_start.setStyleSheet('font-weight: bold;')
        self._btn_mission_start.clicked.connect(self._mission_start)
        btn_cancel = QPushButton('取消任务')
        btn_cancel.clicked.connect(self._mission_cancel)
        mrow.addWidget(self._btn_mission_start)
        mrow.addWidget(btn_cancel)
        self._chk_loop = QCheckBox('循环执行')
        self._chk_loop.setToolTip('勾选后: 每轮点位走完自动重发; 取消或失败自动停止')
        mrow.addWidget(self._chk_loop)
        tv.addLayout(mrow)
        mission_status = QLabel('状态: 未收到')
        mission_status.setWordWrap(True)
        tv.addWidget(mission_status)
        right.addWidget(task_box)

        # OCR 识别结果 (传统路径的 CAPTURING 触发 + 语音路径的 /ocr_feedback)
        ocr_box = QGroupBox('OCR 识别结果')
        ov = QVBoxLayout(ocr_box)
        ocr_text = QPlainTextEdit()
        ocr_text.setReadOnly(True)
        ocr_text.document().setMaximumBlockCount(200)
        ov.addWidget(ocr_text)
        right.addWidget(ocr_box, 1)
        right.addStretch(1)
        root.addLayout(right, 4)

        self._wps = []
        self._wp_file = DEFAULT_WP_FILE
        self._wp_load_file(DEFAULT_WP_FILE, quiet=True)
        return {'widget': w, 'status': mission_status,
                'btn_start': self._btn_mission_start, 'ocr': ocr_text}

    # 点位动作的中文标签 (列表回显用)
    ACTION_LABELS = {'grasp': '抓取', 'place': '放置',
                     'home2': '收臂', 'ready': '预备'}

    def _wp_refresh_list(self):
        self._wp_list.blockSignals(True)
        self._wp_list.clear()
        for i, wp in enumerate(self._wps):
            p = wp['pose']
            name = wp.get('name') or '(未命名)'
            tags = []
            if wp.get('capture'):
                tags.append('拍照')
            act = wp.get('action') or ''
            if act:
                tags.append(self.ACTION_LABELS.get(act, act))
            if int(wp.get('ptz_preset', 0) or 0) > 0:
                tags.append('预置位%d' % int(wp['ptz_preset']))
            suffix = ' [%s]' % '+'.join(tags) if tags else ''
            self._wp_list.addItem('%d. %s  (%.2f, %.2f)%s'
                                  % (i + 1, name,
                                     float(p.get('x', 0.0)),
                                     float(p.get('y', 0.0)),
                                     suffix))
        self._wp_list.blockSignals(False)

    def _wp_record(self):
        pose = self.probe.lookup_pose()
        if pose is None:
            self._sys_log('录点失败: TF map→base_link 不可用(定位在运行吗?)')
            return
        wp = {'name': '点位%d' % (len(self._wps) + 1), 'pose': pose,
              'ptz_preset': 0, 'capture': True, 'action': ''}
        self._wps.append(wp)
        self._wp_refresh_list()
        self._wp_list.setCurrentRow(len(self._wps) - 1)
        self._sys_log('已录制点位: %s (%.2f, %.2f)'
                      % (wp['name'], pose['x'], pose['y']))

    def _wp_delete(self):
        row = self._wp_list.currentRow()
        if 0 <= row < len(self._wps):
            del self._wps[row]
            self._wp_refresh_list()

    def _wp_clear(self):
        self._wps = []
        self._wp_refresh_list()

    def _wp_select(self, row):
        if not (0 <= row < len(self._wps)):
            return
        wp = self._wps[row]
        self._wp_name.setText(wp.get('name', ''))
        self._wp_ptz.setValue(int(wp.get('ptz_preset', 0)))
        self._wp_capture.setChecked(bool(wp.get('capture', False)))
        action = wp.get('action', '') or ''
        idx = self._wp_action.findData(action)
        self._wp_action.setCurrentIndex(idx if idx >= 0 else 0)

    def _wp_apply(self):
        row = self._wp_list.currentRow()
        if not (0 <= row < len(self._wps)):
            self._sys_log('请先选中一个点位')
            return
        wp = self._wps[row]
        wp['name'] = self._wp_name.text().strip()
        wp['ptz_preset'] = self._wp_ptz.value()
        wp['capture'] = self._wp_capture.isChecked()
        wp['action'] = self._wp_action.currentData() or ''
        self._wp_refresh_list()
        self._wp_list.setCurrentRow(row)
        self._sys_log('已应用修改: %s' % (wp['name'] or '点位%d' % (row + 1)))

    def _wp_load_file(self, path, quiet=False):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            wps = []
            for item in (data or {}).get('waypoints', []):
                wps.append({
                    'name': str(item.get('name', '') or ''),
                    'pose': dict(item.get('pose', {}) or {}),
                    'ptz_preset': int(item.get('ptz_preset', 0) or 0),
                    'capture': bool(item.get('capture', False)),
                    'action': str(item.get('action', '') or ''),
                })
            self._wps = wps
            self._wp_file = path
            self._wp_file_label.setText('文件: %s' % path)
            self._wp_refresh_list()
            if not quiet:
                self._sys_log('已加载 %d 个点位: %s' % (len(wps), path))
        except Exception as exc:
            if not quiet:
                QMessageBox.warning(self, '加载失败', '%s\n%s' % (path, exc))

    def _wp_load(self):
        path, _ = QFileDialog.getOpenFileName(
            self, '加载点位YAML', KYBOT_WS + '/location',
            'YAML (*.yaml *.yml)')
        if path:
            self._wp_load_file(path)

    def _wp_save(self):
        path, _ = QFileDialog.getSaveFileName(
            self, '保存点位YAML', self._wp_file, 'YAML (*.yaml)')
        if not path:
            return
        if not (path.endswith('.yaml') or path.endswith('.yml')):
            path += '.yaml'
        try:
            with open(path, 'w', encoding='utf-8') as f:
                yaml.safe_dump({'waypoints': self._wps}, f,
                               allow_unicode=True, default_flow_style=False)
            self._wp_file = path
            self._wp_file_label.setText('文件: %s' % path)
            self._sys_log('已保存 %d 个点位: %s (语音调度即时生效)'
                          % (len(self._wps), path))
        except Exception as exc:
            QMessageBox.warning(self, '保存失败', str(exc))

    def _to_mission_waypoint(self, wp):
        from hk_camera.msg import MissionWaypoint
        mw = MissionWaypoint()
        mw.nav_pose.header.frame_id = 'map'
        mw.nav_pose.header.stamp = self.probe.node.get_clock().now().to_msg()
        p = wp['pose']
        mw.nav_pose.pose.position.x = float(p.get('x', 0.0))
        mw.nav_pose.pose.position.y = float(p.get('y', 0.0))
        mw.nav_pose.pose.position.z = float(p.get('z', 0.0))
        mw.nav_pose.pose.orientation.x = float(p.get('qx', 0.0))
        mw.nav_pose.pose.orientation.y = float(p.get('qy', 0.0))
        mw.nav_pose.pose.orientation.z = float(p.get('qz', 0.0))
        mw.nav_pose.pose.orientation.w = float(p.get('qw', 1.0))
        mw.pan = float(wp.get('ptz_preset', 0))  # 预置位号, 同面板约定
        mw.tilt = 0.0
        mw.zoom = 0.0
        mw.do_capture = bool(wp.get('capture', False))
        mw.extra_action = wp.get('action', '') or ''
        return mw

    def _mission_start(self):
        if not self._wps:
            self._sys_log('没有点位, 无法开始任务')
            return
        if not self.probe.run_cli.wait_for_service(timeout_sec=2.0):
            self._sys_log('/mission/run 服务无应答(mission_executor 在运行吗?)')
            return
        from hk_camera.srv import RunMission
        req = RunMission.Request()
        for wp in self._wps:
            req.waypoints.append(self._to_mission_waypoint(wp))
        self._sys_log('下发任务: %d 个点位...' % len(req.waypoints))
        fut = self.probe.run_cli.call_async(req)
        fut.add_done_callback(lambda f: self.sig_mission.emit(f))

    def _mission_started(self, future):
        try:
            res = future.result()
            self._mission_from_here = bool(res.accepted)
            self._sys_log('任务受理: accepted=%s %s' % (res.accepted, res.message))
        except Exception as exc:
            self._sys_log('任务下发异常: %s' % exc)

    def _mission_cancel(self):
        self._mission_from_here = False  # 取消即退出循环
        if not self.probe.cancel_cli.wait_for_service(timeout_sec=2.0):
            self._sys_log('/mission/cancel 服务无应答')
            return
        from std_srvs.srv import Trigger
        self._sys_log('请求取消任务...')
        self.probe.cancel_cli.call_async(Trigger.Request())

    def _loop_resend(self):
        """循环模式: 一轮走完后自动重发当前点位表."""
        if self._mission_from_here and self._chk_loop.isChecked():
            self._sys_log('循环模式: 自动重发任务')
            self._mission_start()

    # ---------- OCR 识别结果 (任务页显示) ----------

    def _ocr_append(self, text):
        self._mission['ocr'].appendPlainText(
            '[%s] %s' % (time.strftime('%H:%M:%S'), text))

    def _on_ocr_feedback(self, text):
        # 可能在 spin 线程被调, 经信号槽转交 GUI 线程
        self.sig_ocr.emit(text)

    def _on_capture_edge(self):
        """/mission/status 出现 CAPTURING: 巡检流程拍照了, 后台调 OCR.
        3s 去抖: 防止状态周期重发造成同一帧重复识别."""
        now = time.monotonic()
        if now - self._last_ocr_trigger < 3.0:
            return
        self._last_ocr_trigger = now
        threading.Thread(target=self._ocr_on_capture, daemon=True).start()

    def _ocr_on_capture(self):
        if self._ocr_inflight:
            return  # 上一次识别还在跑, 不并发 (ocr_node 单线程处理)
        self._ocr_inflight = True
        try:
            time.sleep(1.2)  # 等拍照动作完成、画面稳定
            try:
                if not self.probe.ocr_cli.service_is_ready():
                    self._on_ocr_feedback('[巡检拍照] OCR服务未运行')
                    return
                from ocr_interfaces.srv import RecognizeText
                req = RecognizeText.Request()
                req.conf_threshold = 0.0
                fut = self.probe.ocr_cli.call_async(req)
                fut.add_done_callback(self._ocr_done)
            except Exception:
                pass
        finally:
            self._ocr_inflight = False

    def _ocr_done(self, future):
        try:
            res = future.result()
            if not res.success:
                text = 'OCR失败: %s' % res.message
            elif not res.detections:
                text = '未识别到文字'
            else:
                items = ['%s(%.2f)' % (d.text, d.confidence)
                         for d in res.detections[:5]]
                text = '识别到 %d 处: %s' % (len(res.detections),
                                             '、'.join(items))
        except Exception as exc:
            text = 'OCR调用异常: %s' % exc
        self._on_ocr_feedback('[巡检拍照] ' + text)

    # ---------- 依赖服务 (任务调度 / 海康相机) ----------

    def _toggle_dep(self, key):
        proc = self._procs.get(key)
        if proc is not None and proc.running():
            proc.stop()
            self._sys_log('停止: %s' % self._deps[key]['title'])
        else:
            if key == 'dep_ocr' and self.probe.has_node('ocr_node'):
                self._sys_log('ocr_node 已在运行(可能由启动页启动), 不重复启动')
                return
            if self._group_running('nav'):
                self._sys_log('"定位+Nav"组在运行, 其中已含%s, 不重复启动'
                              % self._deps[key]['title'])
                return
            _title, cmd = DEP_CMDS[key]
            self._start_proc(key, self._deps[key]['title'], cmd)
            self._sys_log('启动: %s' % self._deps[key]['title'])
            if key == 'dep_camera':
                self._camera_post_start()
            if key == 'dep_ocr':
                self._ocr_post_start()

    # ---------- OCR 启动后链: 预热首次推理 (GPU 冷启动可达 20~40s) ----------

    def _ocr_post_start(self):
        threading.Thread(target=self._ocr_post_start_run, daemon=True).start()

    def _ocr_post_start_run(self):
        for _ in range(120):  # 等 OCR 服务就绪 (最多 60s)
            try:
                if self.probe.ocr_cli.service_is_ready():
                    break
            except Exception:
                pass
            time.sleep(0.5)
        else:
            self.sig_sys.emit('OCR 服务未就绪, 预热跳过')
            return
        self.sig_sys.emit('OCR 预热中(首次推理较慢, 约 20~40s)...')
        from ocr_interfaces.srv import RecognizeText
        req = RecognizeText.Request()
        req.conf_threshold = 0.0
        try:
            fut = self.probe.ocr_cli.call_async(req)
            t0 = time.monotonic()
            while not fut.done():
                if time.monotonic() - t0 > 90.0:
                    self.sig_sys.emit('OCR 预热超时')
                    return
                time.sleep(0.2)
            self.sig_sys.emit('OCR 预热完成, 可以识别')
        except Exception as exc:
            self.sig_sys.emit('OCR 预热异常: %s' % exc)

    # ---------- 相机启动后链: 登录 + 开流 (OCR/图像流前提) ----------

    def _camera_post_start(self):
        """相机节点起来后自动调 login + start_stream, 否则无图像流、OCR 无帧."""
        threading.Thread(target=self._camera_post_start_run, daemon=True).start()

    def _camera_post_start_run(self):
        from std_srvs.srv import Trigger
        # 等相机服务就绪 (最多 60s, nav 组里相机要 21s 才拉起)
        for _ in range(120):
            try:
                if self.probe.login_cli.service_is_ready():
                    break
            except Exception:
                pass
            time.sleep(0.5)
        else:
            self.sig_sys.emit('相机服务未就绪, 登录/开流跳过')
            return
        ok_login = self._call_trigger_blocking(self.probe.login_cli, '登录')
        ok_stream = self._call_trigger_blocking(self.probe.stream_cli, '开流')
        if ok_login and ok_stream:
            self.sig_sys.emit('相机: 登录+开流完成, 图像流已开启')

    def _call_trigger_blocking(self, client, what, timeout=15.0):
        from std_srvs.srv import Trigger
        try:
            fut = client.call_async(Trigger.Request())
            t0 = time.monotonic()
            while not fut.done():
                if time.monotonic() - t0 > timeout:
                    self.sig_sys.emit('相机%s: 超时' % what)
                    return False
                time.sleep(0.1)
            res = fut.result()
            self.sig_sys.emit('相机%s: %s' % (what, res.message))
            return res.success
        except Exception as exc:
            self.sig_sys.emit('相机%s异常: %s' % (what, exc))
            return False

    # ---------- 模式 ----------

    def _apply_mode(self, mode, force=False):
        if not force and (self._group_running('nav')
                          or self._group_running('arm')):
            QMessageBox.information(
                self, '模式锁定', '定位Nav 或机械臂正在运行, 请先停止再切换模式。')
            # 回退单选
            self._radio_two.blockSignals(True)
            self._radio_hand.blockSignals(True)
            (self._radio_two if self._mode == 'twofinger'
             else self._radio_hand).setChecked(True)
            self._radio_two.blockSignals(False)
            self._radio_hand.blockSignals(False)
            return
        self._mode = mode
        self._rows['arm']['title'] = '机械臂抓取(%s)' % MODES[mode]['title']
        self._rows['arm']['name'].setText(self._rows['arm']['title'])
        self._sys_log('当前模式: %s' % MODES[mode]['title'])

    # ---------- 进程组管理 ----------

    def _group_running(self, key):
        return any(p.running() for k, p in self._procs.items()
                   if k == key or k.startswith(key + '_'))

    def _toggle_group(self, key):
        if self._group_running(key):
            self._stop_group(key)
        else:
            self._start_group(key)

    def _start_group(self, key):
        if key == 'camera':
            if self._group_running('nav'):
                self._sys_log('"定位+Nav"组在运行, 其中已含海康相机, 不重复启动')
                return
            if self.probe.has_node('hk_camera_node'):
                self._sys_log('海康相机已在运行, 不重复启动')
                return
        if key == 'ocr' and self.probe.has_node('ocr_node'):
            self._sys_log('ocr_node 已在运行(可能由任务页启动), 不重复启动')
            return
        if key == 'mapping':
            busy = [k for k in MUTEX_WITH_MAPPING if self._group_running(k)]
            if busy:
                names = '、'.join(self._rows[k]['title'] for k in busy)
                ret = QMessageBox.question(
                    self, '互斥确认',
                    '建图与以下功能互斥:\n%s\n\n将先停止它们再开启建图, 继续?' % names)
                if ret != QMessageBox.Yes:
                    return
                for k in busy:
                    self._stop_group(k)
        if key == 'nav':
            # nav 组内含 mission_executor 和 hk_camera, 先停任务页的单独依赖
            for dk, (dtitle, _cmd) in DEP_CMDS.items():
                p = self._procs.get(dk)
                if p is not None and p.running():
                    self._sys_log('%s 已单独运行, 先停止(改由"定位+Nav"组统一拉起)'
                                  % dtitle)
                    p.stop()
            # 启动页单独起的相机也要停
            pc = self._procs.get('camera')
            if pc is not None and pc.running():
                self._sys_log('海康相机已单独运行, 先停止(改由"定位+Nav"组统一拉起)')
                pc.stop()
        # 先清掉手动脚本/上次残留的同名进程, 防重复节点
        for pattern in self.SWEEP_PATTERNS.get(key, []):
            self._do_sweep(pattern)
        cmds = self._group_cmds(key)
        if not cmds:
            return
        for i, spec in enumerate(cmds):
            pkey, title, cmd = spec[0], spec[1], spec[2]
            delay = spec[3] if len(spec) > 3 else 0
            if key == 'arm' and i > 0 and len(spec) <= 3:
                delay = MODES[self._mode]['arm_grasp_delay_s']
            self._start_proc(pkey, title, cmd, delay_s=delay)
        self._sys_log('启动组: %s' % self._rows[key]['title'])
        if key in ('camera', 'nav'):
            # nav 组内含相机但没人开流, 同样补上 登录+开流
            self._camera_post_start()
        if key == 'ocr':
            self._ocr_post_start()

    def _group_cmds(self, key):
        mode = MODES[self._mode]
        if key == 'nav':
            return [('nav', '定位Nav', mode['nav_cmd'])]
        if key == 'arm':
            return mode['arm_cmds']
        if key == 'mapping':
            return MAPPING_CMDS
        title, cmd = FIXED_CMDS[key]
        return [(key, title, cmd)]

    def _start_proc(self, pkey, title, cmd, delay_s=0):
        if pkey in self._procs and self._procs[pkey].running():
            return
        if pkey not in self._logs:
            tab = self._add_log_tab(pkey, title)
            row_key = pkey.split('_')[0]
            if row_key in self._rows:
                self._rows[row_key]['btn_log'].clicked.connect(
                    lambda: self._tabs.setCurrentWidget(tab))
            elif pkey in self._deps:
                self._deps[pkey]['btn_log'].clicked.connect(
                    lambda: self._tabs.setCurrentWidget(tab))
        log = self._logs[pkey]
        if pkey in self._procs:
            self._procs[pkey].qp.deleteLater()
        proc = Proc(pkey, title, cmd, log, self._refresh_buttons)
        self._procs[pkey] = proc
        if delay_s > 0:
            self._sys_log('%s 延迟 %ds 启动...' % (title, delay_s))
            QTimer.singleShot(delay_s * 1000, proc.start)
        else:
            proc.start()
        self._refresh_buttons()

    def _stop_group(self, key):
        for k, p in self._procs.items():
            if (k == key or k.startswith(key + '_')) and p.running():
                p.stop()
        self._sys_log('停止组: %s' % self._rows[key]['title'])
        self._sweep_group(key)

    # 脱离进程组的"弹窗进程"特征 (gnome-terminal 由系统服务代管,
    # 不在 launch 的进程组里, killpg 够不到, 按命令行特征精准清理)
    SWEEP_PATTERNS = {
        'arm': [r'yolo_grasp[.]py'],
        'nav': [r'wit_ros2_im[u]',           # IMU 终端窗
                r'velodyne_localizatio[n]',  # FAST_LIO 定位终端窗
                r'mission_executor[.]launch',  # mission_executor 终端窗
                r'nav2_bringu[p]'],          # Nav2 终端窗
        'mapping': [r'wit_ros2_im[u]',       # 建图脚本的 IMU 窗
                    r'rslidar_sd[k]',        # 建图脚本的雷达窗
                    r'rs_to_velodyn[e]',     # 建图脚本的转换窗
                    r'mapping[.]launch'],    # 建图脚本的 FAST_LIO2 窗
    }

    def _sweep_group(self, key):
        for pattern in self.SWEEP_PATTERNS.get(key, []):
            # 延时 1s 执行, 让 killpg 先生效, 这里只清漏网的
            QTimer.singleShot(1000, lambda p=pattern: self._do_sweep(p))

    def _do_sweep(self, pattern):
        # 方括号写法: 自身命令行不会被正则匹配到
        r = subprocess.run(['pgrep', '-f', pattern],
                           capture_output=True, text=True)
        pids = [int(p) for p in r.stdout.split()
                if p.strip() and int(p) != os.getpid()]
        killed = []
        for pid in pids:
            try:
                if os.getpgid(pid) == pid:
                    # 窗口 shell 是会话 leader: 杀整组(连带到其子进程树)
                    os.killpg(pid, signal.SIGTERM)
                    killed.append('%d(组)' % pid)
                else:
                    os.kill(pid, signal.SIGTERM)
                    killed.append(str(pid))
            except (ProcessLookupError, PermissionError, OSError):
                pass
        if killed:
            self._sys_log('清理残留进程(%s): %s' % (pattern, ' '.join(killed)))

    def _stop_all(self):
        self._seq_steps = []
        for p in self._procs.values():
            if p.running():
                p.stop()
        for key in self.SWEEP_PATTERNS:
            self._sweep_group(key)
        self._status_label.setText('已全部停止')

    # ---------- CAN ----------

    def _setup_can(self):
        cmds = [
            ('can1', 'sudo -n ip link set can1 type can bitrate 500000'),
            ('can1', 'sudo -n ip link set can1 up'),
            ('can2', 'sudo -n ip link set can2 up type can bitrate 1000000'),
        ]
        for iface, cmd in cmds:
            if subprocess.call(['ip', 'link', 'show', iface],
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL) != 0:
                self._sys_log('%s 不存在, 跳过' % iface)
                continue
            r = subprocess.run(cmd.split(), capture_output=True, text=True)
            if r.returncode == 0:
                self._sys_log('%s 配置完成' % iface)
            else:
                self._sys_log('%s 配置失败: %s' % (iface, r.stderr.strip()))

    # ---------- 保存建图 ----------

    def _save_map(self):
        """一次性调用 /map_save (fast_lio2_ws 环境), 输出进系统日志."""
        self._sys_log('调用 /map_save 保存建图...')
        cmd = (SETUP_ENV + 'source %s/fast_lio2_ws/install/setup.bash && '
               'ros2 service call /map_save std_srvs/srv/Trigger "{}"' % DOC)
        qp = QProcess(self)
        qp.setProcessChannelMode(QProcess.MergedChannels)
        qp.readyReadStandardOutput.connect(
            lambda: self._sys_log(
                'map_save: ' + bytes(qp.readAllStandardOutput())
                .decode('utf-8', 'replace').strip()[:300]))
        qp.finished.connect(
            lambda: (self._sys_log('map_save 调用结束 (exit=%d)' % qp.exitCode()),
                     self._oneshots.remove(qp)))
        self._oneshots.append(qp)
        qp.start('bash', ['-c', cmd])

    # ---------- 一键启动 ----------

    def _start_all(self):
        if self._seq_steps:
            self._status_label.setText('一键启动已在进行中')
            return
        steps = [
            ('配置 CAN', self._setup_can, None, 0),
            ('启动 定位+Nav', lambda: self._start_group('nav'),
             lambda: self.probe.has_publisher('/mission/status')
             or self.probe.has_node('mission_executor'), 60),
            ('启动 机械臂抓取', lambda: self._start_group('arm'),
             lambda: self.probe.has_service('/yolo_grasp/grasp_hold'), 90),
            ('启动 语音调度', lambda: self._start_group('brain'),
             lambda: self.probe.has_node('brain_node'), 30),
            ('启动 语音前端', lambda: self._start_group('aiui'),
             lambda: self.probe.has_node('aiui_ros_node'), 30),
        ]
        if self._chk_rviz.isChecked():
            steps.append(('启动 RViz', lambda: self._start_group('rviz'),
                          None, 0))
        self._seq_steps = steps
        self._seq_ready_fn = None
        self._seq_timer.start(1000)
        self._sys_log('一键启动开始')

    def _seq_tick(self):
        if self._seq_ready_fn is not None:
            # 等待当前步骤就绪
            ok = False
            try:
                ok = self._seq_ready_fn()
            except Exception:
                ok = False
            if ok:
                self._sys_log('就绪 ✓')
                self._seq_ready_fn = None
            elif time.time() > self._seq_wait_until:
                self._sys_log('等待超时, 继续下一步(请检查该组日志)')
                self._seq_ready_fn = None
            else:
                return
        if not self._seq_steps:
            self._seq_timer.stop()
            self._status_label.setText('一键启动完成')
            self._sys_log('一键启动完成')
            return
        desc, start_fn, ready_fn, timeout = self._seq_steps.pop(0)
        self._status_label.setText('一键启动: %s ...' % desc)
        self._sys_log(desc)
        start_fn()
        if ready_fn is not None:
            self._seq_ready_fn = ready_fn
            self._seq_wait_until = time.time() + timeout

    # ---------- 状态刷新 ----------

    def _refresh_buttons(self):
        for key, row in self._rows.items():
            row['btn'].setText('停止' if self._group_running(key) else '启动')
        # 建图互斥: 建图运行时, 导航/机械臂/语音/一键启动/模式切换全部禁用
        mapping_on = self._group_running('mapping')
        for k in MUTEX_WITH_MAPPING:
            self._rows[k]['btn'].setEnabled(not mapping_on)
        self._btn_all.setEnabled(not mapping_on)
        self._radio_two.setEnabled(not mapping_on)
        self._radio_hand.setEnabled(not mapping_on)
        # 保存建图: 只有 /map_save 服务在线才可点
        try:
            self._btn_save_map.setEnabled(self.probe.has_service('/map_save'))
        except Exception:
            self._btn_save_map.setEnabled(False)

    def _refresh_status(self):
        self._refresh_buttons()
        checks = {
            'nav': lambda: self.probe.has_publisher('/mission/status')
            or self.probe.has_node('mission_executor'),
            'arm': lambda: self.probe.has_service('/yolo_grasp/grasp_hold'),
            'brain': lambda: self.probe.has_node('brain_node'),
            'aiui': lambda: self.probe.has_node('aiui_ros_node'),
            'camera': lambda: self.probe.has_node('hk_camera_node'),
            'ocr': lambda: self.probe.has_service('/ocr/recognize')
            or self.probe.has_node('ocr_node'),
            'rviz': lambda: self.probe.has_node('rviz'),
            'mapping': lambda: self.probe.has_service('/map_save'),
        }
        for key, check in checks.items():
            if self._group_running(key):
                state = 'starting'
                try:
                    if check():
                        state = 'online'
                except Exception:
                    pass
            else:
                # 进程没起但在线 (可能是别处启动的), 也显示在线
                try:
                    state = 'online' if check() else 'off'
                except Exception:
                    state = 'off'
            self._set_led(self._rows[key]['led'], state)

        # 任务点位页: 状态文本 + 开始按钮可用性
        s = self.probe.mission_status
        if s is None:
            txt = '状态: 未收到(mission_executor 未运行?)'
        else:
            txt = '状态: %s %d/%d %s' % (
                MISSION_STATE_NAMES.get(s.state, str(s.state)),
                s.current_index + 1, s.total_count, s.message)
        self._mission['status'].setText(txt)
        online = (self.probe.has_publisher('/mission/status')
                  or self.probe.has_node('mission_executor'))
        busy = s is not None and s.state not in MISSION_FREE_STATES
        self._mission['btn_start'].setEnabled(online and not busy)

        # 循环重发: 一轮 COMPLETED 边沿 → 5s 后自动重发; 失败/取消退出循环
        prev_state = self._prev_ms_state
        self._prev_ms_state = s.state if s is not None else None
        if s is not None and prev_state != s.state and self._mission_from_here:
            if s.state == 4 and self._chk_loop.isChecked():  # COMPLETED
                self._sys_log('一轮任务完成, 循环模式: 5s 后自动重发')
                QTimer.singleShot(5000, self._loop_resend)
            elif s.state == 5:  # FAILED
                self._mission_from_here = False
                self._sys_log('任务失败, 循环停止')
            elif s.state == 6:  # CANCELED
                self._mission_from_here = False

        # 任务页依赖服务行: 状态 + 按钮
        nav_on = self._group_running('nav')
        for key, dep in self._deps.items():
            proc = self._procs.get(key)
            running = proc is not None and proc.running()
            online = any(self.probe.has_node(n)
                         for n in DEP_NODE_CHECKS.get(key, ()))
            dep['status'].setText(
                '在线' if online else ('启动中' if running else '离线'))
            dep['btn'].setText('停止' if running else '启动')
            dep['btn'].setEnabled(running or not nav_on)

    # ---------- 关闭 ----------

    def closeEvent(self, event):
        running = [p.title for p in self._procs.values() if p.running()]
        if running:
            ret = QMessageBox.question(
                self, '退出确认',
                '以下进程仍在运行:\n%s\n\n退出并全部停止?' % '、'.join(running))
            if ret != QMessageBox.Yes:
                event.ignore()
                return
            self._stop_all()
        self.probe.shutdown()
        event.accept()


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
