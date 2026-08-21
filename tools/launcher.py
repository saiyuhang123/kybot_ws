#!/usr/bin/env python3
"""KYBOT 一键启动面板 (PyQt5, 单文件, 双击可运行)

分组管理: 定位Nav / 机械臂(二指|柔触|灵巧手|打磨头, 单选互斥) / 语音调度 / 语音前端 / RViz
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

# ---------- 定位/导航底座 (与末端无关, 换末端时不重启, 定位不丢) ----------
# 这一组只含底盘/雷达/IMU/EKF/FAST_LIO/Nav2/相机/mission_executor, 没有任何
# 末端专属内容。end_effector_mode 只是 mission_executor 的初始值, 运行中由
# 面板通过 /mission_executor/set_parameters 热切换, 因此换末端无需重启本组。
NAV_CMD = (KYBOT_SETUP + 'ros2 launch kybot_bringup bringup.launch.py '
           'use_ocr:=false end_effector_mode:=%s')
# 路边捡瓶感知 (D435 + 前方 YOLO -> /trash/target): 只有二指参与。
# 柔触和灵巧手只做到点抓取, 没有路边识别环节; 打磨头与瓶子无关。
# 原先由 trash_pipeline 内嵌, 现在拆成独立组, 才能按末端单独起停。
PERCEP_CMD = KYBOT_SETUP + 'ros2 launch trash_mission trash_mission.launch.py'
NEEDS_PERCEP = {'twofinger'}
# 首次随 nav 启动时的错峰秒数, 照搬 trash_pipeline 原本的 TimerAction:
# "D435 + 感知在底盘起来后启动, 避免开机瞬间 CPU 过载", RViz 最后起。
# D435 initial_reset + YOLO-World 加载若与 FAST_LIO/Nav2 初始化同时进行,
# 在 Jetson 上会正面抢 CPU。热换末端时 nav 已稳定, 不需要这个延时。
PERCEP_START_DELAY_S = 12
RVIZ_START_DELAY_S = 16
# 原 trash_pipeline (二指/柔触) 带 use_rviz:=true 会自动拉起 MyPanel 版 RViz,
# 灵巧手/打磨走 bringup 时没有。这里与"路边捡瓶开放范围"解耦, 各自保持原习惯:
# 柔触不再捡瓶, 但 RViz 该有还是有。
NAV_AUTO_RVIZ = {'twofinger', 'softtouch'}
# 可在不停定位导航的前提下互换的末端。打磨头用的是另一套机械臂驱动
# (start_robot.launch.py + 力控 + 深度相机), 不参与热换, 仍需整组停启。
HOT_SWAPPABLE = {'twofinger', 'softtouch', 'linkerhand'}
MISSION_EXECUTOR_NODE = 'mission_executor'
# 旧末端软件栈是否彻底退场的判据。只用节点名: 服务/话题注册在进程被杀后会
# 残留一段时间, 用服务名判"还在"会误判, 白等到超时。三种抓取末端共用
# /yolo_grasp/*, 靠服务名本来也区分不了, 只能确认这些节点"全没了"。
STALE_STACK_NODES = ('robot_cartesian_control', 'gripper_server',
                     'ysURForceAppControl')
RVIZ_CFG = '/home/nvidia/.rviz2/default.rviz'

# ---------- 四种互斥末端的 launch 配对 ----------
MODES = {
    'twofinger': {
        'title': '二指',
        'arm_cmds': [
            ('arm_main', '机械臂驱动(二指)',
             ELITE_SETUP + 'ros2 launch %s/biaoding/yolo_grasp_two_finger.launch.py '
             'run_grasp_main:=false' % ELITE_WS),
            ('arm_grasp', '抓取服务(二指)',
             ELITE_SETUP + 'cd %s/biaoding && python3 yolo_grasp.py '
             '--gripper two_finger --target-class bottle ' % ELITE_WS),
        ],
        'arm_grasp_delay_s': 10,  # 主 launch 起完再起 headless 抓取服务
    },
    'softtouch': {
        'title': '柔触手抓',
        # 柔触只做到点抓取 (与灵巧手一致), 不参与路边识别捡瓶。
        # 但前向停车距离仍用柔触专用的 0.50m (末端更长), 见 mission_executor。
        'arm_cmds': [
            ('arm_main', '机械臂驱动(柔触)',
             ELITE_SETUP + 'ros2 launch %s/biaoding/yolo_grasp_soft_touch.launch.py '
             'run_grasp_main:=true grasp_headless:=false' % ELITE_WS),
        ],
        # 抓取主程序由柔触 launch 在 T+9s 打开独立交互终端（保留键盘调试）。
        'arm_grasp_delay_s': 0,
    },
    'linkerhand': {
        'title': '灵巧手',
        'arm_cmds': [
            ('arm_main', '机械臂抓取(灵巧手)',
             ELITE_SETUP + 'ros2 launch %s/biaoding/yolo_grasp.launch.py' % ELITE_WS),
        ],
        'arm_grasp_delay_s': 0,
    },
    'polish': {
        'title': '打磨头',
        'arm_cmds': [
            # 正式调度由启动面板下发命令：驱动使用 headless，不另起 RViz。
            ('arm_driver', '机械臂驱动(打磨)',
             ELITE_SETUP + 'ros2 launch my_elite_robot_cell_control '
             'start_robot.launch.py headless_mode:=true launch_rviz:=false', 0),
            ('arm_depth', '深度相机(打磨)',
             ELITE_SETUP + 'ros2 launch percipio_camera '
             'percipio_camera.launch.py', 6),
            # 保留 elite_polish.launch.py 自带的交互命令终端，便于随时调试。
            ('arm_polish', '打磨状态机',
             ELITE_SETUP + 'ros2 launch elite_polish_app '
             'elite_polish.launch.py', 10),
            ('arm_bridge', '打磨调度桥接',
             KYBOT_SETUP + 'ros2 run kybot_brain polish_bridge', 13),
        ],
        'arm_grasp_delay_s': 0,
    },
}

MODE_ACTIONS = {
    'twofinger': {'', 'grasp', 'place', 'home2', 'ready'},
    'softtouch': {'', 'grasp', 'place', 'home2', 'ready'},
    'linkerhand': {'', 'grasp', 'place', 'home2', 'ready'},
    'polish': {'', 'polish'},
}
ACTION_OPTIONS = [
    ('无', ''), ('抓取 (grasp)', 'grasp'), ('放置 (place)', 'place'),
    ('收臂 (home2)', 'home2'), ('预备 (ready)', 'ready'),
    ('打磨：命令3视觉全流程 (polish)', 'polish'),
]

# 其他固定组
FIXED_CMDS = {
    'brain': ('语音调度', KYBOT_SETUP + 'ros2 launch kybot_brain kybot_brain.launch.py'),
    'aiui': ('语音前端', KYBOT_SETUP + 'ros2 launch robot_aiui robot_aiui.launch.py'),
    'camera': ('海康相机', KYBOT_SETUP + 'ros2 run hk_camera hk_camera_node'),
    'ocr': ('OCR识别', KYBOT_SETUP + 'ros2 launch ocr_node ocr_node.launch.py'),
    # nav 组从 trash_pipeline 换成 bringup 后不再自带 MyPanel 版 RViz,
    # 这里补上同一份配置, 保证 RViz 面板功能不丢 (配置缺失时退回裸 rviz2)。
    'rviz': ('RViz', KYBOT_SETUP + 'if [ -f %s ]; then exec rviz2 -d %s; '
             'else exec rviz2; fi' % (RVIZ_CFG, RVIZ_CFG)),
}

# 建图组 (对应 ~/Documents/start_fastlioMapping.sh 的四个组件,
# 直接以子进程方式管理, 不弹 gnome-terminal, killpg 即可全停;
# delay 沿用原脚本的错峰秒数)
DOC = '/home/nvidia/Documents'
MAPPING_CMDS = [
    ('mapping_imu', '建图-IMU',
     SETUP_ENV + 'source %s/wit_ros2_imu_src/install/setup.bash && '
     'ros2 run wit_ros2_imu wit_ros2_imu --ros-args '
     '-p port:=/dev/ttyIMU -p baudrate:=921600' % DOC, 0),
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
MUTEX_WITH_MAPPING = ('nav', 'arm', 'brain', 'aiui', 'percep')

# 任务点位页
MISSION_STATE_NAMES = {
    0: '空闲', 1: '导航中', 2: '云台运动中', 3: '拍照中', 4: '已完成',
    5: '失败', 6: '已取消', 7: '目标确认中', 8: '逼近目标中', 9: '抓取中',
    10: '放置中', 11: '退回中', 12: '打磨中',
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
        self.polish_status = ''      # 最近一次 /elite_polish/status
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
            from std_msgs.msg import String
            from hk_camera.msg import MissionStatus
            from hk_camera.srv import RunMission
            self.run_cli = self.node.create_client(RunMission, '/mission/run')
            self.cancel_cli = self.node.create_client(Trigger, '/mission/cancel')
            self.polish_cancel_cli = self.node.create_client(
                Trigger, '/elite_polish/cancel')
            self.node.create_subscription(
                MissionStatus, '/mission/status', self._on_mission_status, 10)
            self.node.create_subscription(
                String, '/elite_polish/status', self._on_polish_status, 10)
            self.login_cli = self.node.create_client(Trigger, '/hk_camera/login')
            self.stream_cli = self.node.create_client(Trigger,
                                                      '/hk_camera/start_stream')
            self.stop_stream_cli = self.node.create_client(
                Trigger, '/hk_camera/stop_stream')
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
        try:
            from rcl_interfaces.srv import GetParameters, SetParameters
            self.set_param_cli = self.node.create_client(
                SetParameters, '/%s/set_parameters' % MISSION_EXECUTOR_NODE)
            self.get_param_cli = self.node.create_client(
                GetParameters, '/%s/get_parameters' % MISSION_EXECUTOR_NODE)
        except Exception as exc:
            self.set_param_cli = None
            self.get_param_cli = None
            self._report_error('参数接口不可用(末端热切换降级): %s' % exc)

    def _wait_future(self, fut, timeout):
        """在非 spin 线程里等服务返回 (探针自带 spin 线程负责推进)."""
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                return None
            time.sleep(0.05)
        return fut.result()

    def set_end_effector_mode(self, mode, timeout=8.0):
        """热切换 mission_executor 的末端模式, 返回 (ok, reason)."""
        cli = getattr(self, 'set_param_cli', None)
        if cli is None:
            return False, '参数接口不可用'
        try:
            from rcl_interfaces.msg import (Parameter, ParameterType,
                                            ParameterValue)
            from rcl_interfaces.srv import SetParameters
            if not cli.wait_for_service(timeout_sec=timeout):
                return False, '/%s/set_parameters 未就绪' % MISSION_EXECUTOR_NODE
            req = SetParameters.Request()
            value = ParameterValue()
            value.type = ParameterType.PARAMETER_STRING
            value.string_value = mode
            param = Parameter()
            param.name = 'end_effector_mode'
            param.value = value
            req.parameters = [param]
            res = self._wait_future(cli.call_async(req), timeout)
            if res is None:
                return False, '下发超时'
            if not res.results:
                return False, '节点无返回'
            if not res.results[0].successful:
                return False, res.results[0].reason or '节点拒绝'
            return True, ''
        except Exception as exc:
            return False, '下发异常: %s' % exc

    def get_end_effector_mode(self, timeout=4.0):
        """回读实际生效的末端模式; 节点不在或失败返回 None."""
        cli = getattr(self, 'get_param_cli', None)
        if cli is None:
            return None
        try:
            from rcl_interfaces.srv import GetParameters
            if not cli.service_is_ready():
                return None
            req = GetParameters.Request()
            req.names = ['end_effector_mode']
            res = self._wait_future(cli.call_async(req), timeout)
            if res is None or not res.values:
                return None
            val = res.values[0]
            # 4 == PARAMETER_STRING
            return val.string_value if val.type == 4 else None
        except Exception:
            return None

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

    def _on_polish_status(self, msg):
        self.polish_status = msg.data.strip()

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
        self._start_timer = None

    def active(self):
        return self.running() or (self._start_timer is not None
                                  and self._start_timer.isActive())

    def start_delayed(self, delay_ms):
        if self.active():
            return
        self._start_timer = QTimer()
        self._start_timer.setSingleShot(True)
        self._start_timer.timeout.connect(self.start)
        self._start_timer.start(delay_ms)

    def start(self):
        if self.running():
            return
        if self._start_timer is not None:
            self._start_timer.stop()
            self._start_timer.deleteLater()
            self._start_timer = None
        self._append('—— 启动: %s ——' % self.cmd.split('&&')[-1].strip()[:120])
        self.qp.start('setsid', ['bash', '-c', self.cmd])
        self._on_state_change()

    def running(self):
        return self.qp.state() != QProcess.NotRunning

    def stop(self):
        if self._start_timer is not None and self._start_timer.isActive():
            self._start_timer.stop()
            self._start_timer.deleteLater()
            self._start_timer = None
            self._append('—— 已取消延迟启动 ——')
            self._on_state_change()
            return
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
        self.resize(1040, 660)
        self._procs = {}          # key -> Proc
        self._mode = None         # 安全默认：必须由操作员确认实际安装末端
        self._seq_steps = []      # 一键启动的待执行步骤
        self._seq_wait_until = 0.0
        self._seq_ready_fn = None
        self._oneshots = []       # 一次性命令的 QProcess (防 GC)
        self._last_ocr_trigger = 0.0  # OCR 触发去抖 (monotonic)
        self._ocr_inflight = False    # OCR 调用在飞标志 (防并发)
        self._prev_ms_state = None    # 任务状态边沿检测 (循环重发用)
        self._mission_from_here = False  # 当前任务是否由本页发起 (循环重发用)
        self._pending_safe_stops = set()  # 等打磨安全收尾后再停止的进程组
        self._safe_stop_deadline = 0.0
        self._seq_title = '一键启动'   # 顺序执行器用途名 (一键启动/热换末端共用)
        self._seq_fail_fn = None       # 某步超时时的回滚钩子
        self._seq_done_fn = None       # 全部步骤成功后的收尾钩子
        self._swap_active = False      # 热换末端进行中: 锁模式选择与启动按钮
        self._swap_prev_mode = None    # 热换失败时回滚用
        self._swap_restore_brain = False  # 热换前语音调度是否在跑
        self._live_mode = None         # mission_executor 实际生效的末端
        self._live_mode_inflight = False
        self._live_mode_next = 0.0
        # 图像流状态。拍照/云台只需登录, 不需要流; 开流会跑 1080p 软解吃 CPU
        self._stream_on = False

        self.probe = RosProbe()
        self._build_ui()
        # 第一次 2s 状态刷新前也必须立即落实“未选择末端不可启动”。
        self._refresh_buttons()
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
        self._radio_soft = QRadioButton('柔触手抓')
        self._radio_hand = QRadioButton('灵巧手')
        self._radio_polish = QRadioButton('打磨头')
        self._mode_group = QButtonGroup(self)
        self._mode_group.addButton(self._radio_two)
        self._mode_group.addButton(self._radio_soft)
        self._mode_group.addButton(self._radio_hand)
        self._mode_group.addButton(self._radio_polish)
        mh.addWidget(self._radio_two)
        mh.addWidget(self._radio_soft)
        mh.addWidget(self._radio_hand)
        mh.addWidget(self._radio_polish)
        left.addWidget(mode_box)
        # 实际生效末端: 从活着的 mission_executor 回读, 防止"面板显示"与
        # "节点实际参数"静默不一致 (外部起的 nav、面板重开都可能造成)
        self._mode_live_label = QLabel('实际生效末端: —')
        self._mode_live_label.setWordWrap(True)
        left.addWidget(self._mode_live_label)
        self._radio_two.toggled.connect(
            lambda c: c and self._apply_mode('twofinger'))
        self._radio_soft.toggled.connect(
            lambda c: c and self._apply_mode('softtouch'))
        self._radio_hand.toggled.connect(
            lambda c: c and self._apply_mode('linkerhand'))
        self._radio_polish.toggled.connect(
            lambda c: c and self._apply_mode('polish'))

        # 分组行
        self._rows = {}
        groups_box = QGroupBox('功能组')
        gv = QVBoxLayout(groups_box)
        for key, title in [('nav', '定位 + Nav'),
                           ('percep', '路边捡瓶感知 (D435, 仅二指)'),
                           ('arm', '机械臂(未选择末端)'),
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
        # 海康相机行加"开流/停流": 拍照只需登录, 开流才跑 1080p 软解
        self._btn_stream = QPushButton('开流')
        self._btn_stream.setFixedWidth(48)
        self._btn_stream.setToolTip(
            '图像流仅 OCR / 看实时画面需要。\n'
            '拍照与云台只需登录, 不需要开流。\n'
            '开流会在本机跑 1080p H.264 软解, 约占 1~2 个 CPU 核。')
        self._btn_stream.clicked.connect(self._toggle_camera_stream)
        ch = self._rows['camera']['btn_log'].parent().layout()
        ch.addWidget(self._btn_stream)
        left.addWidget(groups_box)

        # 全局按钮
        self._btn_can = QPushButton('配置 CAN (can1/can2)')
        self._btn_can.clicked.connect(self._setup_can)
        left.addWidget(self._btn_can)

        self._chk_rviz = QCheckBox('一键启动时包含 RViz')
        left.addWidget(self._chk_rviz)

        # 默认不勾: 日常任务只拍照, 不需要图像流。勾上才在相机起来后自动开流。
        self._chk_stream = QCheckBox('相机启动时自动开图像流 (仅 OCR 需要)')
        self._chk_stream.setChecked(False)
        self._chk_stream.setToolTip(
            '拍照(/hk_camera/capture)由海康 SDK 在相机端出 JPEG, 不需要图像流。\n'
            '开流会在 Jetson 上跑 1080p H.264 软解 (avdec_h264), 约占 1~2 个核。\n'
            'OCR 只能识别图像流里的最新帧, 所以 OCR 启动时会自动开流。')
        left.addWidget(self._chk_stream)

        self._btn_all = QPushButton('一键全部启动')
        self._btn_all.setObjectName('primary')
        self._btn_all.setMinimumHeight(38)
        self._btn_all.clicked.connect(self._start_all)
        left.addWidget(self._btn_all)

        self._btn_stop_all = QPushButton('全部停止')
        self._btn_stop_all.setObjectName('danger')
        self._btn_stop_all.clicked.connect(self._stop_all)
        left.addWidget(self._btn_stop_all)

        self._status_label = QLabel('就绪')
        self._status_label.setWordWrap(True)
        left.addWidget(self._status_label)
        left.addStretch(1)

        left_widget = QWidget()
        left_widget.setLayout(left)
        left_widget.setFixedWidth(358)
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
        for label, val in ACTION_OPTIONS:
            self._wp_action.addItem(label, val)
        self._wp_action.setEnabled(False)
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
        self._btn_mission_start.setObjectName('primary')
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
                     'home2': '收臂', 'ready': '预备', 'polish': '打磨'}

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
        if not self._require_mode('开始任务'):
            return
        if not self._wps:
            self._sys_log('没有点位, 无法开始任务')
            return
        incompatible = [
            '%d:%s' % (i + 1, wp.get('action', ''))
            for i, wp in enumerate(self._wps)
            if (wp.get('action', '') or '') not in MODE_ACTIONS[self._mode]
        ]
        if incompatible:
            self._sys_log(
                '拒绝任务: 点位动作与%s不兼容: %s'
                % (MODES[self._mode]['title'], '、'.join(incompatible)))
            QMessageBox.warning(
                self, '末端动作不匹配',
                '以下点位动作与当前安装的%s不兼容：\n%s'
                % (MODES[self._mode]['title'], '、'.join(incompatible)))
            return
        if not self.probe.run_cli.wait_for_service(timeout_sec=5.0):
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
        if not self.probe.cancel_cli.wait_for_service(timeout_sec=5.0):
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
                if not self._stream_on:
                    # ocr_node 只认图像流里的最新帧, 没流必然识别不出来。
                    # 明确报出来, 免得看成"OCR 识别不准"。
                    self._on_ocr_feedback(
                        '[巡检拍照] 图像流未开启, OCR 无帧可识别; '
                        '请点「海康相机」行的"开流"或勾选自动开流')
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
            if key == 'dep_exec' and not self._require_mode('启动任务调度'):
                return
            if key == 'dep_ocr' and self.probe.has_node('ocr_node'):
                self._sys_log('ocr_node 已在运行(可能由启动页启动), 不重复启动')
                return
            if self._group_running('nav'):
                self._sys_log('"定位+Nav"组在运行, 其中已含%s, 不重复启动'
                              % self._deps[key]['title'])
                return
            _title, cmd = DEP_CMDS[key]
            if key == 'dep_exec':
                cmd += ' end_effector_mode:=%s' % self._mode
            self._start_proc(key, self._deps[key]['title'], cmd)
            self._sys_log('启动: %s' % self._deps[key]['title'])
            if key == 'dep_camera':
                self._camera_post_start()
            if key == 'dep_ocr':
                self._ensure_camera_stream('OCR')
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
        # ocr_node 无帧时 /ocr/recognize 直接返回失败, 模型懒加载不会被触发,
        # 预热就白做了。所以必须先等图像流真的开起来再预热。
        for _ in range(120):  # 最多再等 60s
            if self._stream_on:
                break
            time.sleep(0.5)
        else:
            self.sig_sys.emit('图像流未开启, OCR 预热跳过 '
                              '(开流后首次识别会有 20~40s 冷启动)')
            return
        time.sleep(2.0)  # 等第一帧真正到达 ocr_node
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
            res = fut.result()
            if res.success:
                self.sig_sys.emit('OCR 预热完成, 可以识别')
            else:
                self.sig_sys.emit('OCR 预热未成功: %s' % res.message)
        except Exception as exc:
            self.sig_sys.emit('OCR 预热异常: %s' % exc)

    # ---------- 相机启动后链: 登录(总是) + 开流(按需) ----------

    def _ocr_active(self):
        """OCR 是否在跑。ocr_node 只认 /hk_camera/image_raw 里的最新帧
        (见 ocr_node.py 的 /ocr/recognize 实现, 请求里不带图), 所以
        OCR 一旦运行就必须开流, 否则识别永远返回"无最新帧"。"""
        p = self._procs.get('dep_ocr')
        return (self._group_running('ocr')
                or (p is not None and p.running())
                or self.probe.has_node('ocr_node'))

    def _camera_post_start(self, want_stream=None):
        """相机节点起来后自动登录; 图像流按需开。

        关键: 拍照(/hk_camera/capture)走海康 SDK 在相机端出 JPEG, 只需要
        login, 不需要 start_stream。而开流会在本机跑 1080p H.264 软解
        (hk_decoder 的 avdec_h264) + 每帧多次全帧拷贝, 约吃 1~2 个 CPU 核。
        所以默认不开流, 只有 OCR 在跑或操作员勾选"图像流"时才开。
        """
        if want_stream is None:
            # 在 GUI 线程判定, 不把 Qt 控件带进后台线程
            want_stream = (self._chk_stream.isChecked() or self._ocr_active())
        threading.Thread(target=self._camera_post_start_run,
                         args=(bool(want_stream),), daemon=True).start()

    def _camera_post_start_run(self, want_stream):
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
        if not ok_login:
            return
        if not want_stream:
            self.sig_sys.emit('相机: 已登录(可拍照/云台)。未开图像流 —— '
                              '拍照不需要流; 需要 OCR 或看实时画面时再开流')
            return
        ok_stream = self._call_trigger_blocking(self.probe.stream_cli, '开流')
        if ok_stream:
            self._stream_on = True
            self.sig_sys.emit('相机: 登录+开流完成, 图像流已开启 '
                              '(1080p 软解持续占 CPU, 用完请停流)')

    def _ensure_camera_stream(self, reason):
        """OCR 之类必须吃图像流的功能启动前调用: 补登录 + 开流."""
        if self._stream_on:
            return
        self._sys_log('%s 需要图像流, 自动开流' % reason)
        self._camera_post_start(want_stream=True)

    def _toggle_camera_stream(self):
        """手动开/停图像流。停流后拍照、云台、巡检全部照常可用。"""
        if not self._stream_on:
            self._camera_post_start(want_stream=True)
            return
        if self._ocr_active():
            ret = QMessageBox.question(
                self, '确认停流',
                'OCR 正在运行，它只能识别图像流里的最新帧。\n'
                '停流后 OCR 会返回"无最新帧"。仍要停流?')
            if ret != QMessageBox.Yes:
                return
        threading.Thread(target=self._camera_stop_stream_run,
                         daemon=True).start()

    def _camera_stop_stream_run(self):
        if self._call_trigger_blocking(self.probe.stop_stream_cli, '停流'):
            self._stream_on = False
            self.sig_sys.emit('相机: 已停流, CPU 软解已释放 '
                              '(拍照/云台不受影响)')

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

    def _require_mode(self, action):
        if self._mode in MODES:
            return True
        text = '请先人工确认并选择实际安装的末端（二指、柔触手抓、灵巧手或打磨头）。'
        self._sys_log('%s被阻止: 未选择末端' % action)
        QMessageBox.warning(self, '未选择末端', text)
        return False

    def _sync_mode_radios(self):
        """恢复单选状态；允许安全初始态一个都不选。"""
        radios = {
            'twofinger': self._radio_two,
            'softtouch': self._radio_soft,
            'linkerhand': self._radio_hand,
            'polish': self._radio_polish,
        }
        self._mode_group.setExclusive(False)
        for key, radio in radios.items():
            radio.blockSignals(True)
            radio.setChecked(key == self._mode)
            radio.blockSignals(False)
        self._mode_group.setExclusive(True)

    def _refresh_action_choices(self):
        selected = self._wp_action.currentData()
        allowed = MODE_ACTIONS.get(self._mode, {''})
        self._wp_action.blockSignals(True)
        self._wp_action.clear()
        for label, value in ACTION_OPTIONS:
            if value in allowed:
                self._wp_action.addItem(label, value)
        idx = self._wp_action.findData(selected)
        self._wp_action.setCurrentIndex(idx if idx >= 0 else 0)
        self._wp_action.setEnabled(self._mode in MODES)
        self._wp_action.blockSignals(False)
        self._wp_select(self._wp_list.currentRow())

    def _mission_busy(self):
        return (self.probe.has_node(MISSION_EXECUTOR_NODE)
                and self.probe.mission_status is not None
                and self.probe.mission_status.state not in MISSION_FREE_STATES)

    def _stale_end_effector_stack(self):
        """仍在线的末端软件栈节点; 空列表 = 旧末端已彻底退场, 可以起新末端。

        故意只查节点名: 服务名在进程被杀后会残留, 用它判会一直等到超时。
        """
        return [n for n in STALE_STACK_NODES if self.probe.has_node(n)]

    def _apply_mode(self, mode, force=False):
        if mode == self._mode:
            return
        if self._swap_active:
            self._sys_log('热换末端进行中, 忽略模式切换')
            self._sync_mode_radios()
            return
        mission_busy = self._mission_busy()
        stack_on = (self._group_running('nav') or self._group_running('arm')
                    or self._group_running('brain'))
        if not force and stack_on and not mission_busy \
                and not self._group_running('mapping') \
                and self._mode in HOT_SWAPPABLE and mode in HOT_SWAPPABLE:
            # 抓取家族内互换: 定位/Nav/EKF/FAST_LIO 全程不停
            self._hot_swap_mode(mode)
            return
        if not force and (stack_on or mission_busy):
            if mission_busy:
                text = '任务正在执行，请先取消任务再切换末端。'
            elif self._group_running('mapping'):
                text = '建图正在运行，请先停止建图再切换末端。'
            else:
                text = ('打磨头与抓取末端之间不支持热切换（机械臂驱动不同），\n'
                        '请先停止 定位Nav / 机械臂 / 语音调度 再切换。\n\n'
                        '二指、柔触手抓、灵巧手三者之间可以不停定位导航直接热换。')
            QMessageBox.information(self, '模式锁定', text)
            self._sync_mode_radios()
            return
        self._set_mode_now(mode)

    def _set_mode_now(self, mode):
        self._mode = mode
        self._sync_mode_radios()
        self._rows['arm']['title'] = '机械臂(%s)' % MODES[mode]['title']
        self._rows['arm']['name'].setText(self._rows['arm']['title'])
        self._refresh_action_choices()
        self._refresh_buttons()
        self._sys_log('当前模式: %s' % MODES[mode]['title'])

    # ---------- 热换末端 (不停定位导航) ----------

    def _hot_swap_mode(self, new_mode):
        """在保持定位/Nav 运行的前提下切换末端。

        时序: 停旧末端软件栈 -> 等其彻底退场 -> 热改 mission_executor 参数
              -> 回读校验 -> 按家族起停车前感知 -> 恢复语音调度。
        新末端的机械臂软件栈不自动启动: 必须等人物理换完末端再手动点启动。
        """
        old_mode = self._mode
        if self._seq_busy():
            QMessageBox.information(self, '忙', '一键启动/上一次切换尚未结束。')
            self._sync_mode_radios()
            return
        if not self.probe.has_node(MISSION_EXECUTOR_NODE):
            QMessageBox.warning(
                self, '无法热切换',
                '未发现运行中的 mission_executor，无法热改末端参数。\n'
                '请先停止相关组，用常规方式切换。')
            self._sync_mode_radios()
            return
        old_title = MODES[old_mode]['title']
        new_title = MODES[new_mode]['title']
        ret = QMessageBox.question(
            self, '热换末端确认',
            '将把末端从「%s」切换为「%s」。\n\n'
            '• 定位、Nav2、EKF、FAST_LIO 全程不停，定位不会丢\n'
            '• 会停止：机械臂软件栈、语音调度\n'
            '• 切换完成后请先物理更换末端，再手动启动机械臂\n\n继续？'
            % (old_title, new_title))
        if ret != QMessageBox.Yes:
            self._sync_mode_radios()
            return

        self._swap_active = True
        self._swap_prev_mode = old_mode
        self._swap_restore_brain = self._group_running('brain')
        # 乐观切换: 面板先显示新末端, 任何一步失败都回滚到旧末端
        self._set_mode_now(new_mode)
        self._sys_log('热换末端: %s -> %s (定位/Nav 保持运行)'
                      % (old_title, new_title))

        want_percep = new_mode in NEEDS_PERCEP
        self._seq_title = '热换末端'
        self._seq_fail_fn = self._hot_swap_failed
        self._seq_steps = [
            ('停止旧末端软件栈', self._swap_stop_old,
             lambda: not self._stale_end_effector_stack(), 30),
            ('热改 mission_executor 末端参数',
             lambda: self._swap_set_param(new_mode),
             lambda: self._live_mode == new_mode, 15),
            ('调整车前感知', lambda: self._swap_apply_percep(want_percep),
             (lambda: self.probe.has_node('front_perception_node'))
             if want_percep else None,
             # D435 initial_reset + YOLO-World 模型加载在 Jetson 上较慢
             120 if want_percep else 0),
            ('恢复语音调度', self._swap_restore_services,
             (lambda: self.probe.has_node('brain_node'))
             if self._swap_restore_brain else None,
             30 if self._swap_restore_brain else 0),
        ]
        self._seq_done_fn = self._hot_swap_done
        self._seq_timer.start(1000)
        self._refresh_buttons()

    def _swap_stop_old(self):
        # 语音调度也要停: end_effector_mode 是它的启动参数, 改不了只能重启
        if self._group_running('brain'):
            self._stop_group('brain')
        if self._group_running('arm'):
            self._stop_group('arm')
        else:
            # 进程虽已不在, 残留节点仍可能占着服务名, 照样清一遍
            self._sweep_group('arm')

    def _swap_set_param(self, new_mode):
        """后台线程下发参数 + 回读, 结果写入 _live_mode 供 ready_fn 判定."""
        self._live_mode = None

        def run():
            ok, reason = self.probe.set_end_effector_mode(new_mode)
            if not ok:
                self.sig_sys.emit('热改末端参数失败: %s' % reason)
                return
            back = self.probe.get_end_effector_mode()
            if back != new_mode:
                self.sig_sys.emit('末端参数回读不一致 (期望 %s, 实际 %s)'
                                  % (new_mode, back))
                return
            self._live_mode = back
            self.sig_sys.emit('mission_executor 末端已热切换为 %s 并回读确认'
                              % back)

        threading.Thread(target=run, daemon=True).start()

    def _swap_apply_percep(self, want_percep):
        running = self._group_running('percep')
        if want_percep and not running:
            self._start_group('percep')
        elif not want_percep and running:
            self._stop_group('percep')

    def _swap_restore_services(self):
        if self._swap_restore_brain:
            self._start_group('brain')

    def _hot_swap_done(self):
        self._swap_active = False
        self._seq_fail_fn = None
        msg = ('末端已热切换为「%s」，定位/Nav 未中断。'
               '请确认已物理更换末端后，再点「机械臂」行的启动。'
               % MODES[self._mode]['title'])
        self._sys_log(msg)
        self._status_label.setText(msg)
        self._refresh_buttons()

    def _hot_swap_failed(self):
        """任一步超时: 回滚面板模式, 并明确告知系统当前处于半切换状态."""
        self._swap_active = False
        self._seq_fail_fn = None
        stale = self._stale_end_effector_stack()
        prev = self._swap_prev_mode
        if prev is not None:
            self._set_mode_now(prev)
        detail = ('仍在线的末端软件栈残留: %s' % '、'.join(stale)) if stale \
            else '参数下发或回读未成功'
        text = ('热换末端失败，已回滚面板显示为「%s」。\n%s\n\n'
                '定位/Nav 未受影响。请检查日志后重试，'
                '或停止相关组用常规方式切换。'
                % (MODES[prev]['title'] if prev else '未选择', detail))
        self._sys_log('热换末端失败: %s' % detail)
        QMessageBox.warning(self, '热换末端失败', text)
        self._refresh_buttons()

    # ---------- 进程组管理 ----------

    def _group_running(self, key):
        return any(p.active() for k, p in self._procs.items()
                   if k == key or k.startswith(key + '_'))

    def _toggle_group(self, key):
        if self._group_running(key):
            self._stop_group(key)
        else:
            self._start_group(key)

    def _start_group(self, key, delay_s=0):
        """启动一个进程组。delay_s 为整组附加的错峰延时 (秒)。"""
        if key in ('nav', 'arm', 'brain') and not self._require_mode(
                '启动%s' % self._rows[key]['title']):
            return
        if key == 'arm':
            conflict = self._end_effector_software_conflict()
            if conflict:
                self._sys_log('机械臂启动被阻止: %s' % conflict)
                QMessageBox.warning(self, '末端软件互斥', conflict)
                return
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
        if key == 'percep':
            # 路边捡瓶只开放给二指; mission_executor 那侧也有同样的硬门,
            # 这里拦一道是为了不让无用的 D435 + YOLO 白占 Jetson 的 CPU。
            if self._mode not in NEEDS_PERCEP:
                self._sys_log('路边捡瓶感知只用于二指, 当前末端不启动')
                return
            if self.probe.has_node('front_perception_node'):
                self._sys_log('路边捡瓶感知已在运行, 不重复启动')
                return
        if key == 'mapping':
            if self._seq_busy() or self._swap_active:
                # 建图会停掉 nav/arm/brain/percep, 半路插进热换会把状态搅烂
                self._sys_log('一键启动/热换末端进行中, 暂不开启建图')
                return
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
            self._start_proc(pkey, title, cmd, delay_s=delay + delay_s)
        self._sys_log('启动组: %s' % self._rows[key]['title'])
        if key in ('camera', 'nav'):
            # nav 组内含相机: 补登录(拍照/云台要用)。图像流按需, 见 _camera_post_start
            self._camera_post_start()
        if key == 'ocr':
            self._ensure_camera_stream('OCR')
            self._ocr_post_start()
        if key == 'nav':
            # 原 trash_pipeline 会连带拉起车前感知和 MyPanel 版 RViz, 并且
            # 分别延后 12s / 16s。拆组后必须把这个错峰照搬过来, 否则
            # D435 initial_reset + YOLO 加载会和 FAST_LIO/Nav2 初始化撞车。
            # 用 _start_proc 自带的延时: 期间点"停止"可取消待启动的进程。
            if self._mode in NEEDS_PERCEP \
                    and not self._group_running('percep'):
                self._start_group('percep', delay_s=PERCEP_START_DELAY_S)
            if self._mode in NAV_AUTO_RVIZ \
                    and not self._group_running('rviz'):
                self._start_group('rviz', delay_s=RVIZ_START_DELAY_S)

    def _group_cmds(self, key):
        if key == 'nav':
            # 与末端无关的底座; end_effector_mode 只是 mission_executor 初始值
            return [('nav', '定位Nav', NAV_CMD % self._mode)]
        if key == 'percep':
            return [('percep', '车前感知', PERCEP_CMD)]
        if key == 'arm':
            mode = MODES[self._mode]
            return mode['arm_cmds']
        if key == 'brain':
            return [('brain', '语音调度', FIXED_CMDS['brain'][1]
                     + ' end_effector_mode:=%s' % self._mode)]
        if key == 'mapping':
            return MAPPING_CMDS
        title, cmd = FIXED_CMDS[key]
        return [(key, title, cmd)]

    def _start_proc(self, pkey, title, cmd, delay_s=0):
        if pkey in self._procs and self._procs[pkey].active():
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
            proc.start_delayed(delay_s * 1000)
        else:
            proc.start()
        self._refresh_buttons()

    def _stop_group(self, key):
        if (self._mode == 'polish' and key in ('nav', 'arm')
                and self._polish_busy()):
            self._request_polish_safe_stop({key})
            return
        self._stop_group_now(key)

    def _stop_group_now(self, key):
        for k, p in self._procs.items():
            if (k == key or k.startswith(key + '_')) and p.active():
                p.stop()
        if key in ('camera', 'nav'):
            # 相机节点没了, 流自然也没了
            self._stream_on = False
        self._sys_log('停止组: %s' % self._rows[key]['title'])
        self._sweep_group(key)

    def _polish_busy(self):
        status = getattr(self.probe, 'polish_status', '')
        phase = status.split(':', 1)[0]
        if phase in ('RUNNING', 'CANCELING', 'HOMING'):
            return True
        if phase in ('IDLE', 'COMPLETED', 'FAILED', 'CANCELED'):
            return False
        mission = self.probe.mission_status
        return mission is not None and mission.state == 12

    def _request_polish_safe_stop(self, groups):
        """先请求关磨头/退力控/退刀/Home2，确认终态后才杀进程。"""
        self._pending_safe_stops.update(groups)
        self._safe_stop_deadline = max(
            self._safe_stop_deadline, time.monotonic() + 120.0)
        try:
            from std_srvs.srv import Trigger
            if self.probe.cancel_cli.service_is_ready():
                self.probe.cancel_cli.call_async(Trigger.Request())
            if self.probe.polish_cancel_cli.service_is_ready():
                self.probe.polish_cancel_cli.call_async(Trigger.Request())
        except Exception as exc:
            self._sys_log('请求打磨安全取消异常: %s' % exc)
        self._status_label.setText('正在安全停止打磨：关磨头→退力控→退刀→Home2')
        self._sys_log('已请求安全取消，确认回到 Home2 后再停止相关进程')
        QTimer.singleShot(1000, self._poll_polish_safe_stop)

    def _poll_polish_safe_stop(self):
        if not self._pending_safe_stops:
            return
        if not self._polish_busy():
            groups = list(self._pending_safe_stops)
            self._pending_safe_stops.clear()
            self._safe_stop_deadline = 0.0
            for key in groups:
                self._stop_group_now(key)
            self._status_label.setText('打磨已安全收尾，相关进程已停止')
            return
        if time.monotonic() >= self._safe_stop_deadline:
            groups = '、'.join(sorted(self._pending_safe_stops))
            self._pending_safe_stops.clear()
            self._safe_stop_deadline = 0.0
            text = ('120秒内未确认打磨安全收尾，未强杀%s；请检查打磨日志和机械臂。'
                    % groups)
            self._sys_log(text)
            self._status_label.setText(text)
            QMessageBox.critical(self, '安全停止未确认', text)
            return
        QTimer.singleShot(1000, self._poll_polish_safe_stop)

    # 脱离进程组的"弹窗进程"特征 (gnome-terminal 由系统服务代管,
    # 不在 launch 的进程组里, killpg 够不到, 按命令行特征精准清理)
    SWEEP_PATTERNS = {
        'arm': [r'yolo_grasp[.]py',
                r'soft_touch[.]launch[.]py',
                r'gripper_serve[r]',
                r'ysURForceAppContro[l]',
                r'ysAppComman[d]',
                r'elite_joint_trajectory_bridg[e]',
                r'depth_board_detect_nod[e]',
                r'percipio_camer[a]'],
        'nav': [r'wit_ros2_im[u]',           # IMU 终端窗
                r'velodyne_localizatio[n]',  # FAST_LIO 定位终端窗
                r'mission_executor[.]launch',  # mission_executor 终端窗
                r'nav2_bringu[p]'],          # Nav2 终端窗
        # 车前感知: D435 由 rs_launch 起, killpg 通常够, 残留时按特征补清
        'percep': [r'front_perception_nod[e]',
                   r'realsense2_camera_nod[e]'],
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

    def _seq_busy(self):
        """顺序流程是否占用中。必须连 _seq_ready_fn 一起看: 最后一步弹出后
        _seq_steps 会先空掉, 而就绪等待仍在进行, 此时插入新流程会让旧的
        ready_fn/超时错误地作用到新流程的 done/fail 钩子上。"""
        return bool(self._seq_steps) or self._seq_ready_fn is not None

    def _abort_seq(self, reason):
        """中止顺序流程, 且不触发 done/fail 钩子 (避免误报成功或误回滚)."""
        if not self._seq_busy() and not self._swap_active:
            return
        self._seq_timer.stop()
        self._seq_steps = []
        self._seq_ready_fn = None
        self._seq_fail_fn = None
        self._seq_done_fn = None
        if self._swap_active:
            self._swap_active = False
            # 半切换状态: 参数可能已改也可能没改, 不猜。强制重新回读,
            # 让"实际生效末端"标签把真相显示出来; 不一致时
            # _end_effector_software_conflict() 会拦住启动机械臂。
            self._live_mode = None
            self._live_mode_next = 0.0
            self._sys_log('热换末端被中止(%s): 请看"实际生效末端"确认当前状态'
                          % reason)
        else:
            self._sys_log('%s被中止(%s)' % (self._seq_title, reason))
        self._refresh_buttons()

    def _stop_all(self):
        self._abort_seq('全部停止')
        self._stream_on = False
        safe_polish = self._mode == 'polish' and self._polish_busy()
        for p in self._procs.values():
            if p.active() and not (safe_polish and
                                    (p.key == 'arm' or p.key.startswith('arm_'))):
                p.stop()
        for key in self.SWEEP_PATTERNS:
            if safe_polish and key == 'arm':
                continue
            self._sweep_group(key)
        if safe_polish:
            self._request_polish_safe_stop({'arm'})
        else:
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

    def _arm_online(self):
        if self._mode == 'polish':
            return (self.probe.has_node('ysURForceAppControl')
                    and self.probe.has_service('/elite_polish/run')
                    and self.probe.has_service('/force_mode_server/set_force_mode')
                    and self.probe.has_publisher('/elite_forceapp_cmd_result')
                    and self.probe.has_publisher('/camera/depth/image_raw'))
        if self._mode == 'softtouch':
            # 柔触必须额外确认底层 Modbus 桥接节点和服务都在线，
            # 否则 UI 会误判“机械臂在线”，但实际柔触控制通道未就绪。
            return (self.probe.has_node('robot_cartesian_control')
                    and self.probe.has_service('/yolo_grasp/grasp_hold')
                    and self.probe.has_node('gripper_server')
                    and self.probe.has_service('/gripper_command')
                    and self.probe.has_publisher('/gripper_pressure'))
        if self._mode in ('twofinger', 'linkerhand'):
            return (self.probe.has_node('robot_cartesian_control')
                    and self.probe.has_service('/yolo_grasp/grasp_hold'))
        return False

    def _end_effector_software_conflict(self):
        """阻止不同末端的软件栈同时在线，作为物理互斥的第二道门。"""
        if self._mode == 'polish':
            if self.probe.has_service('/yolo_grasp/grasp_hold'):
                return '检测到抓取服务仍在线，请先停止二指/柔触/灵巧手软件栈。'
        elif self._mode in ('twofinger', 'softtouch', 'linkerhand'):
            if (self.probe.has_service('/elite_polish/run')
                    or self.probe.has_node('ysURForceAppControl')):
                return '检测到打磨软件栈仍在线，请先安全停止打磨系统。'
        # 三种抓取末端共用 /yolo_grasp/*, 靠服务名区分不了, 只能用
        # mission_executor 实际生效的参数兜底: 不一致说明面板与节点脱节,
        # 此时起机械臂会用错误的停车距离/动作白名单跑, 必须拦住。
        if (self._live_mode is not None and self._live_mode != self._mode
                and self.probe.has_node(MISSION_EXECUTOR_NODE)):
            live_title = MODES.get(self._live_mode, {}).get(
                'title', self._live_mode)
            return ('mission_executor 实际生效末端为「%s」，与面板所选「%s」'
                    '不一致。\n请重新点选目标末端完成热切换，'
                    '或停止定位Nav后常规切换。'
                    % (live_title, MODES[self._mode]['title']))
        return ''

    def _start_all(self):
        if self._seq_busy() or self._swap_active:
            self._status_label.setText('一键启动/热换末端已在进行中')
            return
        if not self._require_mode('一键全部启动'):
            return
        conflict = self._end_effector_software_conflict()
        if conflict:
            self._sys_log('一键启动被阻止: %s' % conflict)
            QMessageBox.warning(self, '末端软件互斥', conflict)
            return
        arm_title = '启动 机械臂(%s)' % MODES[self._mode]['title']
        arm_timeout = 180 if self._mode == 'polish' else 90
        steps = [
            ('配置 CAN', self._setup_can, None, 0),
            ('启动 定位+Nav', lambda: self._start_group('nav'),
             lambda: self.probe.has_publisher('/mission/status')
             or self.probe.has_node('mission_executor'), 60),
            (arm_title, lambda: self._start_group('arm'),
             self._arm_online, arm_timeout),
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
        self._seq_title = '一键启动'
        self._seq_fail_fn = None
        self._seq_done_fn = None
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
                self._sys_log('等待超时，%s 已中止，请检查该组日志'
                              % self._seq_title)
                self._seq_ready_fn = None
                self._seq_steps = []
                self._seq_timer.stop()
                self._status_label.setText('%s失败：组件等待超时'
                                           % self._seq_title)
                fail_fn, self._seq_fail_fn = self._seq_fail_fn, None
                self._seq_done_fn = None
                if fail_fn is not None:
                    fail_fn()
                return
            else:
                return
        if not self._seq_steps:
            self._seq_timer.stop()
            self._seq_fail_fn = None
            done_fn, self._seq_done_fn = self._seq_done_fn, None
            self._sys_log('%s完成' % self._seq_title)
            if done_fn is not None:
                done_fn()
            else:
                self._status_label.setText('%s完成' % self._seq_title)
            return
        desc, start_fn, ready_fn, timeout = self._seq_steps.pop(0)
        self._status_label.setText('%s: %s ...' % (self._seq_title, desc))
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
            needs_mode = k in ('nav', 'arm', 'brain')
            self._rows[k]['btn'].setEnabled(
                not mapping_on and not self._swap_active
                and (not needs_mode or self._mode in MODES
                     or self._group_running(k)))
        # 路边捡瓶感知只有二指能启动; 非二指时若已在跑, 仍允许点"停止"清掉
        self._rows['percep']['btn'].setEnabled(
            not mapping_on and not self._swap_active
            and (self._mode in NEEDS_PERCEP
                 or self._group_running('percep')))
        mission_busy = self._mission_busy()
        # 硬锁: 建图中 / 任务执行中 / 热换进行中, 一律不许动末端
        hard_locked = mapping_on or mission_busy or self._swap_active
        stack_on = (self._group_running('nav') or self._group_running('arm')
                    or self._group_running('brain'))
        self._btn_all.setEnabled(not mapping_on and not self._swap_active
                                 and self._mode in MODES)
        # 建图不在 MUTEX_WITH_MAPPING 里, 需单独锁: 它会停 nav/arm/brain/percep
        self._rows['mapping']['btn'].setEnabled(not self._swap_active)
        for key, radio in (('twofinger', self._radio_two),
                           ('softtouch', self._radio_soft),
                           ('linkerhand', self._radio_hand),
                           ('polish', self._radio_polish)):
            if hard_locked:
                radio.setEnabled(False)
            elif not stack_on:
                radio.setEnabled(True)      # 系统全停: 自由选择
            else:
                # 系统在跑: 只允许抓取家族内部热换 (定位/Nav 不停),
                # 打磨头进出都要整组停启, 因此置灰。
                radio.setEnabled(key in HOT_SWAPPABLE
                                 and self._mode in HOT_SWAPPABLE)
        # 保存建图: 只有 /map_save 服务在线才可点
        try:
            self._btn_save_map.setEnabled(self.probe.has_service('/map_save'))
        except Exception:
            self._btn_save_map.setEnabled(False)
        # 开流/停流: 相机节点在线才可点
        self._btn_stream.setText('停流' if self._stream_on else '开流')
        try:
            self._btn_stream.setEnabled(
                self.probe.has_node('hk_camera_node'))
        except Exception:
            self._btn_stream.setEnabled(False)

    def _refresh_live_mode(self):
        """回读 mission_executor 实际生效的末端 (后台线程, 主线程只读缓存)."""
        if self._swap_active:
            # 热换进行中: _live_mode 由 _swap_set_param 独占写入, 这里只渲染,
            # 否则轮询/节点瞬时消失会把它清掉, 让切换步骤白等到超时。
            self._mode_live_label.setText('实际生效末端: 切换中...')
            self._mode_live_label.setStyleSheet('')
            return
        if not self.probe.has_node(MISSION_EXECUTOR_NODE):
            self._live_mode = None
            self._mode_live_label.setText(
                '实际生效末端: — (mission_executor 未运行)')
            self._mode_live_label.setStyleSheet('')
            return
        now = time.monotonic()
        if not self._live_mode_inflight and now >= self._live_mode_next:
            self._live_mode_inflight = True

            def run():
                try:
                    self._live_mode = self.probe.get_end_effector_mode()
                finally:
                    self._live_mode_next = time.monotonic() + 3.0
                    self._live_mode_inflight = False

            threading.Thread(target=run, daemon=True).start()
        live = self._live_mode
        if live is None:
            self._mode_live_label.setText('实际生效末端: 回读中...')
            self._mode_live_label.setStyleSheet('')
            return
        title = MODES.get(live, {}).get('title', live)
        if self._mode is not None and live != self._mode:
            self._mode_live_label.setText(
                '⚠ 实际生效末端: %s，与面板所选「%s」不一致'
                % (title, MODES[self._mode]['title']))
            self._mode_live_label.setStyleSheet('color:#ff6b6b;')
        else:
            self._mode_live_label.setText('实际生效末端: %s' % title)
            self._mode_live_label.setStyleSheet('color:#7ed37e;')

    def _refresh_status(self):
        self._refresh_buttons()
        self._refresh_live_mode()
        checks = {
            # 一律用节点名判活: 服务/话题注册在进程被杀后会残留, 节点名清得快
            'nav': lambda: self.probe.has_node(MISSION_EXECUTOR_NODE),
            'percep': lambda: self.probe.has_node('front_perception_node'),
            'arm': self._arm_online,
            'brain': lambda: self.probe.has_node('brain_node'),
            'aiui': lambda: self.probe.has_node('aiui_ros_node'),
            'camera': lambda: self.probe.has_node('hk_camera_node'),
            'ocr': lambda: self.probe.has_node('ocr_node'),
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
        online = self.probe.has_node('mission_executor')
        busy = s is not None and s.state not in MISSION_FREE_STATES
        self._mission['btn_start'].setEnabled(
            online and not busy and self._mode in MODES)

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
            mode_ok = key != 'dep_exec' or self._mode in MODES
            dep['btn'].setEnabled(running or (not nav_on and mode_ok))

    # ---------- 关闭 ----------

    def closeEvent(self, event):
        if self._mode == 'polish' and self._polish_busy():
            self._request_polish_safe_stop({'nav', 'arm'})
            QMessageBox.warning(
                self, '正在安全停止打磨',
                '当前打磨尚未安全收尾，已请求取消。\n'
                '确认回到 Home2 后请再次关闭面板。')
            event.ignore()
            return
        running = [p.title for p in self._procs.values() if p.active()]
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


# ---------- 深色科技风主题 (机械绿强调色) ----------
THEME_QSS = """
* { font-size: 13px; }
QMainWindow, QWidget {
    background: #1b1e24;
    color: #d7dde4;
}
QGroupBox {
    background: #22262d;
    border: 1px solid #333a43;
    border-radius: 8px;
    margin-top: 14px;
    padding-top: 8px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 6px;
    color: #8fd19a;
    font-weight: bold;
}
QPushButton {
    background: #2b3038;
    border: 1px solid #3c434d;
    border-radius: 6px;
    padding: 6px 10px;
    min-height: 22px;
    color: #d7dde4;
}
QPushButton:hover { background: #343b45; border-color: #4a525d; }
QPushButton:pressed { background: #242932; }
QPushButton:disabled {
    background: #23262c;
    color: #5d656e;
    border-color: #2d3238;
}
QPushButton#primary {
    background: #2eb85c;
    border: none;
    color: #ffffff;
    font-weight: bold;
}
QPushButton#primary:hover { background: #35cb68; }
QPushButton#primary:pressed { background: #27a050; }
QPushButton#primary:disabled { background: #24563a; color: #8fa598; }
QPushButton#danger {
    background: #3a2626;
    border: 1px solid #a04545;
    color: #e8a0a0;
}
QPushButton#danger:hover { background: #472e2e; border-color: #c05555; }
QTabWidget::pane {
    border: 1px solid #333a43;
    border-radius: 6px;
    top: -1px;
}
QTabBar::tab {
    background: #22262d;
    border: 1px solid #333a43;
    border-bottom: none;
    border-top-left-radius: 6px;
    border-top-right-radius: 6px;
    padding: 7px 14px;
    margin-right: 2px;
    color: #9aa4af;
}
QTabBar::tab:selected {
    background: #1b1e24;
    color: #2eb85c;
    border-bottom: 2px solid #2eb85c;
    font-weight: bold;
}
QTabBar::tab:hover:!selected { color: #d7dde4; }
QPlainTextEdit, QListWidget {
    background: #14171b;
    border: 1px solid #2d333b;
    border-radius: 6px;
    color: #c9d2da;
    font-family: "DejaVu Sans Mono", "Monospace", monospace;
    font-size: 12px;
}
QListWidget::item { padding: 4px 6px; border-radius: 4px; }
QListWidget::item:selected { background: #24563a; color: #d8f0de; }
QListWidget::item:hover:!selected { background: #22262d; }
QLineEdit, QSpinBox, QComboBox {
    background: #14171b;
    border: 1px solid #2d333b;
    border-radius: 5px;
    padding: 4px 8px;
    color: #d7dde4;
    min-height: 20px;
}
QLineEdit:focus, QSpinBox:focus, QComboBox:focus { border-color: #2eb85c; }
QComboBox::drop-down { border: none; width: 22px; }
QComboBox QAbstractItemView {
    background: #22262d;
    border: 1px solid #3c434d;
    selection-background-color: #2eb85c;
    selection-color: #ffffff;
    color: #d7dde4;
}
QCheckBox, QRadioButton { color: #d7dde4; spacing: 6px; }
QCheckBox::indicator, QRadioButton::indicator {
    width: 15px; height: 15px;
    border: 1px solid #3c434d;
    background: #14171b;
}
QCheckBox::indicator { border-radius: 3px; }
QCheckBox::indicator:checked {
    background: #2eb85c; border-color: #2eb85c;
    image: none;
}
QRadioButton::indicator { border-radius: 8px; }
QRadioButton::indicator:checked {
    background: #2eb85c; border-color: #2eb85c;
}
QScrollBar:vertical {
    background: #1b1e24; width: 10px; margin: 2px;
    border-radius: 5px;
}
QScrollBar::handle:vertical {
    background: #3c434d; min-height: 24px; border-radius: 5px;
}
QScrollBar::handle:vertical:hover { background: #4a525d; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar:horizontal {
    background: #1b1e24; height: 10px; margin: 2px;
    border-radius: 5px;
}
QScrollBar::handle:horizontal {
    background: #3c434d; min-width: 24px; border-radius: 5px;
}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }
QToolTip {
    background: #2b3038; color: #d7dde4;
    border: 1px solid #3c434d; padding: 4px 8px;
}
"""


def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(THEME_QSS)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
