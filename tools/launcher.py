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

# ---------- 环境自举: 未 source ROS 时自动注入, 保证双击可用 ----------
if 'ROS_DISTRO' not in os.environ:
    try:
        out = subprocess.check_output(
            ['bash', '-c', 'source /opt/ros/humble/setup.bash && env'],
            text=True, stderr=subprocess.DEVNULL)
        for line in out.splitlines():
            k, _, v = line.partition('=')
            if k:
                os.environ[k] = v
    except subprocess.CalledProcessError:
        pass  # rclpy 不可用时降级为"仅进程管理"模式
os.environ['ROS_DOMAIN_ID'] = '42'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

from PyQt5.QtCore import QProcess, Qt, QTimer  # noqa: E402
from PyQt5.QtGui import QColor, QPalette  # noqa: E402
from PyQt5.QtWidgets import (QApplication, QButtonGroup, QCheckBox,  # noqa: E402
                             QGroupBox, QHBoxLayout, QLabel, QMainWindow,
                             QMessageBox, QPlainTextEdit, QPushButton,
                             QRadioButton, QTabWidget, QVBoxLayout, QWidget)

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
    'rviz': ('RViz', KYBOT_SETUP + 'rviz2'),
}

LOG_LINE_LIMIT = 5000


class RosProbe:
    """内嵌 rclpy 探针: 查 ROS 图判断各组是否真的在线. rclpy 不可用时全部返回 False."""

    def __init__(self):
        self.node = None
        try:
            import rclpy
            rclpy.init(args=None)
            self.node = rclpy.create_node('kybot_launcher_probe')
            self._rclpy = rclpy
            self._thread = threading.Thread(target=self._spin, daemon=True)
            self._thread.start()
        except Exception:
            self.node = None

    def _spin(self):
        from rclpy.executors import SingleThreadedExecutor
        ex = SingleThreadedExecutor()
        ex.add_node(self.node)
        while True:
            try:
                ex.spin_once(timeout_sec=0.5)
            except Exception:
                return

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
        if self.node is not None:
            try:
                self.node.destroy_node()
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
    def __init__(self):
        super().__init__()
        self.setWindowTitle('KYBOT 启动面板')
        self.resize(1020, 660)
        self._procs = {}          # key -> Proc
        self._mode = 'twofinger'  # 默认二指
        self._seq_steps = []      # 一键启动的待执行步骤
        self._seq_wait_until = 0.0
        self._seq_ready_fn = None

        self.probe = RosProbe()
        self._build_ui()
        self._apply_mode('twofinger', force=True)

        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._refresh_status)
        self._status_timer.start(2000)
        self._seq_timer = QTimer(self)
        self._seq_timer.timeout.connect(self._seq_tick)

    # ---------- UI ----------

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

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
                           ('rviz', 'RViz')]:
            row = self._make_group_row(key, title, gv)
            self._rows[key] = row
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
        cmds = self._group_cmds(key)
        if not cmds:
            return
        for i, (pkey, title, cmd) in enumerate(cmds):
            delay = 0
            if key == 'arm' and i > 0:
                delay = MODES[self._mode]['arm_grasp_delay_s']
            self._start_proc(pkey, title, cmd, delay_s=delay)
        self._sys_log('启动组: %s' % self._rows[key]['title'])

    def _group_cmds(self, key):
        mode = MODES[self._mode]
        if key == 'nav':
            return [('nav', '定位Nav', mode['nav_cmd'])]
        if key == 'arm':
            return mode['arm_cmds']
        title, cmd = FIXED_CMDS[key]
        return [(key, title, cmd)]

    def _start_proc(self, pkey, title, cmd, delay_s=0):
        if pkey in self._procs and self._procs[pkey].running():
            return
        if pkey not in self._logs:
            tab = self._add_log_tab(pkey, title)
            row_key = pkey.split('_')[0]
            self._rows[row_key]['btn_log'].clicked.connect(
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

    def _stop_all(self):
        self._seq_steps = []
        for p in self._procs.values():
            if p.running():
                p.stop()
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

    def _refresh_status(self):
        self._refresh_buttons()
        checks = {
            'nav': lambda: self.probe.has_publisher('/mission/status')
            or self.probe.has_node('mission_executor'),
            'arm': lambda: self.probe.has_service('/yolo_grasp/grasp_hold'),
            'brain': lambda: self.probe.has_node('brain_node'),
            'aiui': lambda: self.probe.has_node('aiui_ros_node'),
            'rviz': lambda: self.probe.has_node('rviz'),
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
