"""LLM 工具层: schema 定义 + 分发执行.

每个工具都是现有 ROS2 服务的纯客户端调用, 不新增任何服务端.
安全约束: 导航只接受 location.yaml 白名单内的点位名, LLM 不接触原始坐标.
"""

import json
import math
import os
import threading
import time

from geometry_msgs.msg import Twist
from hk_camera.msg import MissionStatus, MissionWaypoint
from hk_camera.srv import CapturePicture, RunMission
from ocr_interfaces.srv import RecognizeText
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .waypoints import find_waypoint, load_waypoints

# approach 逼近的传感超时 (与 mission_executor 一致)
ODOM_TIMEOUT_SEC = 0.3
SCAN_TIMEOUT_SEC = 0.5

# 任务"空闲/终态"集合, 其余视为执行中
FREE_STATES = {
    MissionStatus.STATE_IDLE,
    MissionStatus.STATE_COMPLETED,
    MissionStatus.STATE_FAILED,
    MissionStatus.STATE_CANCELED,
}

STATE_NAMES = {
    MissionStatus.STATE_IDLE: '空闲',
    MissionStatus.STATE_NAVIGATING: '导航中',
    MissionStatus.STATE_PTZ_MOVING: '云台运动中',
    MissionStatus.STATE_CAPTURING: '拍照中',
    MissionStatus.STATE_COMPLETED: '已完成',
    MissionStatus.STATE_FAILED: '失败',
    MissionStatus.STATE_CANCELED: '已取消',
    MissionStatus.STATE_BOTTLE_CONFIRMING: '目标确认中',
    MissionStatus.STATE_APPROACHING: '逼近目标中',
    MissionStatus.STATE_GRASPING: '抓取中',
    MissionStatus.STATE_PLACING: '放置中',
    MissionStatus.STATE_RETREATING: '退回中',
}

# 任务终态集合
TERMINAL_STATES = {
    MissionStatus.STATE_COMPLETED: '已完成',
    MissionStatus.STATE_FAILED: '失败',
    MissionStatus.STATE_CANCELED: '已取消',
}


class WaitInterrupted(Exception):
    """等待任务完成期间收到新的用户指令, 中断等待, 交回对话循环."""

# OpenAI function calling 工具描述
TOOL_SCHEMAS = [
    {
        'type': 'function',
        'function': {
            'name': 'list_waypoints',
            'description': '列出所有可导航的点位名称及其到达后的动作(拍照/抓取等)',
            'parameters': {'type': 'object', 'properties': {}, 'required': []},
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'goto_waypoint',
            'description': '让机器人导航到指定名称的点位并执行该点位配置的动作(如拍照/抓取)。只能用 list_waypoints 里存在的点位名。',
            'parameters': {
                'type': 'object',
                'properties': {
                    'name': {'type': 'string', 'description': '点位名称, 必须是白名单里的名字'},
                },
                'required': ['name'],
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'run_route',
            'description': '依次执行多点任务: 按顺序导航到多个点位并执行各点位配置的动作。用于"去点位1拍照抓取后再回来"这类往返/多目标指令(回来也要给白名单里的点位名, 如"出发点")。单点任务用 goto_waypoint。',
            'parameters': {
                'type': 'object',
                'properties': {
                    'names': {
                        'type': 'array',
                        'items': {'type': 'string'},
                        'description': '点位名称列表, 按执行顺序排列, 必须都在白名单内',
                    },
                },
                'required': ['names'],
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'navigate_and_wait',
            'description': '导航到指定点位并等待真正到达才返回(纯导航, 不执行该点yaml配置的拍照/抓取动作, 那些由你按需单独调用)。返回真实结果: 到达/失败/被取消/超时。',
            'parameters': {
                'type': 'object',
                'properties': {
                    'name': {'type': 'string', 'description': '点位名称, 必须是白名单里的名字'},
                },
                'required': ['name'],
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'approach',
            'description': '底盘慢速直线逼近前方目标(0.1m/s, 前向2D雷达测距, 距障碍约0.7m自动停车, 返回真实前进距离和前方距离)。导航到点位后、抓取前必须调用。',
            'parameters': {
                'type': 'object',
                'properties': {
                    'distance': {'type': 'number', 'default': 1.5,
                                 'description': '最长逼近距离(米), 默认1.5, 上限2.0'},
                },
                'required': [],
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'retreat',
            'description': '底盘慢速直线后退(0.1m/s)。grasp 结束后会自动退回, 一般不用手动调; 用户明确说"后退一点"时用。',
            'parameters': {
                'type': 'object',
                'properties': {
                    'distance': {'type': 'number',
                                 'description': '后退距离(米), 默认退回上次逼近的距离, 上限2.0'},
                },
                'required': [],
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'grasp',
            'description': '机械臂在当前位置视觉识别并抓取目标(瓶子等), 抓完自动原路后退到导航位置。返回真实抓取结果, 最长约2分钟。必须在 approach 逼近到位后调用。',
            'parameters': {'type': 'object', 'properties': {}, 'required': []},
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'place',
            'description': '把机械臂抓着的物体放到示教放置位并松手, 返回真实结果。',
            'parameters': {'type': 'object', 'properties': {}, 'required': []},
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'arm_home',
            'description': '机械臂收拢回 Home2 安全位姿(巡检/移动前的标准收臂动作)。',
            'parameters': {'type': 'object', 'properties': {}, 'required': []},
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'arm_open',
            'description': '张开机械手, 松开抓着的物体(原地放下)。用于"放到这里/松手"这类不需要示教放置位的场景。',
            'parameters': {'type': 'object', 'properties': {}, 'required': []},
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'arm_ready',
            'description': '机械臂运动到预备位姿。一般不需要调用(抓取服务内部会自行处理姿态), 仅用户明确要求时使用。',
            'parameters': {'type': 'object', 'properties': {}, 'required': []},
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'cancel_mission',
            'description': '取消当前正在执行的巡检/导航任务',
            'parameters': {'type': 'object', 'properties': {}, 'required': []},
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'get_mission_status',
            'description': '查询当前任务执行状态(空闲/导航中/已完成等)',
            'parameters': {'type': 'object', 'properties': {}, 'required': []},
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'capture_photo',
            'description': '控制相机在当前位置立即拍一张照片',
            'parameters': {
                'type': 'object',
                'properties': {
                    'quality': {'type': 'integer', 'description': '图片质量 0-2, 0 最好',
                                'default': 1},
                },
                'required': [],
            },
        },
    },
]


class ToolExecutor:
    """工具分发执行 + 审计日志. 在 brain_node 的工作线程里运行."""

    def __init__(self, node, waypoints_file, status_provider,
                 service_timeout_sec=10.0, nav_timeout_sec=300.0,
                 arm_timeout_sec=130.0, interrupt_check=None, set_waiting=None,
                 odom_provider=None, scan_provider=None, cmd_vel_pub=None,
                 approach_speed=0.1, approach_stop_distance=0.7,
                 approach_max_distance=1.5,
                 audit_file='~/.kybot_brain/audit.jsonl'):
        self._node = node
        self._logger = node.get_logger()
        self._waypoints_file = waypoints_file
        self._status_provider = status_provider  # callable -> MissionStatus | None
        self._timeout = service_timeout_sec
        self._nav_timeout = nav_timeout_sec      # navigate_and_wait 等终态上限
        self._arm_timeout = arm_timeout_sec      # 机械臂 Trigger 调用上限
        self._interrupt_check = interrupt_check or (lambda: False)
        self._set_waiting = set_waiting or (lambda active: None)
        # approach 逼近用: 传感数据提供方 + 底盘指令发布器
        self._odom_provider = odom_provider      # callable -> (Odometry, monotonic) | None
        self._scan_provider = scan_provider      # callable -> (LaserScan, monotonic) | None
        self._cmd_vel_pub = cmd_vel_pub
        self._approach_speed = approach_speed
        self._approach_stop = approach_stop_distance
        self._approach_max = approach_max_distance
        self._audit_file = os.path.expanduser(audit_file)
        os.makedirs(os.path.dirname(self._audit_file), exist_ok=True)
        self._audit_lock = threading.Lock()

        self._run_cli = node.create_client(RunMission, '/mission/run')
        self._cancel_cli = node.create_client(Trigger, '/mission/cancel')
        self._capture_cli = node.create_client(CapturePicture, '/hk_camera/capture')
        # 机械臂服务 (elite 侧 yolo_grasp.py, 跨机 DDS)
        self._grasp_cli = node.create_client(Trigger, '/yolo_grasp/grasp_hold')
        self._place_cli = node.create_client(Trigger, '/yolo_grasp/place')
        self._arm_home_cli = node.create_client(Trigger, '/yolo_grasp/home2')
        self._arm_ready_cli = node.create_client(Trigger, '/yolo_grasp/ready')
        self._arm_open_cli = node.create_client(Trigger, '/yolo_grasp/open')
        # OCR 识别 (拍照后顺手识别), 结果发 /ocr_feedback 供 UI 显示
        self._ocr_cli = node.create_client(RecognizeText, '/ocr/recognize')
        self._ocr_pub = node.create_publisher(String, '/ocr_feedback', 10)
        # 抓取硬门: 必须有一次成功的 approach 才允许 grasp; 新导航后复位
        self._approach_ok = False
        self._last_approach_dist = 0.0  # 上次逼近前进的距离, grasp 后原路退回用

    # ---------- 分发 ----------

    def execute(self, name, args):
        """执行一个工具, 返回给 LLM 的字符串结果. 绝不抛异常."""
        args = args or {}
        self._logger.info('执行工具 %s, 参数 %s'
                          % (name, json.dumps(args, ensure_ascii=False)))
        try:
            handler = getattr(self, '_tool_' + name, None)
            if handler is None:
                result = '未知工具: %s' % name
            else:
                result = handler(**args)
        except WaitInterrupted:
            raise  # 中断等待要传到对话循环, 不能吞
        except TypeError as exc:  # LLM 给了错误参数名
            result = '工具参数错误: %s' % exc
        except Exception as exc:  # noqa: BLE001 - 工具失败不能拖垮对话
            result = '工具执行异常: %s' % exc
        self._logger.info('工具 %s 结果: %s' % (name, result))
        self._audit(name, args, result)
        return result

    # ---------- 工具实现 ----------

    def _tool_list_waypoints(self):
        wps = self._load()
        if not wps:
            return '点位文件为空或读取失败, 请先用 RViz 面板录制点位'
        lines = ['%d. %s' % (i + 1, wp.brief()) for i, wp in enumerate(wps)]
        return '可导航点位:\n' + '\n'.join(lines)

    def _tool_goto_waypoint(self, name=''):
        name = str(name).strip()
        wps = self._load()
        wp = find_waypoint(wps, name)
        if wp is None:
            return ('拒绝执行: "%s" 不在点位白名单中。可用点位: %s'
                    % (name, '、'.join(w.name for w in wps) or '(空)'))
        return self._start_mission([wp], '正在前往"%s"' % wp.name)

    def _tool_run_route(self, names=None):
        wps = self._load()
        names = [str(n).strip() for n in (names or []) if str(n).strip()]
        if not names:
            return '参数错误: names 至少需要一个点位名'
        selected = [find_waypoint(wps, n) for n in names]
        bad = [n for n, wp in zip(names, selected) if wp is None]
        if bad:
            return ('拒绝执行: "%s" 不在点位白名单中。可用点位: %s'
                    % ('、'.join(bad), '、'.join(w.name for w in wps) or '(空)'))
        return self._start_mission(
            selected, '依次前往 %s' % ' → '.join(wp.name for wp in selected))

    def _start_mission(self, selected, desc):
        """忙时拒绝 + 下发 /mission/run (单点/多点共用)."""
        status = self._status_provider()
        if status is not None and status.state not in FREE_STATES:
            return ('拒绝执行: 当前有任务进行中(状态: %s), 请先取消或等待完成'
                    % STATE_NAMES.get(status.state, str(status.state)))
        req = RunMission.Request()
        for wp in selected:
            req.waypoints.append(self._to_mission_waypoint(wp))
        res = self._call(self._run_cli, req, '/mission/run')
        if res is None:
            return '调用 /mission/run 失败: 服务无应答(mission_executor 在运行吗?)'
        if res.accepted:
            self._approach_ok = False  # 位置变了, 之前的逼近作废
            return '任务已启动: %s。用 get_mission_status 可查进度。' % desc
        return '任务被拒绝: %s' % res.message

    def _tool_cancel_mission(self):
        res = self._call(self._cancel_cli, Trigger.Request(), '/mission/cancel')
        if res is None:
            return '调用 /mission/cancel 失败: 服务无应答'
        return ('已取消当前任务' if res.success
                else '取消失败: %s' % res.message)

    def _tool_get_mission_status(self):
        status = self._status_provider()
        if status is None:
            return '尚未收到任务状态(mission_executor 未运行?)'
        return ('状态: %s; 进度: %d/%d; 说明: %s'
                % (STATE_NAMES.get(status.state, str(status.state)),
                   status.current_index + 1, status.total_count, status.message))

    def _tool_capture_photo(self, quality=1):
        req = CapturePicture.Request()
        req.quality = max(0, min(2, int(quality)))
        req.save_path = ''  # 空 = 仅拍照发布, 走巡检流程的拍照才落盘
        res = self._call(self._capture_cli, req, '/hk_camera/capture')
        if res is None:
            return '调用 /hk_camera/capture 失败: 服务无应答(hk_camera 在运行吗?)'
        if not res.success:
            return '拍照失败: %s' % res.message
        return '拍照成功: %s%s' % (res.message, self._ocr_feedback())

    def _ocr_feedback(self):
        """拍照后调一次 OCR 识别当前画面, 结果发 /ocr_feedback 并返回描述."""
        req = RecognizeText.Request()
        req.conf_threshold = 0.0  # 用节点默认阈值
        # 40s: 容忍 OCR 首次推理的 GPU 冷启动(模型加载)和并发排队
        res = self._call(self._ocr_cli, req, '/ocr/recognize', timeout=40.0)
        if res is None:
            return ('；OCR这次没识别成功(服务未启动或响应超时, '
                    '照片已保存, 可稍后重试)')
        if not res.success:
            return '；OCR失败: %s' % res.message
        if not res.detections:
            text = '未识别到文字'
            detail = text
        else:
            items = ['%s(%.2f)' % (d.text, d.confidence)
                     for d in res.detections[:5]]
            text = '识别到 %d 处文字: %s' % (len(res.detections),
                                           '、'.join(items))
            detail = text
        msg = String()
        msg.data = '[语音拍照] %s' % detail
        self._ocr_pub.publish(msg)
        return '；OCR: %s' % text

    # ---------- 分步同步工具 (返回真实执行结果) ----------

    def _tool_navigate_and_wait(self, name=''):
        """纯导航 + 等待终态. 不触发点位 yaml 里配置的拍照/action."""
        name = str(name).strip()
        wp = find_waypoint(self._load(), name)
        if wp is None:
            wps = self._load()
            return ('拒绝执行: "%s" 不在点位白名单中。可用点位: %s'
                    % (name, '、'.join(w.name for w in wps) or '(空)'))
        status = self._status_provider()
        if status is not None and status.state not in FREE_STATES:
            return ('拒绝执行: 当前有任务进行中(状态: %s), 请先取消或等待完成'
                    % STATE_NAMES.get(status.state, str(status.state)))
        req = RunMission.Request()
        req.waypoints.append(self._to_mission_waypoint(wp, pure_nav=True))
        res = self._call(self._run_cli, req, '/mission/run')
        if res is None:
            return '调用 /mission/run 失败: 服务无应答(mission_executor 在运行吗?)'
        if not res.accepted:
            return '导航被拒绝: %s' % res.message
        self._approach_ok = False  # 位置变了, 之前的逼近作废
        return self._wait_mission_terminal('前往"%s"' % wp.name)

    def _tool_grasp(self):
        if not self._approach_ok:
            return ('拒绝抓取: 还没有成功逼近过目标。抓取前必须先调 approach '
                    '逼近到目标跟前(约0.7m雷达早停), 这是硬约束, 不要跳过。')
        result = self._call_arm(self._grasp_cli, '/yolo_grasp/grasp_hold', '抓取')
        # 与 executor 的 grasp 动作一致: 抓完(无论成败)原路退回导航点,
        # 避免贴着障碍时 Nav2 规划扭来扭去
        dist = self._last_approach_dist
        self._approach_ok = False
        if dist > 0.05:
            ok_r, info = self._retreat(dist)
            if ok_r:
                result += '；已后退 %.2f m 退回导航位置' % info
            else:
                result += '；后退失败(%s), 注意车仍贴着目标' % info
        return result

    def _tool_retreat(self, distance=None):
        """手动后退: 默认退回上次逼近的距离."""
        if distance is None:
            distance = self._last_approach_dist or 0.5
        try:
            target = max(0.1, min(2.0, float(distance)))
        except (TypeError, ValueError):
            return '参数错误: distance 必须是数字(米)'
        ok, info = self._retreat(target)
        if ok:
            self._approach_ok = False
            return '已后退 %.2f m' % info
        return '后退失败: %s' % info

    def _tool_place(self):
        return self._call_arm(self._place_cli, '/yolo_grasp/place', '放置')

    def _tool_arm_home(self):
        return self._call_arm(self._arm_home_cli, '/yolo_grasp/home2', '收臂')

    def _tool_arm_open(self):
        return self._call_arm(self._arm_open_cli, '/yolo_grasp/open', '张手')

    def _tool_arm_ready(self):
        return self._call_arm(self._arm_ready_cli, '/yolo_grasp/ready', '预备')

    def _tool_approach(self, distance=None):
        """开环慢速直线逼近: 判据复刻 executor 的 driveDistance
        (±10°前向窗, 0.7m 早停, 0.1m/s, odom/scan 超时保护)."""
        self._approach_ok = False  # 本次逼近的结果决定能否抓取
        if self._cmd_vel_pub is None or self._odom_provider is None \
                or self._scan_provider is None:
            return '逼近不可用: 底盘传感/指令接口未配置'
        try:
            target = float(distance) if distance is not None \
                else self._approach_max
        except (TypeError, ValueError):
            return '参数错误: distance 必须是数字(米)'
        target = max(0.1, min(2.0, target))  # 硬上限 2m, 防 LLM 乱填

        # 等新鲜里程计 (最多 2s)
        start = None
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self._check_interrupt()
            o = self._odom_provider()
            if o is not None and time.monotonic() - o[1] <= ODOM_TIMEOUT_SEC:
                start = o[0].pose.pose
                break
            time.sleep(0.05)
        if start is None:
            return '逼近中止: 等不到新鲜里程计(/odom 在发布吗?)'
        sx, sy = start.position.x, start.position.y
        q = start.orientation
        start_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cos_yaw, sin_yaw = math.cos(start_yaw), math.sin(start_yaw)

        max_time = target / self._approach_speed + 5.0
        t0 = time.monotonic()
        try:
            while time.monotonic() - t0 < max_time:
                self._check_interrupt()
                o = self._odom_provider()
                if o is None or time.monotonic() - o[1] > ODOM_TIMEOUT_SEC:
                    self._stop_base()
                    return '逼近中止: 里程计超时'
                pos = o[0].pose.pose.position
                traveled = max(0.0, (pos.x - sx) * cos_yaw
                               + (pos.y - sy) * sin_yaw)
                if traveled >= target:
                    self._stop_base()
                    return ('前进了 %.2f m(走满设定距离, 雷达未见目标, '
                            '可能仍离目标较远)' % traveled)
                s = self._scan_provider()
                if s is None or time.monotonic() - s[1] > SCAN_TIMEOUT_SEC:
                    self._stop_base()
                    return '逼近中止: 雷达数据超时(/scan_fe 在发布吗?)'
                front = self._front_distance(s[0])
                if front is not None and front <= self._approach_stop:
                    self._stop_base()
                    time.sleep(1.0)  # 停稳后再允许抓取, 防车体晃动影响手眼
                    self._approach_ok = True
                    self._last_approach_dist = traveled
                    return ('逼近完成: 前进 %.2f m, 前方目标距离 %.2f m, 已停车'
                            % (traveled, front))
                self._publish_vel(self._approach_speed)
                time.sleep(0.05)
            self._stop_base()
            return '逼近中止: 超时(%.0fs)' % max_time
        except WaitInterrupted:
            self._stop_base()  # 中断也必须先停车
            raise

    @staticmethod
    def _front_distance(scan):
        """前向 ±10° 窗口内最近有效距离, 无有效点返回 None."""
        half = math.radians(10.0)
        best = None
        angle = scan.angle_min
        for r in scan.ranges:
            if -half <= angle <= half and math.isfinite(r) \
                    and scan.range_min <= r <= scan.range_max:
                if best is None or r < best:
                    best = r
            angle += scan.angle_increment
        return best

    def _publish_vel(self, vx):
        twist = Twist()
        twist.linear.x = vx
        self._cmd_vel_pub.publish(twist)

    def _retreat(self, distance):
        """开环慢速直线后退 (复刻 driveDistance 后退分支: 无雷达早停,
        纯里程计测距). 返回 (成功与否, 距离或失败原因)."""
        if self._cmd_vel_pub is None or self._odom_provider is None:
            return False, '底盘接口未配置'
        start = None
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self._check_interrupt()
            o = self._odom_provider()
            if o is not None and time.monotonic() - o[1] <= ODOM_TIMEOUT_SEC:
                start = o[0].pose.pose
                break
            time.sleep(0.05)
        if start is None:
            return False, '等不到新鲜里程计'
        sx, sy = start.position.x, start.position.y
        q = start.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        max_time = distance / self._approach_speed + 5.0
        t0 = time.monotonic()
        try:
            while time.monotonic() - t0 < max_time:
                self._check_interrupt()
                o = self._odom_provider()
                if o is None or time.monotonic() - o[1] > ODOM_TIMEOUT_SEC:
                    self._stop_base()
                    return False, '里程计超时'
                pos = o[0].pose.pose.position
                # 后退方向投影取负
                traveled = max(0.0, -((pos.x - sx) * cos_yaw
                                      + (pos.y - sy) * sin_yaw))
                if traveled >= distance:
                    self._stop_base()
                    return True, traveled
                self._publish_vel(-self._approach_speed)
                time.sleep(0.05)
            self._stop_base()
            return False, '超时'
        except WaitInterrupted:
            self._stop_base()
            raise

    def _stop_base(self):
        for _ in range(3):
            self._publish_vel(0.0)
            time.sleep(0.02)

    def _call_arm(self, client, srv_name, action):
        """机械臂 Trigger 调用, 阻塞等真实结果 (elite 侧最长约 120s)."""
        res = self._call(client, Trigger.Request(), srv_name,
                         timeout=self._arm_timeout)
        if res is None:
            return ('%s失败: %s 无应答(机械臂侧 yolo_grasp.py 在运行吗?)'
                    % (action, srv_name))
        return ('%s成功: %s' % (action, res.message) if res.success
                else '%s失败: %s' % (action, res.message))

    def _wait_mission_terminal(self, desc):
        """等 /mission/status 到终态. 两阶段: 先确认任务开跑, 再等完成."""
        self._set_waiting(True)
        try:
            # 阶段1: 等状态离开空闲集合 (确认任务真的开始执行), 最多 15s
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline:
                self._check_interrupt()
                s = self._status_provider()
                if s is not None and s.state not in FREE_STATES:
                    break
                time.sleep(0.5)
            else:
                return ('%s: /mission/run 已受理但 15s 内未见任务开始, '
                        '请用 get_mission_status 确认实际状态' % desc)
            # 阶段2: 等终态
            deadline = time.monotonic() + self._nav_timeout
            while time.monotonic() < deadline:
                self._check_interrupt()
                s = self._status_provider()
                if s is not None and s.state in TERMINAL_STATES:
                    text = TERMINAL_STATES[s.state]
                    if s.message:
                        text += '(%s)' % s.message
                    return '%s: %s' % (desc, text)
                time.sleep(1.0)
            return ('%s: 等待超时(%.0fs), 任务可能仍在执行, '
                    '用 get_mission_status 查询' % (desc, self._nav_timeout))
        finally:
            self._set_waiting(False)

    def _check_interrupt(self):
        if self._interrupt_check():
            raise WaitInterrupted()

    # ---------- 内部工具 ----------

    def _load(self):
        """每次调用重新读文件, 面板新录的点位即时生效."""
        return load_waypoints(self._waypoints_file)

    def _to_mission_waypoint(self, wp, pure_nav=False):
        """yaml 点位 -> MissionWaypoint. pan 字段按面板约定塞预置位号.
        pure_nav=True 时剥掉拍照/action, 只导航(分步模式下动作由 LLM 逐步调用)."""
        mw = MissionWaypoint()
        mw.nav_pose.header.frame_id = 'map'
        mw.nav_pose.header.stamp = self._node.get_clock().now().to_msg()
        p = wp.pose
        mw.nav_pose.pose.position.x = float(p.get('x', 0.0))
        mw.nav_pose.pose.position.y = float(p.get('y', 0.0))
        mw.nav_pose.pose.position.z = float(p.get('z', 0.0))
        mw.nav_pose.pose.orientation.x = float(p.get('qx', 0.0))
        mw.nav_pose.pose.orientation.y = float(p.get('qy', 0.0))
        mw.nav_pose.pose.orientation.z = float(p.get('qz', 0.0))
        mw.nav_pose.pose.orientation.w = float(p.get('qw', 1.0))
        mw.pan = float(wp.ptz_preset)  # 海康预置位编号, 见 my_panel.cpp:322
        mw.tilt = 0.0
        mw.zoom = 0.0
        mw.do_capture = wp.capture and not pure_nav
        mw.extra_action = '' if pure_nav else wp.action
        return mw

    def _call(self, client, req, srv_name, timeout=None):
        """同步调用服务(在工作线程里, 主线程 rclpy.spin 不受影响).
        Humble 的 Client.call 不支持 timeout_sec, 用 call_async + Event 实现."""
        if not client.wait_for_service(timeout_sec=2.0):
            self._logger.warn('服务 %s 不在线' % srv_name)
            return None
        done = threading.Event()
        future = client.call_async(req)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout or self._timeout):
            client.remove_pending_request(future)
            self._logger.warn('服务 %s 调用超时' % srv_name)
            return None
        try:
            return future.result()
        except Exception as exc:  # noqa: BLE001
            self._logger.warn('服务 %s 调用异常: %s' % (srv_name, exc))
            return None

    def _audit(self, tool, args, result):
        line = json.dumps({
            'ts': time.strftime('%Y-%m-%d %H:%M:%S'),
            'tool': tool, 'args': args, 'result': result[:500],
        }, ensure_ascii=False)
        with self._audit_lock:
            try:
                with open(self._audit_file, 'a', encoding='utf-8') as f:
                    f.write(line + '\n')
            except OSError as exc:
                self._logger.warn('审计日志写入失败: %s' % exc)
