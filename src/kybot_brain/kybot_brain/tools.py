"""LLM 工具层: schema 定义 + 分发执行.

每个工具都是现有 ROS2 服务的纯客户端调用, 不新增任何服务端.
安全约束: 导航只接受 location.yaml 白名单内的点位名, LLM 不接触原始坐标.
"""

import json
import os
import threading
import time

from hk_camera.msg import MissionStatus, MissionWaypoint
from hk_camera.srv import CapturePicture, RunMission
from std_srvs.srv import Trigger

from .waypoints import find_waypoint, load_waypoints

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
                 service_timeout_sec=10.0, audit_file='~/.kybot_brain/audit.jsonl'):
        self._node = node
        self._logger = node.get_logger()
        self._waypoints_file = waypoints_file
        self._status_provider = status_provider  # callable -> MissionStatus | None
        self._timeout = service_timeout_sec
        self._audit_file = os.path.expanduser(audit_file)
        os.makedirs(os.path.dirname(self._audit_file), exist_ok=True)
        self._audit_lock = threading.Lock()

        self._run_cli = node.create_client(RunMission, '/mission/run')
        self._cancel_cli = node.create_client(Trigger, '/mission/cancel')
        self._capture_cli = node.create_client(CapturePicture, '/hk_camera/capture')

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
        return ('拍照成功: %s' % res.message if res.success
                else '拍照失败: %s' % res.message)

    # ---------- 内部工具 ----------

    def _load(self):
        """每次调用重新读文件, 面板新录的点位即时生效."""
        return load_waypoints(self._waypoints_file)

    def _to_mission_waypoint(self, wp):
        """yaml 点位 -> MissionWaypoint. pan 字段按面板约定塞预置位号."""
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
        mw.do_capture = wp.capture
        mw.extra_action = wp.action
        return mw

    def _call(self, client, req, srv_name):
        """同步调用服务(在工作线程里, 主线程 rclpy.spin 不受影响)."""
        if not client.wait_for_service(timeout_sec=2.0):
            self._logger.warn('服务 %s 不在线' % srv_name)
            return None
        try:
            return client.call(req, timeout_sec=self._timeout)
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
