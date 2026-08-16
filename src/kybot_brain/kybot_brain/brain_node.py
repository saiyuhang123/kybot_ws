"""kybot_brain 主节点: 文本指令 -> DeepSeek function calling -> 现有调度服务.

输入: 订阅 /brain_text (std_msgs/String)
输出: 发布 /brain_reply (回复) 和 /tts_text (为语音播报预留)
执行: 复用 /mission/run, /mission/cancel, /mission/status, /hk_camera/capture,
      /yolo_grasp/* (elite 侧)

线程模型: 主线程 rclpy.spin 处理订阅; LLM 循环 + 服务调用在独立工作线程。
执行范式: 分步同步——navigate_and_wait 等工具阻塞等真实结果(导航靠等
/mission/status 终态), LLM 逐步决策; 等待期间收到新指令(如"取消")会
中断等待并优先处理新指令。
"""

import json
import os
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from hk_camera.msg import MissionStatus
from .llm_client import LLMClient, LLMError
from .tools import STATE_NAMES, TOOL_SCHEMAS, ToolExecutor, WaitInterrupted
from .waypoints import load_waypoints

MAX_LLM_ROUNDS = 8  # 与 agent_bridge.cpp 一致
TERMINAL_STATES = {
    MissionStatus.STATE_COMPLETED: '任务已完成',
    MissionStatus.STATE_FAILED: '任务失败',
    MissionStatus.STATE_CANCELED: '任务已取消',
}

SYSTEM_PROMPT_TMPL = """你是巡检机器人 KYBOT 的调度助手, 通过调用工具控制机器人。
当前点位白名单(名称 — 该点 yaml 预配置的动作):
{waypoint_lines}

工作方式: 分步同步执行。
1. 把用户的复合指令拆成有序步骤, 一步一步调用工具。每个工具都返回真实执行结果,
   必须看上一步结果再决定下一步: 失败就停止后续步骤并向用户说明原因, 不要硬往下走。
2. 导航用 navigate_and_wait: 纯导航, 会等到真正到达/失败才返回(不触发点位yaml
   预配置的拍照/抓取动作, 那些由你按需单独调用: grasp 抓取、place 放置、
   capture_photo 拍照、arm_home 收臂、arm_ready 预备)。
   抓取前必须先调 approach 慢速逼近到目标跟前(雷达自动停车), 再调 grasp;
   grasp 结束后会自动后退到导航位置, 不用你额外处理。
3. 只有"纯多点跑圈巡检、途中不需要看结果做决定"时才用 run_route 一次下发;
   单点快速前往(不等结果)才用 goto_waypoint。
4. "回来/返回"的目标也必须是白名单里的点位名; 白名单中没有名字含
   "出发点/起点/家/原位"的点位时, "回来"一律先反问用户回到哪里, 严禁自己猜。
5. 名字严格用白名单; 不确定先调 list_waypoints; 名单里没有的说去不了, 不许编造。
   用户要求的动作超出点位配置/工具能力时, 如实说明做不到, 不许假装做了。
6. 用户问进度调 get_mission_status; 用户说停/取消调 cancel_mission; 只有用户明确说"停/取消/别去了"才允许取消。
7. 收到听不清、无意义或闲聊性质的话(如唤醒词、误识别)时, 不要调用任何会改变
   机器人状态的工具(导航/取消/抓取等), 用一句话询问或回应即可。
8. 工具返回互相矛盾(如取消说没任务、状态却说运行中)时, 不要反复重试同一工具,
   如实告诉用户系统状态异常、建议检查底层, 然后结束本轮。
7. 全部用简洁的中文口语回答, 一两句话说完。"""


class BrainNode(Node):
    """大模型调度节点."""

    def __init__(self):
        super().__init__('brain_node')
        self._declare_params()
        self._load_params()

        # LLM 客户端
        if not self._api_key:
            self.get_logger().warn(
                '未配置 api_key: 请 export DEEPSEEK_API_KEY 或设置参数 api_key。'
                '节点照常运行, 但 LLM 调用会失败。')
        self._llm = LLMClient(self._api_key, self._api_base, self._model,
                              self._llm_timeout, self.get_logger())

        # 会话状态 (worker 线程与 status 回调都会碰, 加锁)
        self._lock = threading.Lock()
        self._messages = [{'role': 'system',
                           'content': self._build_system_prompt()}]
        self._last_status = None
        self._waiting = 0  # >0 表示有同步工具在等任务终态, 期间不重复播报事件
        self._interrupt = threading.Event()  # 新指令到达时置位, 中断等待
        self._last_odom = None       # (Odometry, 接收时刻 monotonic)
        self._last_scan = None       # (LaserScan, 接收时刻 monotonic)

        # /cmd_vel 发布器: approach 工具慢速逼近用 (与 executor 同一入口)
        cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 工具执行器
        self._tools = ToolExecutor(
            self, self._waypoints_file, self._get_status,
            service_timeout_sec=self._service_timeout,
            nav_timeout_sec=self._nav_timeout,
            arm_timeout_sec=self._arm_timeout,
            interrupt_check=self._interrupt.is_set,
            set_waiting=self._set_waiting,
            odom_provider=self._get_odom,
            scan_provider=self._get_scan,
            cmd_vel_pub=cmd_vel_pub,
            approach_speed=self._approach_speed,
            approach_stop_distance=self._approach_stop_distance,
            approach_max_distance=self._approach_max_distance,
            audit_file=self._audit_file)

        # ROS 接线
        self._reply_pub = self.create_publisher(String, '/brain_reply', 10)
        self._tts_pub = self.create_publisher(String, '/tts_text', 10)
        self.create_subscription(String, '/brain_text', self._on_text, 10)
        self.create_subscription(MissionStatus, '/mission/status',
                                 self._on_status, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(LaserScan, '/scan_fe', self._on_scan, 10)

        # LLM 工作线程: 回调只入队, 避免阻塞 executor
        self._queue = queue.Queue()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            'kybot_brain 就绪: 模型=%s, 点位文件=%s, 发 /brain_text 即可对话'
            % (self._model, self._waypoints_file))

    # ---------- 参数 ----------

    def _declare_params(self):
        self.declare_parameter('api_base', 'https://api.deepseek.com')
        self.declare_parameter('model', 'deepseek-v4-flash')
        self.declare_parameter('api_key', '')  # 空则用 DEEPSEEK_API_KEY
        self.declare_parameter('waypoints_file',
                               '/home/nvidia/kybot_ws/location/location.yaml')
        self.declare_parameter('llm_timeout_sec', 60.0)
        self.declare_parameter('service_timeout_sec', 10.0)
        self.declare_parameter('nav_timeout_sec', 300.0)   # 导航等终态上限
        self.declare_parameter('arm_timeout_sec', 130.0)   # 机械臂调用上限
        # approach 逼近参数, 与 mission_executor 的 driveDistance 一致
        self.declare_parameter('approach_speed', 0.1)          # m/s
        self.declare_parameter('approach_stop_distance', 0.7)  # 前方障碍早停 (m)
        self.declare_parameter('approach_max_distance', 1.5)   # 单次逼近上限 (m)
        self.declare_parameter('history_max', 20)
        self.declare_parameter('audit_file', '~/.kybot_brain/audit.jsonl')

    def _load_params(self):
        p = self.get_parameter
        self._api_base = p('api_base').value
        self._model = p('model').value
        self._api_key = (p('api_key').value
                         or os.getenv('DEEPSEEK_API_KEY', '')
                         or os.getenv('DASHSCOPE_API_KEY', ''))
        self._waypoints_file = p('waypoints_file').value
        self._llm_timeout = p('llm_timeout_sec').value
        self._service_timeout = p('service_timeout_sec').value
        self._nav_timeout = p('nav_timeout_sec').value
        self._arm_timeout = p('arm_timeout_sec').value
        self._approach_speed = p('approach_speed').value
        self._approach_stop_distance = p('approach_stop_distance').value
        self._approach_max_distance = p('approach_max_distance').value
        self._history_max = p('history_max').value
        self._audit_file = p('audit_file').value

    def _build_system_prompt(self):
        """每次对话前刷新点位名单, 面板新录的点位即时生效."""
        try:
            wps = load_waypoints(self._waypoints_file)
            lines = '\n'.join('  - ' + wp.brief() for wp in wps) or '  (空)'
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('点位文件读取失败: %s' % exc)
            lines = '  (点位文件读取失败)'
        return SYSTEM_PROMPT_TMPL.format(waypoint_lines=lines)

    # ---------- ROS 回调 ----------

    # 只有明确的控制/转向意图才打断执行中的等待(导航/逼近);
    # 查状态、闲聊等新指令排队处理, 不打断 (语音场景误识别多, 全部打断会误杀任务)
    INTERRUPT_KEYWORDS = ('停', '取消', '别', '算了', '回')

    def _on_text(self, msg):
        text = msg.data.strip()
        if text:
            self.get_logger().info('收到指令: %s' % text)
            if any(k in text for k in self.INTERRUPT_KEYWORDS):
                self._interrupt.set()
            self._queue.put(text)

    def _on_status(self, msg):
        """缓存状态 + 检测终态事件, 通知用户并回填会话."""
        with self._lock:
            prev = self._last_status
            self._last_status = msg
            if prev is None or prev.state == msg.state:
                return
            if msg.state not in TERMINAL_STATES:
                return
            # 与状态更新同一把锁里读 _waiting, 否则等待中的工作线程
            # 可能在两把锁之间把 _waiting 清零, 造成重复播报
            waiting = self._waiting > 0
            text = '%s%s' % (TERMINAL_STATES[msg.state],
                             (': ' + msg.message) if msg.message else '')
            self._messages.append(
                {'role': 'system', 'content': '[任务事件] ' + text})
            self._trim_history_locked()
        self.get_logger().info('任务终态: %s' % text)
        # 有同步工具在等终态时, 结果会经工具返回给 LLM, 不重复播报
        if not waiting:
            self._announce(text)

    def _get_status(self):
        with self._lock:
            return self._last_status

    def _on_odom(self, msg):
        with self._lock:
            self._last_odom = (msg, time.monotonic())

    def _on_scan(self, msg):
        with self._lock:
            self._last_scan = (msg, time.monotonic())

    def _get_odom(self):
        with self._lock:
            return self._last_odom

    def _get_scan(self):
        with self._lock:
            return self._last_scan

    def _set_waiting(self, active):
        with self._lock:
            self._waiting += 1 if active else -1

    # ---------- LLM 循环 (工作线程) ----------

    def _worker_loop(self):
        while True:
            text = self._queue.get()
            if text is None:  # 关停信号
                return
            try:
                self._handle(text)
            except Exception as exc:  # noqa: BLE001 - worker 不许死
                self.get_logger().error('处理指令异常: %s' % exc)

    def _handle(self, text):
        self._interrupt.clear()  # 新回合开始, 清掉自己的中断标记
        with self._lock:
            self._messages[0] = {'role': 'system',
                                 'content': self._build_system_prompt()}
            self._messages.append({'role': 'user', 'content': text})
            self._trim_history_locked()

        for _ in range(MAX_LLM_ROUNDS):
            with self._lock:
                snapshot = self._ordered_snapshot_locked()
            try:
                message = self._llm.chat(snapshot, TOOL_SCHEMAS)
            except LLMError as exc:
                self._announce('大模型调用失败: %s' % exc)
                return

            tool_calls = message.get('tool_calls') or []
            with self._lock:
                self._messages.append(message)  # 原样入历史(含 tool_calls)
            if not tool_calls:
                reply = (message.get('content') or '').strip()
                if reply:
                    self._announce(reply)
                return

            for i, call in enumerate(tool_calls):
                func = call.get('function', {})
                name = func.get('name', '')
                try:
                    args = json.loads(func.get('arguments') or '{}')
                except ValueError:
                    args = {}
                try:
                    result = self._tools.execute(name, args)
                except WaitInterrupted:
                    # 等待被新指令打断: 补齐未应答的 tool_call 保持历史合法,
                    # 然后结束本回合, 新指令由 worker 循环取出处理
                    with self._lock:
                        for rest in tool_calls[i:]:
                            self._messages.append({
                                'role': 'tool',
                                'tool_call_id': rest.get('id', ''),
                                'name': rest.get('function', {}).get('name', ''),
                                'content': '(该步骤被用户新指令中断, 未完成, 不要当作已成功。'
                                           '若是导航步骤, 任务可能仍在后台执行, 请先用 '
                                           'get_mission_status 确认状态再决定下一步)',
                            })
                    self.get_logger().info('等待被新指令中断, 转交新指令')
                    return
                with self._lock:
                    self._messages.append({
                        'role': 'tool',
                        'tool_call_id': call.get('id', ''),
                        'name': name,
                        'content': result,
                    })
        self._announce('处理超时: 工具调用轮数过多, 请换个说法再试')

    def _ordered_snapshot_locked(self):
        """生成发送给 LLM 的历史快照 (调用方须已持锁)。

        DeepSeek 比 Qwen 更严格: 带 tool_calls 的 assistant 消息后面必须
        紧跟对应 tool 消息。状态回调/历史裁剪可能把 system 事件插到两者
        之间, 直接发送会被 DeepSeek 以 400 拒绝。这里把夹在工具调用和
        工具结果之间的非 tool 消息挪到该组工具结果之后, 保证 API 合法。
        """
        ordered = []
        deferred = []
        pending_ids = []
        tool_seq_start = None
        for msg in self._messages:
            if pending_ids:
                if (msg.get('role') == 'tool'
                        and msg.get('tool_call_id') in pending_ids):
                    ordered.append(msg)
                    pending_ids.remove(msg['tool_call_id'])
                    if not pending_ids:
                        ordered.extend(deferred)
                        deferred = []
                else:
                    deferred.append(msg)
                continue
            if msg.get('role') == 'tool':
                continue  # 孤立 tool 消息: 已被历史裁剪丢了 assistant, 丢弃
            if (msg.get('role') == 'assistant' and msg.get('tool_calls')):
                ordered.append(msg)
                pending_ids = [tc.get('id') for tc in msg['tool_calls']
                               if tc.get('id')]
                if pending_ids:
                    tool_seq_start = len(ordered) - 1
                continue
            ordered.append(msg)
        if pending_ids:
            # 不完整的 tool_calls 序列: 丢弃这条 assistant 及其零散 tool 回执
            del ordered[tool_seq_start:]
            ordered.extend(deferred)
        return ordered

    def _trim_history_locked(self):
        """保留 system + 最近 N 条 (调用方须已持锁)."""
        if len(self._messages) > self._history_max + 1:
            self._messages = ([self._messages[0]]
                              + self._messages[-self._history_max:])

    # ---------- 输出 ----------

    def _announce(self, text):
        """打印 + /brain_reply + /tts_text 三路输出."""
        self.get_logger().info('回复: %s' % text)
        msg = String()
        msg.data = text
        self._reply_pub.publish(msg)
        self._tts_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._queue.put(None)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
