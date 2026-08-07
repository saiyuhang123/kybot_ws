"""kybot_brain 主节点: 文本指令 -> Qwen function calling -> 现有调度服务.

输入: 订阅 /brain_text (std_msgs/String)
输出: 发布 /brain_reply (回复) 和 /tts_text (为语音播报预留)
执行: 复用 /mission/run, /mission/cancel, /mission/status, /hk_camera/capture

线程模型: 主线程 rclpy.spin 处理订阅; LLM 循环 + 服务调用在独立工作线程,
长任务(导航)异步, 靠 /mission/status 终态事件回填会话.
"""

import json
import os
import queue
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from hk_camera.msg import MissionStatus
from .llm_client import LLMClient, LLMError
from .tools import STATE_NAMES, TOOL_SCHEMAS, ToolExecutor
from .waypoints import load_waypoints

MAX_LLM_ROUNDS = 8  # 与 agent_bridge.cpp 一致
TERMINAL_STATES = {
    MissionStatus.STATE_COMPLETED: '任务已完成',
    MissionStatus.STATE_FAILED: '任务失败',
    MissionStatus.STATE_CANCELED: '任务已取消',
}

SYSTEM_PROMPT_TMPL = """你是巡检机器人 KYBOT 的调度助手, 通过调用工具控制机器人。
当前点位白名单(名称 — 到达后动作):
{waypoint_lines}

规则:
1. 去单个点位调 goto_waypoint, name 必须严格使用白名单里的名称;
   不确定名字时先调 list_waypoints 拿最新名单; 名单里没有就如实说去不了, 不许编造。
2. 一句话要求依次去多个点位(含"去X之后再回来"的往返)时, 调 run_route 一次性下发,
   不要拆成多次 goto_waypoint; "回来/返回"的目标也必须是白名单里的点位名。
   白名单中没有名字含"出发点/起点/家/原位"的点位时, "回来"一律先反问用户
   回到哪个点位, 严禁拿刚去过的点位充当返程点。
3. 任务是异步的: 工具返回"已启动"就告诉用户已开始执行, 不要反复调用;
   之后收到"[任务事件]"消息时, 用一句话转告用户结果。
4. 用户问进度就调 get_mission_status; 用户说停/取消就调 cancel_mission。
5. 全部用简洁的中文口语回答, 一两句话说完。"""


class BrainNode(Node):
    """大模型调度节点."""

    def __init__(self):
        super().__init__('brain_node')
        self._declare_params()
        self._load_params()

        # LLM 客户端
        if not self._api_key:
            self.get_logger().warn(
                '未配置 api_key: 请 export DASHSCOPE_API_KEY 或设置参数 api_key。'
                '节点照常运行, 但 LLM 调用会失败。')
        self._llm = LLMClient(self._api_key, self._api_base, self._model,
                              self._llm_timeout, self.get_logger())

        # 会话状态 (worker 线程与 status 回调都会碰, 加锁)
        self._lock = threading.Lock()
        self._messages = [{'role': 'system',
                           'content': self._build_system_prompt()}]
        self._last_status = None

        # 工具执行器
        self._tools = ToolExecutor(
            self, self._waypoints_file, self._get_status,
            service_timeout_sec=self._service_timeout,
            audit_file=self._audit_file)

        # ROS 接线
        self._reply_pub = self.create_publisher(String, '/brain_reply', 10)
        self._tts_pub = self.create_publisher(String, '/tts_text', 10)
        self.create_subscription(String, '/brain_text', self._on_text, 10)
        self.create_subscription(MissionStatus, '/mission/status',
                                 self._on_status, 10)

        # LLM 工作线程: 回调只入队, 避免阻塞 executor
        self._queue = queue.Queue()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            'kybot_brain 就绪: 模型=%s, 点位文件=%s, 发 /brain_text 即可对话'
            % (self._model, self._waypoints_file))

    # ---------- 参数 ----------

    def _declare_params(self):
        self.declare_parameter('api_base',
                               'https://dashscope.aliyuncs.com/compatible-mode/v1')
        self.declare_parameter('model', 'qwen-plus')
        self.declare_parameter('api_key', '')  # 空则用 DASHSCOPE_API_KEY
        self.declare_parameter('waypoints_file',
                               '/home/nvidia/kybot_ws/location/location.yaml')
        self.declare_parameter('llm_timeout_sec', 60.0)
        self.declare_parameter('service_timeout_sec', 10.0)
        self.declare_parameter('history_max', 20)
        self.declare_parameter('audit_file', '~/.kybot_brain/audit.jsonl')

    def _load_params(self):
        p = self.get_parameter
        self._api_base = p('api_base').value
        self._model = p('model').value
        self._api_key = p('api_key').value or os.getenv('DASHSCOPE_API_KEY', '')
        self._waypoints_file = p('waypoints_file').value
        self._llm_timeout = p('llm_timeout_sec').value
        self._service_timeout = p('service_timeout_sec').value
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

    def _on_text(self, msg):
        text = msg.data.strip()
        if text:
            self.get_logger().info('收到指令: %s' % text)
            self._queue.put(text)

    def _on_status(self, msg):
        """缓存状态 + 检测终态事件, 通知用户并回填会话."""
        with self._lock:
            prev = self._last_status
            self._last_status = msg
        if prev is None or prev.state == msg.state:
            return
        if msg.state in TERMINAL_STATES:
            text = '%s%s' % (TERMINAL_STATES[msg.state],
                             (': ' + msg.message) if msg.message else '')
            self.get_logger().info('任务终态: %s' % text)
            self._announce(text)
            with self._lock:
                self._messages.append(
                    {'role': 'system', 'content': '[任务事件] ' + text})
                self._trim_history_locked()

    def _get_status(self):
        with self._lock:
            return self._last_status

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
        with self._lock:
            self._messages[0] = {'role': 'system',
                                 'content': self._build_system_prompt()}
            self._messages.append({'role': 'user', 'content': text})
            self._trim_history_locked()

        for _ in range(MAX_LLM_ROUNDS):
            with self._lock:
                snapshot = list(self._messages)
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

            for call in tool_calls:
                func = call.get('function', {})
                name = func.get('name', '')
                try:
                    args = json.loads(func.get('arguments') or '{}')
                except ValueError:
                    args = {}
                result = self._tools.execute(name, args)
                with self._lock:
                    self._messages.append({
                        'role': 'tool',
                        'tool_call_id': call.get('id', ''),
                        'name': name,
                        'content': result,
                    })
        self._announce('处理超时: 工具调用轮数过多, 请换个说法再试')

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
