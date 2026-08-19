"""Elite 打磨应用的任务级服务桥接。

底层 elite_polish_app 目前使用 Int32 话题：
  /elite_forceapp_cmd         3=视觉+打磨, 5=安全取消, 0=回 Home2
  /elite_forceapp_cmd_result  100=回 Home2, 104=成功, 204=失败, 205=取消

本节点把它封装成阻塞到终态的 Trigger 服务，供 mission_executor 和
kybot_brain 复用，并透传底层明确的成功、失败、取消结果。
"""

import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger


class PolishBridge(Node):
    """为 Elite 打磨状态机提供串行、可等待、可取消的服务接口。"""

    def __init__(self):
        super().__init__('elite_polish_bridge')
        self.declare_parameter('run_timeout_sec', 900.0)
        self.declare_parameter('home_timeout_sec', 90.0)
        self._run_timeout = float(self.get_parameter('run_timeout_sec').value)
        self._home_timeout = float(self.get_parameter('home_timeout_sec').value)

        self._cond = threading.Condition()
        self._busy = False
        self._cancel_requested = False
        self._last_result = None
        self._last_detail = ''
        self._state = 'IDLE'

        self._cmd_pub = self.create_publisher(
            Int32, '/elite_forceapp_cmd', 1)
        self._status_pub = self.create_publisher(
            String, '/elite_polish/status', 10)
        self.create_subscription(
            Int32, '/elite_forceapp_cmd_result', self._on_result, 10)
        self.create_subscription(
            String, '/elite_forceapp_result_detail', self._on_detail, 10)

        # 三个服务各用独立 callback group；MultiThreadedExecutor 允许 run
        # 阻塞等结果时，result 订阅和 cancel 服务仍能被处理。
        self._run_group = MutuallyExclusiveCallbackGroup()
        self._cancel_group = MutuallyExclusiveCallbackGroup()
        self._home_group = MutuallyExclusiveCallbackGroup()
        self.create_service(
            Trigger, '/elite_polish/run', self._run,
            callback_group=self._run_group)
        self.create_service(
            Trigger, '/elite_polish/cancel', self._cancel,
            callback_group=self._cancel_group)
        self.create_service(
            Trigger, '/elite_polish/home', self._home,
            callback_group=self._home_group)
        self.create_service(
            Trigger, '/elite_polish/get_status', self._get_status)

        self._publish_state('IDLE', '打磨桥接已就绪')
        self.get_logger().info(
            'Elite polish bridge ready: /elite_polish/run|cancel|home')

    def _publish_command(self, value):
        msg = Int32()
        msg.data = int(value)
        self._cmd_pub.publish(msg)

    def _publish_state(self, state, detail=''):
        with self._cond:
            self._state = state
        msg = String()
        msg.data = state + (': ' + detail if detail else '')
        self._status_pub.publish(msg)
        self.get_logger().info(msg.data)

    def _core_online(self):
        return (self.count_subscribers('/elite_forceapp_cmd') > 0
                and self.count_publishers('/elite_forceapp_cmd_result') > 0)

    def _on_result(self, msg):
        with self._cond:
            self._last_result = int(msg.data)
            self._cond.notify_all()
        self.get_logger().info('收到打磨底层结果码: %d' % msg.data)

    def _on_detail(self, msg):
        with self._cond:
            self._last_detail = msg.data.strip()
            self._cond.notify_all()

    def _begin(self):
        with self._cond:
            if self._busy:
                return False
            self._busy = True
            self._cancel_requested = False
            self._last_result = None
            self._last_detail = ''
            return True

    def _finish(self):
        with self._cond:
            self._busy = False
            self._cancel_requested = False
            self._cond.notify_all()

    def _wait_for(self, expected, timeout):
        deadline = time.monotonic() + timeout
        with self._cond:
            while self._last_result not in expected:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._cond.wait(timeout=min(0.5, remaining))
            return self._last_result

    def _wait_for_detail(self, timeout=0.5):
        """结果码和详细原因来自两个话题，容忍 DDS 跨话题到达乱序。"""
        deadline = time.monotonic() + timeout
        with self._cond:
            while not self._last_detail:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._cond.wait(timeout=remaining)
            return self._last_detail

    def _request_safe_cancel(self):
        """命令5由底层负责关打磨头、退出力控、安全退刀并回 Home2。"""
        self._publish_command(5)

    def _run(self, _request, response):
        if not self._core_online():
            response.success = False
            response.message = ('打磨底层未就绪: 请检查 ysURForceAppControl '
                                '和 /elite_forceapp_cmd_result')
            return response
        if not self._begin():
            response.success = False
            response.message = '已有打磨/回位任务正在执行'
            return response

        self._publish_state('RUNNING', '已下发命令3（深度视觉+自动打磨）')
        self._publish_command(3)
        try:
            result = self._wait_for({104, 204, 205}, self._run_timeout)
            detail = self._wait_for_detail() if result is not None else ''
            if result == 104:
                response.success = True
                response.message = detail or '打磨成功并已回 Home2'
                self._publish_state('COMPLETED', response.message)
            elif result == 205:
                response.success = False
                response.message = detail or '打磨已取消并已回 Home2'
                self._publish_state('CANCELED', response.message)
            elif result == 204:
                response.success = False
                response.message = detail or '打磨失败，已安全收尾并回 Home2'
                self._publish_state('FAILED', response.message)
            elif result is None:
                self._request_safe_cancel()
                # 超时本身是失败，但仍等待底层完成退刀和 Home2，避免任务线程
                # 提前释放后又接收下一条移动命令。
                # 104 也确认命令3已正常完成并回 Home2；它可能在
                # 超时边界与取消命令交错到达。
                safe_result = self._wait_for({104, 204, 205}, self._home_timeout)
                response.success = False
                response.message = (
                    '打磨等待超时，安全取消后已回 Home2'
                    if safe_result in (104, 204, 205)
                    else '打磨等待超时，已请求安全取消，但未确认回到 Home2')
                self._publish_state('FAILED', response.message)
            else:
                response.success = False
                response.message = '打磨返回未知结果码: %s' % result
                self._publish_state('FAILED', response.message)
            return response
        finally:
            self._finish()

    def _cancel(self, _request, response):
        with self._cond:
            if not self._busy:
                response.success = False
                response.message = '当前没有打磨任务'
                return response
            self._cancel_requested = True
        self._publish_state('CANCELING', '关闭打磨头、退出力控并回 Home2')
        self._request_safe_cancel()
        response.success = True
        response.message = '已下发打磨安全取消请求'
        return response

    def _home(self, _request, response):
        if not self._core_online():
            response.success = False
            response.message = '打磨底层未就绪'
            return response
        if not self._begin():
            response.success = False
            response.message = '已有打磨/回位任务正在执行'
            return response
        self._publish_state('HOMING', '关闭打磨头并回 Home2')
        try:
            self._publish_command(0)
            result = self._wait_for({100, 204, 205}, self._home_timeout)
            # 若底层原本正由手动命令执行打磨，命令0会把它安全取消并返回
            # 204/205；这两种结果同样确认机械臂已经到 Home2。
            response.success = result in (100, 204, 205)
            response.message = ('机械臂已回 Home2' if response.success
                                else '回 Home2 等待超时')
            self._publish_state(
                'IDLE' if response.success else 'FAILED', response.message)
            return response
        finally:
            self._finish()

    def _get_status(self, _request, response):
        with self._cond:
            response.success = not self._busy
            response.message = self._state
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PolishBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
