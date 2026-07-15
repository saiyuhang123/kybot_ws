#!/usr/bin/env python3
"""
cmd_vel_bridge.py — Nav2 /cmd_vel → 底盘 ctrl_cmd / io_cmd 桥接节点

将 geometry_msgs/Twist 转换为底盘运动控制命令:
  - linear.x  → ctrl_cmd_velocity (m/s) + 挡位 (D前进 / R后退 / N空挡)
  - angular.z → ctrl_cmd_steering (度), 通过 Ackermann 逆模型计算

安全机制:
  - 超时未收到 /cmd_vel 则自动急停
  - 节点退出时自动急停
  - 速度和转向角限幅

用法:
  ros2 run yhs_can_control cmd_vel_bridge.py

参数 (可通过 --ros-args -p 覆盖):
  wheel_base:        轴距 (m), 默认 0.6
  max_steering:      最大转向角 (度), 默认 45.0
  max_velocity:      最大速度 (m/s), 默认 2.0
  cmd_vel_timeout:   超时停车时间 (秒), 默认 0.5
  velocity_deadband: 速度死区 (m/s), 默认 0.01
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from yhs_can_interfaces.msg import CtrlCmd, IoCmd

# 说明书挡位定义
GEAR_DISABLE = 0
GEAR_P = 1
GEAR_R = 2
GEAR_N = 3
GEAR_D = 4

GEAR_NAME = {
    GEAR_DISABLE: "Disable",
    GEAR_P: "P",
    GEAR_R: "R",
    GEAR_N: "N",
    GEAR_D: "D",
}


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # --- 参数声明 ---
        self.declare_parameter('wheel_base', 0.6)
        self.declare_parameter('max_steering', 45.0)
        self.declare_parameter('max_velocity', 2.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('velocity_deadband', 0.01)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_steering = self.get_parameter('max_steering').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.velocity_deadband = self.get_parameter('velocity_deadband').value

        # --- 内部状态 ---
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_received = False
        self.enabled = False
        self._shutdown_done = False

        # --- 发布者 (queue=1, 与现有脚本一致) ---
        self.ctrl_pub = self.create_publisher(CtrlCmd, 'ctrl_cmd', 1)
        self.io_pub = self.create_publisher(IoCmd, 'io_cmd', 1)

        # --- 订阅者 ---
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # --- 定时器: 100Hz 持续发布 ---
        self.timer = self.create_timer(0.01, self.timer_callback)

        # --- 关闭钩子 ---
        self._shutdown_registered = False
        self._register_shutdown_hook()

        self.get_logger().info(
            f'cmd_vel_bridge started: wheel_base={self.wheel_base:.2f}m, '
            f'max_vel={self.max_velocity:.1f}m/s, max_steer={self.max_steering:.0f}°, '
            f'timeout={self.cmd_vel_timeout:.1f}s'
        )

    def _register_shutdown_hook(self):
        """注册一个一次性定时器用于在 shutdown 时急停。

        rclpy 没有直接的 on_shutdown 回调，但可以通过
        Node.destroy_node() 被调用时清理。更可靠的方式是
        在 finally 块中处理，见 main()。
        """
        pass

    # ---- 订阅回调 ----

    def cmd_vel_callback(self, msg: Twist):
        """接收 Nav2 的 /cmd_vel 消息，仅缓存最新值和时间戳。"""
        self.target_linear_x = msg.linear.x
        self.target_angular_z = msg.angular.z
        self.last_cmd_vel_time = self.get_clock().now()

        if not self.cmd_vel_received:
            self.get_logger().info('收到首个 /cmd_vel 消息')

        # 调试打印：每 50 条打印一次原始输入（约 0.5~1s 一次）
        if not hasattr(self, '_debug_cnt'):
            self._debug_cnt = 0
        self._debug_cnt += 1
        if self._debug_cnt % 1 == 0:
            self.get_logger().info(
                f'[调试 /cmd_vel] '
                f'线速度 x={msg.linear.x:.3f} m/s, '
                f'y={msg.linear.y:.3f}, z={msg.linear.z:.3f}, '
                f'角速度 x={msg.angular.x:.3f}, y={msg.angular.y:.3f}, '
                f'z={msg.angular.z:.3f} rad/s'
            )

        self.cmd_vel_received = True

    # ---- 定时器回调 (100Hz) ----

    def timer_callback(self):
        """主循环：检查超时、计算命令、发布。"""
        now = self.get_clock().now()
        dt = (now - self.last_cmd_vel_time).nanoseconds * 1e-9

        # --- 超时检查 ---
        if not self.cmd_vel_received:
            # 还没收到过 cmd_vel，不发布任何东西
            return

        if dt > self.cmd_vel_timeout:
            if self.enabled:
                self.get_logger().warn(
                    f'cmd_vel 超时 ({dt:.2f}s > {self.cmd_vel_timeout:.1f}s)，急停！'
                )
            self._emergency_stop()
            return

        # --- 有效的 cmd_vel ---
        linear_x = self.target_linear_x
        angular_z = self.target_angular_z

        # 速度限幅
        linear_x = max(-self.max_velocity, min(self.max_velocity, linear_x))

        # 挡位和正向速度
        if linear_x > self.velocity_deadband:
            gear = GEAR_D
            velocity = linear_x
        elif linear_x < -self.velocity_deadband:
            gear = GEAR_R
            velocity = -linear_x  # 正向值，后退靠 R 挡
        else:
            gear = GEAR_N
            velocity = 0.0

        # 转向角: Ackermann 逆模型 (用 atan 而非 atan2，
        # 因为 linear_x 可能为负，atan2 会把结果偏移到 ±180° 附近)
        if abs(linear_x) > self.velocity_deadband:
            steering_rad = math.atan(self.wheel_base * angular_z / linear_x)
        else:
            steering_rad = 0.0
        steering_deg = math.degrees(steering_rad)
        steering_deg = max(-self.max_steering,
                           min(self.max_steering, steering_deg))

        # 首次使能日志
        if not self.enabled:
            self.get_logger().info(
                f'底盘使能: gear={GEAR_NAME.get(gear, "?")}, '
                f'vel={velocity:.2f}m/s, steer={steering_deg:.1f}°'
            )
            self.enabled = True

        # 发布
        self._publish_ctrl(gear, velocity, steering_deg)
        self._publish_io(enable=True)

    # ---- 发布方法 ----

    def _publish_ctrl(self, gear: int, velocity: float, steering: float):
        """发布 CtrlCmd 消息。"""
        msg = CtrlCmd()
        msg.ctrl_cmd_gear = gear
        msg.ctrl_cmd_velocity = velocity
        msg.ctrl_cmd_steering = steering
        self.ctrl_pub.publish(msg)

    def _publish_io(self, enable: bool):
        """发布 IoCmd 消息，仅控制使能，灯光全部关闭。"""
        msg = IoCmd()
        msg.io_cmd_enable = enable
        msg.io_cmd_lower_beam_headlamp = False
        msg.io_cmd_upper_beam_headlamp = False
        msg.io_cmd_turn_lamp = 0
        msg.io_cmd_braking_lamp = False
        msg.io_cmd_clearance_lamp = False
        msg.io_cmd_fog_lamp = False
        msg.io_cmd_speaker = False
        msg.io_cmd_dis_charge = False
        self.io_pub.publish(msg)

    def _emergency_stop(self):
        """急停：速度归零、挡位 N、取消使能。"""
        self.enabled = False
        self.cmd_vel_received = False
        self._publish_ctrl(GEAR_N, 0.0, 0.0)
        self._publish_io(enable=False)

    def emergency_stop(self):
        """外部可调用的急停接口。"""
        self._emergency_stop()
        self.get_logger().info('cmd_vel_bridge: 急停完成')


def main():
    rclpy.init()
    node = CmdVelBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 确保退出前执行急停
        if not node._shutdown_done:
            node.emergency_stop()
            node._shutdown_done = True
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
