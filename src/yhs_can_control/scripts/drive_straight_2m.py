#!/usr/bin/env python3
"""
直线行驶 2m 测试脚本
用法:
    ros2 run yhs_can_control drive_straight_2m.py

小车笔直向前行驶 2m 后自动停止，用于测试里程计精度。
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from yhs_can_interfaces.msg import CtrlCmd, IoCmd

# 挡位定义
GEAR_D = 4
GEAR_N = 3

# 参数
TARGET_DISTANCE = 4.0       # 目标距离 (m)
CRUISE_SPEED = 0.1          # 巡航速度 (m/s)
PUBLISH_RATE = 0.01         # 100Hz


class DriveStraightTest(Node):
    def __init__(self):
        super().__init__('drive_straight_test')

        self.ctrl_pub = self.create_publisher(CtrlCmd, 'ctrl_cmd', 1)
        self.io_pub = self.create_publisher(IoCmd, 'io_cmd', 1)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        self.start_x = None
        self.start_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_vel = 0.0
        self.distance = 0.0
        self.got_first_odom = False
        self.odom_msg_count = 0

        self.velocity = 0.0
        self.state = 'WAIT_ODOM'

        self.enable_count = 0

        # 主控定时器 100Hz
        self.timer = self.create_timer(PUBLISH_RATE, self.control_loop)

        self.get_logger().info("=" * 50)
        self.get_logger().info(f"直线行驶测试: 目标 {TARGET_DISTANCE}m, 恒速 {CRUISE_SPEED}m/s")
        self.get_logger().info("等待 /odom 数据...")

    def odom_callback(self, msg: Odometry):
        self.odom_msg_count += 1
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_vel = msg.twist.twist.linear.x

        if not self.got_first_odom:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.got_first_odom = True
            self.get_logger().info(
                f"记录起点: x={self.start_x:.3f}, y={self.start_y:.3f}")
        else:
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            self.distance = math.sqrt(dx * dx + dy * dy)

    def publish_ctrl(self):
        msg = CtrlCmd()
        msg.ctrl_cmd_gear = GEAR_D if self.state not in ('WAIT_ODOM', 'DONE') else GEAR_N
        msg.ctrl_cmd_velocity = self.velocity
        msg.ctrl_cmd_steering = 0.0
        self.ctrl_pub.publish(msg)

    def publish_io(self, enable=True):
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

    def control_loop(self):
        """主控制循环, 100Hz"""

        if self.state == 'WAIT_ODOM':
            if self.got_first_odom:
                self.state = 'ENABLE'
                self.get_logger().info("使能底盘...")
            else:
                self.publish_io(enable=False)
                self.publish_ctrl()
                return

        if self.state == 'ENABLE':
            self.publish_io(enable=True)
            self.publish_ctrl()
            self.enable_count += 1
            if self.enable_count >= 100:  # 使能保持 1s
                self.state = 'CRUISE'
                self.velocity = CRUISE_SPEED
                self.get_logger().info(f"起步! 恒速 {CRUISE_SPEED}m/s, D挡(gear=4)")

        elif self.state == 'CRUISE':
            self.publish_io(enable=True)
            self.publish_ctrl()
            if self.distance >= TARGET_DISTANCE:
                self.state = 'DONE'
                self.velocity = 0.0
                self.get_logger().info("=" * 50)
                self.get_logger().info("  测试完成!")
                self.get_logger().info(f"  odom 行驶距离: {self.distance:.3f} m")
                self.get_logger().info(f"  实际目标距离: {TARGET_DISTANCE:.3f} m")
                self.get_logger().info(f"  误差: {self.distance - TARGET_DISTANCE:.3f} m"
                                       f" ({(self.distance - TARGET_DISTANCE) / TARGET_DISTANCE * 100:.1f}%)")
                self.get_logger().info("=" * 50)
                self.get_logger().info("按 Ctrl+C 退出")

        elif self.state == 'DONE':
            self.velocity = 0.0
            self.publish_io(enable=False)
            self.publish_ctrl()

    def print_progress(self):
        """0.5s 打印状态"""
        if self.state == 'WAIT_ODOM':
            self.get_logger().info(
                f"  [状态:等待odom] odom收到:{self.got_first_odom} odom计数:{self.odom_msg_count}")
        elif self.state == 'ENABLE':
            self.get_logger().info(
                f"  [状态:使能中] 倒计时:{100 - self.enable_count}ticks vel={self.velocity:.2f}")
        elif self.state == 'CRUISE':
            pct = self.distance / TARGET_DISTANCE * 100
            bar = '#' * int(pct / 5) + '-' * (20 - int(pct / 5))
            self.get_logger().info(
                f"  [{bar}] {self.distance:.2f}m / {TARGET_DISTANCE}m "
                f"({pct:.0f}%)  vel={self.current_vel:.2f}m/s")
        elif self.state == 'DONE':
            pass


def main():
    rclpy.init()
    node = DriveStraightTest()

    progress_timer = node.create_timer(0.5, node.print_progress)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.state != 'DONE':
            node.get_logger().info("手动中断，紧急停止...")
            node.velocity = 0.0
            node.publish_io(enable=False)
            node.publish_ctrl()
        print()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
