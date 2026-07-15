#!/usr/bin/env python3
"""
里程计测试工具
用法:
    ros2 run yhs_can_control test_odom.py

订阅 /odom 话题，实时显示位置、速度、航向等信息。
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomTester(Node):
    def __init__(self):
        super().__init__('odom_tester')

        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        # 统计
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.total_distance = 0.0
        self.max_speed = 0.0
        self.msg_count = 0
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_time = self.get_clock().now()
        self.start_time = self.get_clock().now()

        # 1Hz 刷新显示，不刷屏
        self.display_timer = self.create_timer(1.0, self.display_callback)

        self.get_logger().info("里程计测试工具已启动，监听 /odom ...")
        self.print_header()

    def odom_callback(self, msg: Odometry):
        self.msg_count += 1

        # 位姿
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # 偏航角 (从四元数提取)
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

        # 速度
        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z

        # 累计里程
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt > 0 and dt < 1.0:
            dx = self.x - self.last_x
            dy = self.y - self.last_y
            self.total_distance += math.sqrt(dx * dx + dy * dy)

        # 最高速度
        abs_vel = abs(self.linear_vel)
        if abs_vel > self.max_speed:
            self.max_speed = abs_vel

        self.last_x = self.x
        self.last_y = self.y
        self.last_time = now

    def print_header(self):
        print()
        print(" " + "=" * 72)
        print(" {:>6s} {:>10s} {:>10s} {:>8s} {:>9s} {:>9s} {:>8s}".format(
            "msg#", "x(m)", "y(m)", "yaw(°)", "vel(m/s)", "ang(°/s)", "dist(m)"))
        print(" " + "-" * 72)

    def display_callback(self):
        if self.msg_count == 0:
            print("  (等待 /odom 数据...)")
            return

        yaw_deg = math.degrees(self.yaw)
        ang_deg = math.degrees(self.angular_vel)
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        print(" {:>6d} {:>10.3f} {:>10.3f} {:>8.1f} {:>9.3f} {:>9.1f} {:>8.3f}".format(
            self.msg_count, self.x, self.y, yaw_deg,
            self.linear_vel, ang_deg, self.total_distance))

        # 每10秒打印统计摘要
        if int(elapsed) % 10 == 0 and int(elapsed) > 0:
            t = int(elapsed) - 10
            if t >= 0 and hasattr(self, '_last_summary') and self._last_summary != int(elapsed):
                pass  # avoid duplicates
            self._last_summary = int(elapsed)

    def print_summary(self):
        print(" " + "-" * 72)
        print("  [统计] 累计里程: {:.3f} m  |  最高速度: {:.3f} m/s".format(
            self.total_distance, self.max_speed))


def main():
    rclpy.init()
    node = OdomTester()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_summary()
        print("\n退出。")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
