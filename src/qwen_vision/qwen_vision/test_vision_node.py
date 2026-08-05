#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试脚本：测试 qwen_point.py 的深度相机位置计算 + OpenCV 画框可视化功能

用法：
  1. 先启动 qwen_point 节点和相机驱动
  2. 运行本脚本：
     ros2 run qwen_vision test_vision_node

功能：
  - 订阅 /visualization 话题，用 OpenCV 实时显示带框的标注图像
  - 订阅 /object_position 话题，打印计算出的 3D 位置
  - 订阅 /object_name、/image_description 话题，打印检测结果
  - 可通过命令行输入目标物名称，调用 /locate_object_sync 服务触发检测
  - 按 'q' 退出，按 's' 触发一次检测，按 't' 输入目标物名称

可选 ROS 参数：
  display (bool, default=True)  — 是否弹窗显示可视化图像
  save_dir  (str,  default='')   — 不为空时保存收到的图像到该目录
  target    (str,  default='')   — 启动时自动设置的目标物
"""

import os
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

import cv2
import json
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge


class VisionTester(Node):
    def __init__(self):
        super().__init__('vision_tester')

        # 参数
        self.declare_parameter('display', True)
        self.declare_parameter('save_dir', '')
        self.declare_parameter('target', 'apple')

        self.display_enabled = self.get_parameter('display').get_parameter_value().bool_value
        self.save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        self.default_target = self.get_parameter('target').get_parameter_value().string_value

        if self.save_dir and not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        self.bridge = CvBridge()

        # 最新数据缓存
        self.lock = threading.Lock()
        self.status_lock = threading.Lock()
        self.status = "Ready: click the window, then press 's' to detect"
        self.latest_vis_img = None       # 可视化图像 (numpy)
        self.latest_position = None      # PoseStamped
        self.latest_object_name = ""
        self.latest_description = ""
        self.frame_count = 0

        # === 订阅 ===
        self.vis_sub = self.create_subscription(
            Image, '/visualization',
            self.visualization_callback, 1
        )
        self.pos_sub = self.create_subscription(
            PoseStamped, '/object_position',
            self.position_callback, 1
        )
        self.name_sub = self.create_subscription(
            String, '/object_name',
            self.name_callback, 1
        )
        self.desc_sub = self.create_subscription(
            String, '/image_description',
            self.description_callback, 1
        )

        # === 服务客户端 ===
        self.get_logger().info("Waiting for /locate_object_sync service...")
        self.sync_locate = self.create_client(Trigger, '/locate_object_sync')
        try:
            if self.sync_locate.wait_for_service(timeout_sec=5.0):
                self.get_logger().info("✓ /locate_object_sync service available")
            else:
                self.get_logger().warn("✗ /locate_object_sync service NOT available (is qwen_point running?)")
                self.sync_locate = None
        except Exception:
            self.get_logger().warn("✗ /locate_object_sync service NOT available (is qwen_point running?)")
            self.sync_locate = None

        # 发布目标物名称
        self.target_pub = self.create_publisher(String, '/target_object', 1)

        # 后台 ROS 线程：保证订阅/服务回调在显示循环期间也能执行
        self._spin_executor = SingleThreadedExecutor()
        self._spin_executor.add_node(self)
        self._spin_thread = threading.Thread(
            target=self._spin_executor.spin, daemon=True,
            name='vision_tester_ros_spin')
        self._spin_thread.start()

        self.get_logger().info("=" * 60)
        self.get_logger().info("Vision Tester initialized!")
        self.get_logger().info(f"  Display: {self.display_enabled}")
        self.get_logger().info(f"  Save dir: {self.save_dir or '(disabled)'}")
        self.get_logger().info("  Controls: [s] trigger detect  [t] set target  [q] quit")
        self.get_logger().info("=" * 60)

    def set_status(self, text: str):
        with self.status_lock:
            self.status = text
        self.get_logger().info(f"[status] {text}")

    def shutdown_spin_thread(self):
        try:
            self._spin_executor.shutdown()
        except Exception:
            pass
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)

    # ---------- 回调 ----------
    def visualization_callback(self, msg: Image):
        """接收 /visualization 话题的标注图像"""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock:
                self.latest_vis_img = img
                self.frame_count += 1

            # 可选存图
            if self.save_dir:
                stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                fname = os.path.join(
                    self.save_dir,
                    f"vis_{stamp_ns}.png"
                )
                cv2.imwrite(fname, img)
        except Exception as e:
            self.get_logger().warn(f"visualization_callback error: {e}")

    def position_callback(self, msg: PoseStamped):
        """接收 /object_position 话题的 3D 位置"""
        p = msg.pose.position
        with self.lock:
            self.latest_position = msg
        self.get_logger().info(
            f"📍 Position [{msg.header.frame_id}]: "
            f"x={p.x:.3f}  y={p.y:.3f}  z={p.z:.3f} "
            f"(dist={float(np.sqrt(p.x**2 + p.y**2 + p.z**2)):.3f} m)"
        )

    def name_callback(self, msg: String):
        with self.lock:
            self.latest_object_name = msg.data
        self.get_logger().info(f"🏷️  Object name: {msg.data}")

    def description_callback(self, msg: String):
        with self.lock:
            self.latest_description = msg.data
        self.get_logger().info(f"📝 Description: {msg.data}")

    # ---------- 触发检测 ----------
    def trigger_detect(self):
        """调用同步定位服务"""
        if self.sync_locate is None:
            self.set_status("✗ /locate_object_sync service NOT available")
            self.get_logger().error("Service /locate_object_sync not available!")
            return False

        try:
            self.set_status(">>> Triggering detection...")
            self.get_logger().info(">>> Triggering detection...")
            future = self.sync_locate.call_async(Trigger.Request())

            deadline = time.time() + 20.0
            while rclpy.ok() and not future.done():
                if time.time() > deadline:
                    self.set_status("✗ Detection timeout")
                    self.get_logger().warn("Detection timeout")
                    return False
                time.sleep(0.05)

            resp = future.result()
            if resp.success:
                self.set_status(f"✓ {resp.message}")
                self.get_logger().info(f"✓ Detection SUCCESS: {resp.message}")
                return True
            else:
                self.set_status(f"✗ Detection FAILED: {resp.message}")
                self.get_logger().warn(f"✗ Detection FAILED: {resp.message}")
                return False
        except Exception as e:
            self.set_status(f"✗ Service call failed: {e}")
            self.get_logger().error(f"Service call failed: {e}")
            return False

    def set_target(self, target_name: str):
        """通过 /target_object 话题设置目标物"""
        target_name = target_name.strip() #去掉 target_name 首尾的所有空白字符（例如空格、制表符 \t、换行符 \n 等）
        if not target_name:
            self.get_logger().warn("Empty target name, ignored.")
            return
        self.target_pub.publish(String(data=target_name))
        self.get_logger().info(f"🎯 Target set to: {target_name}")

    def run_terminal_commands(self):
        """显示模式下在后台读取终端命令，避免窗口被 input() 卡住"""
        print("Terminal commands: s/detect, t <name>, h, q")
        while rclpy.ok():
            try:
                cmd = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not cmd:
                continue
            parts = cmd.split(None, 1)
            action = parts[0].lower()
            if action in ('q', 'quit', 'exit'):
                rclpy.shutdown()
                break
            elif action in ('s', 'detect'):
                threading.Thread(target=self.trigger_detect, daemon=True).start()
            elif action == 't' and len(parts) > 1:
                self.set_target(parts[1])
                time.sleep(0.3)
                threading.Thread(target=self.trigger_detect, daemon=True).start()
            elif action == 'h':
                print_help()
            else:
                print(f"Unknown command: {cmd}")

    # ---------- 显示循环 ----------
    def run_display_loop(self):
        """OpenCV 弹窗循环显示 /visualization 图像"""
        if not self.display_enabled:
            self.get_logger().info("Display disabled. Press Ctrl-C to exit.")
            rclpy.spin(self)
            return

        cv2.namedWindow("Vision Test - /visualization", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Vision Test - /visualization", 960, 720)

        # 叠加信息用的空白画布（无图像时显示提示）
        placeholder = np.zeros((480, 640, 3), dtype=np.uint8)

        print_help()
        threading.Thread(target=self.run_terminal_commands, daemon=True,
                         name='vision_tester_terminal_input').start()

        rate = self.create_rate(30)  # 30Hz 刷新
        while rclpy.ok():
            with self.lock:
                img = self.latest_vis_img.copy() if self.latest_vis_img is not None else None
                obj_name = self.latest_object_name
                desc = self.latest_description
                pos = self.latest_position
            with self.status_lock:
                status = self.status

            if img is None:
                # 无图像时显示占位符 + 提示文字
                display = placeholder.copy()
                cv2.putText(display, "Waiting for /visualization...",
                            (50, 220), cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (255, 255, 255), 2)
                cv2.putText(display, "Press 's' to trigger detection",
                            (50, 270), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (200, 200, 200), 2)
                cv2.putText(display, status,
                            (50, 330), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 255) if status.startswith("✗") else (180, 180, 180), 2)
            else:
                display = img

                # 在图像顶部叠加文字信息
                y_offset = 30
                if obj_name:
                    cv2.putText(display, f"Target: {obj_name}",
                                (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (255, 255, 0), 2)
                    y_offset += 28
                if desc:
                    cv2.putText(display, desc,
                                (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (255, 255, 0), 2)
                    y_offset += 22
                if pos is not None:
                    p = pos.pose.position
                    cv2.putText(display,
                                f"Pos({pos.header.frame_id}): "
                        f"({p.x:.3f}, {p.y:.3f}, {p.z:.3f})",
                                (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 255, 255), 2)
                cv2.putText(display, status,
                            (10, display.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 0, 255) if status.startswith("✗") else (255, 255, 0), 2)

            cv2.imshow("Vision Test - /visualization", display)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                self.get_logger().info("User pressed 'q', exiting...")
                break
            elif key == ord('s'):
                # 触发一次检测
                threading.Thread(target=self.trigger_detect, daemon=True).start()
            elif key == ord('t'):
                # 在终端输入目标物名称，不阻塞窗口
                print("Type target in the terminal:  t <name>  (e.g. t apple)")
            elif key == ord('h'):
                print_help()

            rate.sleep()

        cv2.destroyAllWindows()


def print_help():
    print("\n" + "=" * 40)
    print("  Keyboard Controls:")
    print("    s  — Trigger object detection")
    print("    t  — Set target object name + detect")
    print("    h  — Show this help")
    print("    q  — Quit")
    print("=" * 40 + "\n")


# ========== 命令行交互模式（无 GUI 时用）==========
def run_cli_mode(tester: VisionTester):
    """无 GUI 的纯命令行交互"""
    print_help()
    print("CLI mode: type commands and press Enter.")
    print("  s / detect   — trigger detection")
    print("  t <name>     — set target (e.g. 't water bottle')")
    print("  q / quit     — exit")
    print()

    while rclpy.ok():
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not cmd:
            continue

        parts = cmd.split(None, 1)
        action = parts[0].lower()

        if action in ('q', 'quit', 'exit'):
            break
        elif action in ('s', 'detect'):
            tester.trigger_detect()
        elif action == 't' and len(parts) > 1:
            tester.set_target(parts[1])
            time.sleep(0.3)
            tester.trigger_detect()
        elif action == 'h':
            print_help()
        else:
            print(f"Unknown command: {cmd}")


def main(args=None):
    rclpy.init(args=args)
    tester = VisionTester()

    try:
        # 如果启动时指定了 target，就设置并触发
        if tester.default_target:
            time.sleep(0.5)
            tester.set_target(tester.default_target)
            time.sleep(0.3)
            tester.trigger_detect()

        if tester.display_enabled:
            tester.run_display_loop()
        else:
            run_cli_mode(tester)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        tester.get_logger().fatal(f"Fatal error: {str(e)}")
    finally:
        try:
            tester.shutdown_spin_thread()
        except Exception:
            pass
        try:
            tester.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
