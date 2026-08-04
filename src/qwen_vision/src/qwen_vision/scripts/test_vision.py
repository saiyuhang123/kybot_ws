#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试脚本：测试 qwen_point.py 的深度相机位置计算 + OpenCV 画框可视化功能

用法：
  1. 先启动 qwen_point 节点和相机驱动
  2. 运行本脚本：
     rosrun qwen_vision test_vision.py

功能：
  - 订阅 /visualization 话题，用 OpenCV 实时显示带框的标注图像
  - 订阅 /object_position 话题，打印计算出的 3D 位置
  - 订阅 /object_name、/image_description 话题，打印检测结果
  - 可通过命令行输入目标物名称，调用 /locate_object_sync 服务触发检测
  - 按 'q' 退出，按 's' 触发一次检测，按 't' 输入目标物名称

可选 ROS 参数：
  ~display (bool, default=True)  — 是否弹窗显示可视化图像
  ~save_dir  (str,  default='')   — 不为空时保存收到的图像到该目录
  ~target    (str,  default='')   — 启动时自动设置的目标物
"""

import os
import sys
import time
import threading
import rospy
import cv2
import json
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest
from cv_bridge import CvBridge


class VisionTester:
    def __init__(self):
        rospy.init_node('vision_tester', anonymous=True)

        # 参数
        self.display_enabled = rospy.get_param('~display', True)
        self.save_dir = rospy.get_param('~save_dir', '')
        self.default_target = rospy.get_param('~target', 'apple')

        if self.save_dir and not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        self.bridge = CvBridge()

        # 最新数据缓存
        self.lock = threading.Lock()
        self.latest_vis_img = None       # 可视化图像 (numpy)
        self.latest_position = None      # PoseStamped
        self.latest_object_name = ""
        self.latest_description = ""
        self.frame_count = 0

        # === 订阅 ===
        self.vis_sub = rospy.Subscriber(
            '/visualization', Image,
            self.visualization_callback, queue_size=1
        )
        self.pos_sub = rospy.Subscriber(
            '/object_position', PoseStamped,
            self.position_callback, queue_size=1
        )
        self.name_sub = rospy.Subscriber(
            '/object_name', String,
            self.name_callback, queue_size=1
        )
        self.desc_sub = rospy.Subscriber(
            '/image_description', String,
            self.description_callback, queue_size=1
        )

        # === 服务客户端 ===
        rospy.loginfo("Waiting for /locate_object_sync service...")
        try:
            rospy.wait_for_service('/locate_object_sync', timeout=5.0)
            self.sync_locate = rospy.ServiceProxy('/locate_object_sync', Trigger)
            rospy.loginfo("✓ /locate_object_sync service available")
        except rospy.ROSException:
            rospy.logwarn("✗ /locate_object_sync service NOT available (is qwen_point running?)")
            self.sync_locate = None

        # 发布目标物名称
        self.target_pub = rospy.Publisher('/target_object', String, queue_size=1)

        rospy.loginfo("=" * 60)
        rospy.loginfo("Vision Tester initialized!")
        rospy.loginfo("  Display: %s", self.display_enabled)
        rospy.loginfo("  Save dir: %s", (self.save_dir or '(disabled)'))
        rospy.loginfo("  Controls: [s] trigger detect  [t] set target  [q] quit")
        rospy.loginfo("=" * 60)

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
                fname = os.path.join(
                    self.save_dir,
                    f"vis_{msg.header.stamp.to_nsec()}.png"
                )
                cv2.imwrite(fname, img)
        except Exception as e:
            rospy.logwarn("visualization_callback error: %s", e)

    def position_callback(self, msg: PoseStamped):
        """接收 /object_position 话题的 3D 位置"""
        p = msg.pose.position
        with self.lock:
            self.latest_position = msg
        rospy.loginfo(
            "📍 Position [%s]: x=%.3f  y=%.3f  z=%.3f (dist=%.3f m)",
            msg.header.frame_id,
            p.x, p.y, p.z,
            float(np.sqrt(p.x**2 + p.y**2 + p.z**2))
        )

    def name_callback(self, msg: String):
        with self.lock:
            self.latest_object_name = msg.data
        rospy.loginfo("🏷️  Object name: %s", msg.data)

    def description_callback(self, msg: String):
        with self.lock:
            self.latest_description = msg.data
        rospy.loginfo("📝 Description: %s", msg.data)

    # ---------- 触发检测 ----------
    def trigger_detect(self):
        """调用同步定位服务"""
        if self.sync_locate is None:
            rospy.logerr("Service /locate_object_sync not available!")
            return False

        try:
            rospy.loginfo(">>> Triggering detection...")
            resp = self.sync_locate(TriggerRequest())
            if resp.success:
                rospy.loginfo("✓ Detection SUCCESS: %s", resp.message)
                return True
            else:
                rospy.logwarn("✗ Detection FAILED: %s", resp.message)
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return False

    def set_target(self, target_name: str):
        """通过 /target_object 话题设置目标物"""
        target_name = target_name.strip() #去掉 target_name 首尾的所有空白字符（例如空格、制表符 \t、换行符 \n 等）
        if not target_name:
            rospy.logwarn("Empty target name, ignored.")
            return
        self.target_pub.publish(String(target_name))
        rospy.loginfo("🎯 Target set to: %s", target_name)

    # ---------- 显示循环 ----------
    def run_display_loop(self):
        """OpenCV 弹窗循环显示 /visualization 图像"""
        if not self.display_enabled:
            rospy.loginfo("Display disabled. Press Ctrl-C to exit.")
            rospy.spin()
            return

        cv2.namedWindow("Vision Test - /visualization", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Vision Test - /visualization", 960, 720)

        # 叠加信息用的空白画布（无图像时显示提示）
        placeholder = np.zeros((480, 640, 3), dtype=np.uint8)

        print_help()

        rate = rospy.Rate(30)  # 30Hz 刷新
        while not rospy.is_shutdown():
            with self.lock:
                img = self.latest_vis_img.copy() if self.latest_vis_img is not None else None
                obj_name = self.latest_object_name
                desc = self.latest_description
                pos = self.latest_position

            if img is None:
                # 无图像时显示占位符 + 提示文字
                display = placeholder.copy()
                cv2.putText(display, "Waiting for /visualization...",
                            (50, 220), cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (255, 255, 255), 2)
                cv2.putText(display, "Press 's' to trigger detection",
                            (50, 270), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (200, 200, 200), 2)
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

            cv2.imshow("Vision Test - /visualization", display)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                rospy.loginfo("User pressed 'q', exiting...")
                break
            elif key == ord('s'):
                # 触发一次检测
                threading.Thread(target=self.trigger_detect, daemon=True).start()
            elif key == ord('t'):
                # 通过终端输入目标物名称
                print("\n" + "=" * 40)
                target = input("Enter target object name: ").strip()
                if target:
                    self.set_target(target)
                    # 设置后自动触发一次检测
                    rospy.sleep(0.3)
                    threading.Thread(target=self.trigger_detect, daemon=True).start()
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

    while not rospy.is_shutdown():
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
            rospy.sleep(0.3)
            tester.trigger_detect()
        elif action == 'h':
            print_help()
        else:
            print(f"Unknown command: {cmd}")


if __name__ == '__main__':
    try:
        tester = VisionTester()

        # 如果启动时指定了 target，就设置并触发
        if tester.default_target:
            rospy.sleep(0.5)
            tester.set_target(tester.default_target)
            rospy.sleep(0.3)
            tester.trigger_detect()

        if tester.display_enabled:
            tester.run_display_loop()
        else:
            run_cli_mode(tester)

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logfatal("Fatal error: %s", str(e))
