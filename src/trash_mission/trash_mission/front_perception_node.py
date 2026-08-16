#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M2：车前 D435 实时检测 + 距离测量 + 停车确认。

订阅车前 D435 彩色图和对齐深度，用 YOLO 模型实时检测（默认使用轻量
yolov8n.pt，可通过 model_path 参数换成 YOLO-World 等模型），
在检测框内取深度中位数作为距离；订阅 /odom 判断车是否停稳，
停稳后多帧距离稳定才输出 stationary_confirm。

发布：
  /trash/annotated_image   画框+距离标注图
  /trash/target            TrashTarget 结构化结果
  /trash/detection         JSON 调试结果

用法：
  ros2 run trash_mission front_perception_node
  ros2 run trash_mission front_perception_node --ros-args \
      -p prompts:="bottle" -p show:=true

运行时切换提示词：
  ros2 topic pub /trash/target_class std_msgs/msg/String \
      "{data: 'bottle'}"
"""

import json
import threading
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
from trash_mission_interfaces.msg import TrashTarget


class FrontPerceptionNode(Node):
    def __init__(self):
        super().__init__('front_perception_node')

        # ---------------- 参数 ----------------
        self.declare_parameter(
            'model_path',
            '/home/nvidia/Documents/elite_robot_ws/YOLO/yolov8n.pt')
        self.declare_parameter('prompts', 'bottle')
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('color_topic',
                               '/front_camera/camera/color/image_raw')
        self.declare_parameter(
            'depth_topic',
            '/front_camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('info_topic',
                               '/front_camera/camera/color/camera_info')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('show', True)
        self.declare_parameter('infer_every', 1)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('stationary_speed_threshold', 0.05)
        self.declare_parameter('confirm_frames', 5)
        self.declare_parameter('confirm_tol', 0.05)
        self.declare_parameter('distance_window', 7)
        self.declare_parameter('min_valid_depth_ratio', 0.2)
        self.declare_parameter('depth_grace', 0.5)  # 深度短暂失效时沿用旧距离的秒数

        self.model_path = self.get_parameter('model_path').value
        self.prompts = [
            p.strip() for p in
            self.get_parameter('prompts').value.split(',') if p.strip()
        ]
        self.conf = self.get_parameter('conf').value
        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.info_topic = self.get_parameter('info_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.show = self.get_parameter('show').value
        self.infer_every = max(1, self.get_parameter('infer_every').value)
        self.depth_scale = self.get_parameter('depth_scale').value
        self.speed_thresh = self.get_parameter(
            'stationary_speed_threshold').value
        self.confirm_frames = max(
            3, self.get_parameter('confirm_frames').value)
        self.confirm_tol = self.get_parameter('confirm_tol').value
        self.distance_window = max(
            3, self.get_parameter('distance_window').value)
        self.min_valid_ratio = self.get_parameter(
            'min_valid_depth_ratio').value
        self.depth_grace = self.get_parameter('depth_grace').value
        # 标准检测模型（如 yolov8n）按 COCO 类别 id 过滤；None 表示不过滤
        self._class_ids = None

        self.bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._latest_stamp = None
        self._frame_count = 0
        self._quit = False

        # M2：深度/内参/里程计缓存
        self._depth_lock = threading.Lock()
        self._latest_depth = None
        self._fx = None
        self._ppx = None
        self._odom_speed = None
        self._odom_stamp = None
        self._have_odom = False
        self._odom_stale_warned = False

        # 距离滤波与停车确认
        self._dist_window = deque(maxlen=self.distance_window)
        self._confirm_window = deque(maxlen=max(10, self.confirm_frames + 5))
        self._stationary_confirm = False
        self._last_filtered_dist = None
        self._last_lateral = None
        self._last_real_dist_time = None

        # ---------------- 模型 ----------------
        self.get_logger().info(f'加载模型: {self.model_path}')
        self.model = YOLO(self.model_path)
        self._apply_prompts()
        self.get_logger().info(
            f'模型就绪, 提示词: {self.prompts}, conf={self.conf}')

        # 预热一次，避免第一帧卡顿
        try:
            self.model(np.zeros((480, 640, 3), dtype=np.uint8),
                       verbose=False)
            self.get_logger().info('模型预热完成')
        except Exception as e:
            self.get_logger().warn(f'模型预热失败(不影响运行): {e}')

        # ---------------- 话题 ----------------
        img_qos = QoSProfile(depth=2,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
        self.annotated_pub = self.create_publisher(
            Image, '/trash/annotated_image', 10)
        self.target_pub = self.create_publisher(
            TrashTarget, '/trash/target', 10)
        self.detection_pub = self.create_publisher(
            String, '/trash/detection', 10)
        self.create_subscription(
            String, '/trash/target_class', self._target_class_cb, 10)
        self.create_subscription(
            Image, self.color_topic, self._color_cb, img_qos)
        self.create_subscription(
            Image, self.depth_topic, self._depth_cb, img_qos)
        self.create_subscription(
            CameraInfo, self.info_topic, self._info_cb, 10)
        self.create_subscription(
            Odometry, self.odom_topic, self._odom_cb, 10)

        # 处理帧的定时器（20Hz 轮询最新帧，不在回调里阻塞）
        self.create_timer(0.05, self._process_latest_frame)
        self.get_logger().info(
            f'订阅 {self.color_topic} / {self.depth_topic} / '
            f'{self.odom_topic}，发布 /trash/target')

    # ---------------- 提示词 ----------------
    def _apply_prompts(self):
        """把提示词应用到当前模型。

        YOLO-World 等开放词汇模型使用 set_classes；
        yolov8n 等标准检测模型把提示词映射为 COCO 类别 id，
        推理时通过 classes= 过滤，避免检测出全部 80 类。
        """
        self._class_ids = None
        if hasattr(self.model, 'set_classes'):
            try:
                self.model.set_classes(self.prompts)
                self.get_logger().info(f'已设置开放词汇: {self.prompts}')
                return
            except Exception as e:
                self.get_logger().warn(f'set_classes 失败({e})，改为类别id过滤')

        if any(p.strip().lower() in ('all', '全部', '*')
               for p in self.prompts):
            self.get_logger().info('提示词为 all，使用模型全部类别')
            return

        names = self.model.names if isinstance(self.model.names, dict) else {}
        class_ids = []
        for prompt in self.prompts:
            p = prompt.strip().lower()
            for class_id, name in names.items():
                name_l = str(name).lower()
                if name_l and name_l in p:
                    class_ids.append(int(class_id))
        self._class_ids = sorted(set(class_ids))
        if not self._class_ids:
            self.get_logger().warn(
                f'标准检测模型中没有匹配提示词的类别: {self.prompts}，'
                '将不输出检测结果')
        else:
            mapped = [names[i] for i in self._class_ids]
            self.get_logger().info(
                f'标准检测模型类别过滤: {self.prompts} -> {mapped} '
                f'(class ids={self._class_ids})')

    def _target_class_cb(self, msg):
        prompts = [p.strip() for p in msg.data.split(',') if p.strip()]
        if not prompts:
            return
        self.prompts = prompts
        self._apply_prompts()
        self.get_logger().info(f'提示词已切换为: {self.prompts}')

    # ---------------- 图像回调 ----------------
    def _color_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return
        with self._frame_lock:
            self._latest_frame = frame
            self._latest_stamp = msg.header.stamp
            self._frame_count += 1

    def _depth_cb(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error(f'深度图转换失败: {e}')
            return
        with self._depth_lock:
            self._latest_depth = depth

    def _info_cb(self, msg):
        try:
            if len(msg.k) >= 9:
                self._fx = float(msg.k[0])
                self._ppx = float(msg.k[2])
        except Exception as e:
            self.get_logger().error(f'camera_info 解析失败: {e}')

    def _odom_cb(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._odom_speed = float(np.hypot(vx, vy))
        self._odom_stamp = self.get_clock().now()
        self._have_odom = True

    # ---------------- M2 辅助 ----------------
    def _is_moving(self):
        """车是否在运动。没有里程计数据时按静止处理，方便单独调试。"""
        if not self._have_odom or self._odom_speed is None:
            if not self._odom_stale_warned:
                self.get_logger().warn(
                    f'未收到 {self.odom_topic}，按静止状态处理')
                self._odom_stale_warned = True
            return False
        if self._odom_stamp is None:
            return False
        age = (self.get_clock().now() - self._odom_stamp).nanoseconds / 1e9
        if age > 1.0:
            if not self._odom_stale_warned:
                self.get_logger().warn(
                    f'{self.odom_topic} 数据超时({age:.1f}s)，按静止处理')
                self._odom_stale_warned = True
            return False
        return self._odom_speed >= self.speed_thresh

    def _pick_best_object(self, objects):
        if not objects:
            return None
        bottles = [o for o in objects if 'bottle' in o['class'].lower()]
        pool = bottles if bottles else objects
        return max(pool, key=lambda o: o['conf'])

    def _compute_distance(self, box):
        """检测框内缩 20% 取深度中位数，返回 (距离米, 横向偏移米)。"""
        with self._depth_lock:
            depth = self._latest_depth
        if depth is None:
            return None, None

        h, w = depth.shape[:2]
        x1, y1, x2, y2 = box['x1'], box['y1'], box['x2'], box['y2']
        bw = x2 - x1
        bh = y2 - y1
        if bw < 8 or bh < 8:
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            x1, y1, x2, y2 = cx - 3, cy - 3, cx + 3, cy + 3
        else:
            x1 += bw * 0.1
            x2 -= bw * 0.1
            y1 += bh * 0.1
            y2 -= bh * 0.1

        x1 = max(0, int(x1))
        y1 = max(0, int(y1))
        x2 = min(w - 1, int(x2))
        y2 = min(h - 1, int(y2))
        if x2 <= x1 or y2 <= y1:
            return None, None

        roi = depth[y1:y2, x1:x2]
        if roi.size == 0:
            return None, None
        vals = roi.astype(np.float32).flatten()
        valid = vals[(vals > 0) & (vals < 10000)]
        if valid.size / vals.size < self.min_valid_ratio:
            return None, None

        dist = float(np.median(valid)) * self.depth_scale
        lateral = None
        if self._fx and self._ppx:
            cx = (box['x1'] + box['x2']) / 2.0
            lateral = float((cx - self._ppx) * dist / self._fx)
        return dist, lateral

    # ---------------- 推理 ----------------
    def _process_latest_frame(self):
        if self._quit:
            return
        with self._frame_lock:
            frame = self._latest_frame
            stamp = self._latest_stamp
            count = self._frame_count
        if frame is None:
            return
        if count % self.infer_every != 0:
            return

        # 模糊分：Laplacian 方差，越低越模糊
        blur = float(cv2.Laplacian(frame, cv2.CV_64F).var())

        try:
            results = self.model.predict(frame, conf=self.conf,
                                         classes=self._class_ids,
                                         verbose=False)
        except Exception as e:
            self.get_logger().error(f'推理异常: {e}')
            return

        annotated = frame.copy()
        objects = []
        r = results[0]
        names = r.names
        if r.boxes is not None:
            for box in r.boxes:
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = names.get(cls_id, str(cls_id))
                objects.append({
                    'class': cls_name,
                    'conf': round(conf, 3),
                    'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                })
                color = (0, 255, 0) if 'bottle' in cls_name.lower() \
                    else (0, 200, 255)
                cv2.rectangle(annotated, (int(x1), int(y1)),
                              (int(x2), int(y2)), color, 2)
                label = f'{cls_name} {conf:.2f}'
                cv2.putText(annotated, label, (int(x1), int(y1) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # ---------------- M2：距离测量与停车确认 ----------------
        best = self._pick_best_object(objects)
        moving = self._is_moving()
        dist = None
        lateral = None

        if best is not None:
            dist, lateral = self._compute_distance(best)
            if dist is not None:
                self._dist_window.append(dist)
                self._last_filtered_dist = float(
                    np.median(self._dist_window))
                self._last_lateral = lateral
                self._last_real_dist_time = self.get_clock().now()
            else:
                # 距离无效时不进确认窗口
                self._confirm_window.clear()
                self._stationary_confirm = False
        current_dist_valid = dist is not None

        # 深度短暂失效兜底：检测还在但深度只有零星有效帧时，
        # 短时间（depth_grace 秒）内继续用最近一次有效距离，
        # 避免小车一启动就因单帧深度丢失被调度判为“距离丢失”。
        depth_fallback = False
        if best is not None and self._last_real_dist_time is not None:
            age = (self.get_clock().now() -
                   self._last_real_dist_time).nanoseconds / 1e9
            depth_fallback = age <= self.depth_grace

        if dist is None or best is None:
            self._confirm_window.clear()
            self._stationary_confirm = False
        elif moving:
            # 运动中距离不可信，只粗检
            self._confirm_window.clear()
            self._stationary_confirm = False
        else:
            self._confirm_window.append(self._last_filtered_dist)
            if len(self._confirm_window) >= self.confirm_frames:
                spread = max(self._confirm_window) - min(self._confirm_window)
                self._stationary_confirm = spread <= self.confirm_tol
            else:
                self._stationary_confirm = False

        # 在框下方画距离
        if best is not None:
            dist_text = f'{self._last_filtered_dist:.2f}m' \
                if self._last_filtered_dist is not None else '--m'
            cv2.putText(annotated, dist_text,
                        (int(best['x1']), int(best['y2']) + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if self._stationary_confirm
                        else (0, 200, 255), 2)

        # 画面信息
        state = 'MOVING' if moving else 'STOP'
        confirm = 'OK' if self._stationary_confirm else '--'
        info = (f'blur={blur:.0f} detected={len(objects)} '
                f'dist={self._last_filtered_dist if self._last_filtered_dist is not None else "--"}m '
                f'{state} confirm={confirm} '
                f'prompts={",".join(self.prompts)}')
        cv2.putText(annotated, info, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # 发布标注图
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated,
                                                      encoding='bgr8')
            annotated_msg.header.stamp = stamp
            annotated_msg.header.frame_id = 'front_camera_color_optical_frame'
            self.annotated_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'发布标注图失败: {e}')

        # 发布结构化目标
        target = TrashTarget()
        target.stamp = stamp
        target.detected = best is not None
        target.cls_name = best['class'] if best is not None else ''
        target.confidence = float(best['conf']) if best is not None else 0.0
        target.distance = float(self._last_filtered_dist or 0.0)
        target.distance_valid = (
            (current_dist_valid or depth_fallback)
            and self._last_filtered_dist is not None)
        target.lateral_offset = float(self._last_lateral or 0.0)
        target.moving = moving
        target.stationary_confirm = self._stationary_confirm
        target.blur = float(blur)
        self.target_pub.publish(target)

        # 发布 JSON 调试结果
        sec = stamp.sec + stamp.nanosec * 1e-9 if stamp else 0.0
        payload = {
            'stamp': round(sec, 3),
            'detected': best is not None,
            'cls': best['class'] if best is not None else None,
            'conf': best['conf'] if best is not None else None,
            'distance': round(self._last_filtered_dist, 3)
                if self._last_filtered_dist is not None else None,
            'lateral': round(self._last_lateral, 3)
                if self._last_lateral is not None else None,
            'moving': moving,
            'stationary_confirm': self._stationary_confirm,
            'blur': round(blur, 1),
        }
        self.detection_pub.publish(String(data=json.dumps(payload)))

        if self.show:
            try:
                cv2.imshow('Front D435 - YOLO', annotated)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.get_logger().info('按 Q 退出')
                    self._quit = True
                    rclpy.shutdown()
                elif key == ord('b'):
                    self._set_prompt_from_key('bottle')
                elif key == ord('a'):
                    self._set_prompt_from_key('all')
            except cv2.error as e:
                self.get_logger().warn(
                    f'无法打开显示窗口(无图形界面)，自动切换为无窗口模式: {e}')
                self.show = False

    def _set_prompt_from_key(self, prompts):
        self.prompts = [p.strip() for p in prompts.split(',') if p.strip()]
        self._apply_prompts()
        self.get_logger().info(f'按键切换提示词: {self.prompts}')


def main(args=None):
    rclpy.init(args=args)
    node = FrontPerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.show:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
