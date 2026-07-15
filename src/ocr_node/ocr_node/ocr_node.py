#!/usr/bin/env python3
"""
OCR Node - PaddleOCR-based text and number recognition for ROS2.

Subscribes:
  /hk_camera/image_raw (sensor_msgs/Image) — camera frames

Publishes:
  /ocr/result        (ocr_interfaces/OcrResult)    — recognized text detections
  /ocr/annotated_img (sensor_msgs/Image)           — annotated preview image

Parameters (ROS2):
  lang                 — OCR language: 'ch' (Chinese+English), 'en' (English only)
  use_gpu              — Use GPU acceleration (True/False)
  conf_threshold       — Minimum confidence to publish (0.0-1.0)
  processing_interval  — Minimum seconds between OCR runs (float, default 0.5)
  enable_annotated_img — Publish annotated image (True/False)
"""

import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from ocr_interfaces.msg import OcrDetection, OcrResult


class OcrNode(Node):
    """ROS2 node that runs PaddleOCR on incoming camera frames."""

    def __init__(self):
        super().__init__('ocr_node')

        # ---- Parameters ----
        self.declare_parameter('lang', 'ch')
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('processing_interval', 0.5)
        self.declare_parameter('enable_annotated_img', True)

        self._lang = self.get_parameter('lang').value
        self._use_gpu = self.get_parameter('use_gpu').value
        self._conf_threshold = self.get_parameter('conf_threshold').value
        self._interval = self.get_parameter('processing_interval').value
        self._publish_annotated = self.get_parameter('enable_annotated_img').value

        self.get_logger().info(
            f'Initializing PaddleOCR (lang={self._lang}, gpu={self._use_gpu})...'
        )

        # ---- Lazy-import PaddleOCR (survives missing package until first frame) ----
        self._ocr = None
        self._ocr_lock = threading.Lock()
        try:
            self._init_ocr()
        except Exception as e:
            self.get_logger().warn(
                f'PaddleOCR not available yet: {e}. '
                'Will retry on first frame.'
            )

        # ---- CV Bridge ----
        self._bridge = CvBridge()

        # ---- Subscriber ----
        self._sub = self.create_subscription(
            Image,
            '/hk_camera/image_raw',
            self._image_callback,
            10,
        )

        # ---- Publishers ----
        self._result_pub = self.create_publisher(OcrResult, '/ocr/result', 10)
        self._annotated_pub = None
        if self._publish_annotated:
            self._annotated_pub = self.create_publisher(Image, '/ocr/annotated_img', 10)

        # ---- Processing throttle ----
        self._last_time = 0.0
        self._processing = False
        self._frame_count = 0

        self.get_logger().info(
            'OCR Node ready. '
            f'Listening on /hk_camera/image_raw, '
            f'interval={self._interval}s'
        )

    # ------------------------------------------------------------------
    def _init_ocr(self):
        """Create the PaddleOCR instance (call with lock held)."""
        import paddleocr  # noqa: F401 — verify availability
        from paddleocr import PaddleOCR

        # Suppress PaddleOCR's own debug logs
        self._ocr = PaddleOCR(
            lang=self._lang,
            use_gpu=self._use_gpu,
            show_log=False,
        )
        self.get_logger().info(
            f'PaddleOCR initialized (lang={self._lang}, gpu={self._use_gpu})'
        )

    # ------------------------------------------------------------------
    def _image_callback(self, msg: Image):
        """ROS subscriber callback — throttle and dispatch to worker thread."""
        now = time.time()
        if now - self._last_time < self._interval:
            return               # skip — not yet time for next frame
        if self._processing:
            return               # skip — still working on previous frame
        self._last_time = now
        self._processing = True
        self._frame_count += 1

        t = threading.Thread(target=self._process_frame, args=(msg,), daemon=True)
        t.start()

    # ------------------------------------------------------------------
    def _process_frame(self, msg: Image):
        """Decode image, run OCR, publish results (runs in worker thread)."""
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            self._processing = False
            return

        # ---- Lazy-init OCR on first frame if it failed at startup ----
        if self._ocr is None:
            with self._ocr_lock:
                if self._ocr is None:
                    try:
                        self._init_ocr()
                    except Exception as exc:
                        self.get_logger().error(
                            f'Cannot initialize PaddleOCR: {exc}'
                        )
                        self._processing = False
                        return

        # ---- Run OCR ----
        t0 = time.time()
        try:
            raw = self._ocr.ocr(cv_image)
        except Exception as exc:
            self.get_logger().error(f'PaddleOCR inference error: {exc}')
            self._processing = False
            return
        proc_ms = (time.time() - t0) * 1000.0

        # ---- Build result message ----
        result = OcrResult()
        result.header = msg.header
        result.processing_time_ms = float(proc_ms)

        detections = []
        if raw and raw[0]:
            for line in raw[0]:
                box = line[0]          # [[x1,y1],[x2,y2],[x3,y3],[x4,y4]]
                text, conf = line[1]

                if conf < self._conf_threshold:
                    continue

                d = OcrDetection()
                d.corners = [float(v) for p in box for v in p]
                d.text = text
                d.confidence = float(conf)
                detections.append(d)

                # ---- Draw on preview image ----
                pts = np.array([[int(p[0]), int(p[1])] for p in box], dtype=np.int32)
                hull = cv2.convexHull(pts)
                cv2.polylines(cv_image, [hull], isClosed=True, color=(0, 255, 0), thickness=2)
                cv2.putText(
                    cv_image, f'{text} ({conf:.2f})',
                    org=(int(box[0][0]), max(int(box[0][1]) - 8, 10)),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.5,
                    color=(0, 255, 0),
                    thickness=1,
                )

        result.detections = detections
        self._result_pub.publish(result)

        # ---- Publish annotated image ----
        if self._annotated_pub is not None:
            ann_msg = self._bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            ann_msg.header = msg.header
            self._annotated_pub.publish(ann_msg)

        if detections:
            texts = ', '.join(d.text for d in detections[:5])
            if len(detections) > 5:
                texts += '...'
            self.get_logger().info(
                f'[{proc_ms:.0f}ms] {len(detections)} text(s): {texts}'
            )
        else:
            self.get_logger().debug(f'[{proc_ms:.0f}ms] no text detected')

        self._processing = False


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = OcrNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
