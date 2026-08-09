#!/usr/bin/env python3
"""假 mission_executor + 假臂/相机/传感服务, 用于离线测试 brain 分步链路.

用法: 设一个隔离域后启动, 再启动 brain, 发 /brain_text 即可:
    export ROS_DOMAIN_ID=99
    python3 tools/fake_executor.py &
    ros2 launch kybot_brain kybot_brain.launch.py
行为: /mission/run 受理后发布 导航中(8s)→已完成; grasp 2s 后成功;
拍照立即成功; /odom 静止、/scan_fe 正前方 0.5m 有障碍(approach 立即早停).
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from hk_camera.msg import MissionStatus
from hk_camera.srv import CapturePicture, RunMission
from nav_msgs.msg import Odometry
from ocr_interfaces.msg import OcrDetection
from ocr_interfaces.srv import RecognizeText
from sensor_msgs.msg import LaserScan


class FakeExecutor(Node):
    def __init__(self):
        super().__init__('fake_executor')
        self._msg = ''
        self._state = MissionStatus.STATE_IDLE
        self._status_pub = self.create_publisher(MissionStatus, '/mission/status', 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._scan_pub = self.create_publisher(LaserScan, '/scan_fe', 10)
        self.create_service(RunMission, '/mission/run', self._on_run)
        self.create_service(Trigger, '/mission/cancel', self._on_cancel)
        self.create_service(Trigger, '/yolo_grasp/grasp_hold', self._on_grasp)
        self.create_service(CapturePicture, '/hk_camera/capture', self._on_capture)
        self.create_service(RecognizeText, '/ocr/recognize', self._on_ocr)
        # 周期广播当前状态, 模拟真实 executor
        self.create_timer(0.5, self._pub_status)
        self.create_timer(0.1, self._pub_sensors)

    def _pub_sensors(self):
        o = Odometry()
        o.pose.pose.orientation.w = 1.0
        self._odom_pub.publish(o)
        s = LaserScan()
        s.angle_min = -math.pi
        s.angle_increment = math.radians(1.0)
        s.range_min = 0.05
        s.range_max = 30.0
        s.ranges = [10.0] * 360
        s.ranges[180] = 0.5  # 正前方 0.5m 障碍 → approach 应立即早停
        self._scan_pub.publish(s)

    def _set(self, state, message=''):
        self._state = state
        self._msg = message
        self._pub_status()

    def _pub_status(self):
        m = MissionStatus()
        m.state = self._state
        m.current_index = 0
        m.total_count = 1
        m.message = self._msg
        self._status_pub.publish(m)

    def _on_run(self, req, res):
        wp = req.waypoints[0]
        self.get_logger().info('收到任务: %d 个点, capture=%s action="%s"'
                               % (len(req.waypoints), wp.do_capture,
                                  wp.extra_action))
        res.accepted = True
        res.message = 'ok'
        threading.Thread(target=self._run_mission, daemon=True).start()
        return res

    def _run_mission(self):
        self._set(MissionStatus.STATE_NAVIGATING, 'fake nav')
        time.sleep(8.0)
        self._set(MissionStatus.STATE_COMPLETED, 'fake 到达')

    def _on_cancel(self, req, res):
        self._set(MissionStatus.STATE_CANCELED, 'fake 取消')
        res.success = True
        res.message = 'canceled'
        return res

    def _on_grasp(self, req, res):
        time.sleep(2.0)
        res.success = True
        res.message = 'fake 已抓住瓶子'
        return res

    def _on_capture(self, req, res):
        res.success = True
        res.message = 'fake 拍照成功'
        return res

    def _on_ocr(self, req, res):
        res.success = True
        res.message = 'ok'
        d1 = OcrDetection(); d1.text = '压力表'; d1.confidence = 0.95
        d2 = OcrDetection(); d2.text = '23.5'; d2.confidence = 0.88
        res.detections = [d1, d2]
        res.processing_time_ms = 120.0
        return res


def main():
    rclpy.init()
    rclpy.spin(FakeExecutor())


if __name__ == '__main__':
    main()
