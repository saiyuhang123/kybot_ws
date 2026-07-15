#!/usr/bin/env python3
# coding=utf-8

import copy
import threading

import rclpy
from rclpy.node import Node
import numpy as np

import tf2_ros
try:
    import tf_transformations
except ImportError:
    from tf2_ros import transformations as tf_transformations

from geometry_msgs.msg import Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry

class TransformFusionNode(Node):
    def __init__(self):
        super().__init__('transform_fusion')
        self.FREQ_PUB_LOCALIZATION = 50.0  # Hz

        self.lock = threading.Lock()
        self.cur_odom_fastlio = None        # FastLIO /Odometry: odom_FL -> body
        self.cur_map_to_odom_fl = None      # ICP 结果: map -> odom_FL

        # tf buffer 用于获取 EKF 发布的 odom->base_link
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 구독자
        self.create_subscription(Odometry, '/Odometry', self.cb_save_fastlio_odom, 1)
        self.create_subscription(Odometry, '/map_to_odom', self.cb_save_map_to_odom, 1)

        # 퍼블리셔, 브로드캐스터
        self.pub_localization = self.create_publisher(Odometry, '/localization', 1)
        self.tf_broadcaster    = tf2_ros.TransformBroadcaster(self)

        # 주기 타이머
        period = 1.0 / self.FREQ_PUB_LOCALIZATION
        self.create_timer(period, self.timer_callback)
        self.get_logger().info('Transform Fusion Node Initialized')

    def cb_save_fastlio_odom(self, msg: Odometry):
        with self.lock:
            self.cur_odom_fastlio = msg

    def cb_save_map_to_odom(self, msg: Odometry):
        with self.lock:
            self.cur_map_to_odom_fl = msg

    def pose_to_mat(self, odom_msg: Odometry) -> np.ndarray:
        t = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        trans = tf_transformations.translation_matrix([t.x, t.y, t.z])
        rot   = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        return trans @ rot

    def tf_to_mat(self, tf_msg: TransformStamped) -> np.ndarray:
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        trans = tf_transformations.translation_matrix([t.x, t.y, t.z])
        rot   = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        return trans @ rot

    def timer_callback(self):
        with self.lock:
            fastlio_odom = copy.deepcopy(self.cur_odom_fastlio)
            map2odom_fl  = copy.deepcopy(self.cur_map_to_odom_fl)
        if fastlio_odom is None:
            return

        # FastLIO 的全局位姿: map -> body
        T_map_to_odom_fl   = self.pose_to_mat(map2odom_fl) if map2odom_fl is not None else np.eye(4)
        T_odom_fl_to_body  = self.pose_to_mat(fastlio_odom)
        T_map_to_body      = T_map_to_odom_fl @ T_odom_fl_to_body

        # 获取 EKF 发布的 odom -> base_link
        T_odom_ekf_to_body = None
        try:
            ekf_tf = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())
            T_odom_ekf_to_body = self.tf_to_mat(ekf_tf)
        except Exception as e:
            self.get_logger().warn(f'Cannot lookup odom->base_link from EKF: {e}')

        # 用 EKF 的 odom 定义来计算 map->odom
        if T_odom_ekf_to_body is not None:
            T_map_to_odom = T_map_to_body @ np.linalg.inv(T_odom_ekf_to_body)
        else:
            T_map_to_odom = T_map_to_odom_fl

        trans = tf_transformations.translation_from_matrix(T_map_to_odom)
        quat  = tf_transformations.quaternion_from_matrix(T_map_to_odom)

        # tf 브로드캐스트: map -> odom (与 EKF 的 odom 定义一致)
        t_msg = TransformStamped()
        t_msg.header.stamp = self.get_clock().now().to_msg()
        t_msg.header.frame_id    = 'map'
        t_msg.child_frame_id     = 'odom'
        t_msg.transform.translation.x = trans[0]
        t_msg.transform.translation.y = trans[1]
        t_msg.transform.translation.z = trans[2]
        t_msg.transform.rotation.x    = quat[0]
        t_msg.transform.rotation.y    = quat[1]
        t_msg.transform.rotation.z    = quat[2]
        t_msg.transform.rotation.w    = quat[3]
        self.tf_broadcaster.sendTransform(t_msg)

        # fused localization (map -> body)
        xyz   = tf_transformations.translation_from_matrix(T_map_to_body)
        quat2 = tf_transformations.quaternion_from_matrix(T_map_to_body)

        loc_msg = Odometry()
        loc_msg.header.stamp          = fastlio_odom.header.stamp
        loc_msg.header.frame_id       = 'map'
        loc_msg.child_frame_id        = 'body'
        loc_msg.pose.pose.position    = Point(x=xyz[0], y=xyz[1], z=xyz[2])
        loc_msg.pose.pose.orientation = Quaternion(
            x=quat2[0], y=quat2[1], z=quat2[2], w=quat2[3]
        )
        loc_msg.twist = fastlio_odom.twist
        self.pub_localization.publish(loc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TransformFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
