#!/usr/bin/env python3
# coding=utf-8

import copy
import threading
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
import tf_transformations

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header


class GlobalLocalizationNode(Node):
    # PointField.datatype → numpy dtype, 用于向量化解析 PointCloud2
    _PF2NP = {
        PointField.INT8:    np.int8,
        PointField.UINT8:   np.uint8,
        PointField.INT16:   np.int16,
        PointField.UINT16:  np.uint16,
        PointField.INT32:   np.int32,
        PointField.UINT32:  np.uint32,
        PointField.FLOAT32: np.float32,
        PointField.FLOAT64: np.float64,
    }

    def __init__(self):
        super().__init__('fast_lio_localization')

        # ─── Parameters ───────────────────────────────────────────
        self.declare_parameter('map_voxel_size', 0.1)
        self.declare_parameter('scan_voxel_size', 0.1)
        self.declare_parameter('freq_localization', 0.5)      # Hz
        self.declare_parameter('localization_th', 0.9)
        self.declare_parameter('fov', 2 * np.pi)
        self.declare_parameter('fov_far', 100.0)

        self.map_voxel_size    = self.get_parameter('map_voxel_size').value
        self.scan_voxel_size   = self.get_parameter('scan_voxel_size').value
        self.freq_localization = self.get_parameter('freq_localization').value
        self.localization_th   = self.get_parameter('localization_th').value
        self.FOV               = self.get_parameter('fov').value
        self.FOV_FAR           = self.get_parameter('fov_far').value

        # ─── State Variables ─────────────────────────────────────
        self.global_map      = None
        self.map_points      = None   # 降采样后全局地图点 (N×3), crop 直接用
        self.map_normals     = None   # 与 map_points 对齐的法向量, 供 point-to-plane ICP
        self.initialized     = False
        self.T_map_to_odom   = np.eye(4)
        self.cur_odom        = None
        self.cur_scan        = None
        self.cur_scan_odom   = None  # 与 cur_scan 时间对齐的里程计

        # ─── Publishers ──────────────────────────────────────────
        self.pub_pc_in_map   = self.create_publisher(PointCloud2, '/cur_scan_in_map', 1)
        self.pub_submap      = self.create_publisher(PointCloud2, '/submap', 1)
        self.pub_map_to_odom = self.create_publisher(Odometry,     '/map_to_odom', 1)

        # ─── Subscriptions ───────────────────────────────────────
        self.create_subscription(PointCloud2,                  '/cloud_registered', self.cb_save_cur_scan, 1)
        self.create_subscription(Odometry,                    '/Odometry',         self.cb_save_cur_odom,  1)
        self._map_sub  = self.create_subscription(PointCloud2, '/global_map',               self.cb_init_map,      1)
        self._init_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.cb_init_pose,
            1
        )

        self.get_logger().info('GlobalLocalizationNode initialized.')

    def pc2_to_array(self, pc_msg: PointCloud2) -> np.ndarray:
        """PointCloud2 → (N×3) NumPy array, numpy 向量化解析 (避免逐点 Python 循环)"""
        names, formats, offsets = [], [], []
        for f in pc_msg.fields:
            dt = self._PF2NP[f.datatype]
            formats.append((dt, (f.count,)) if f.count > 1 else dt)
            names.append(f.name)
            offsets.append(f.offset)
        dtype = np.dtype({
            'names': names, 'formats': formats,
            'offsets': offsets, 'itemsize': pc_msg.point_step,
        })
        arr = np.frombuffer(pc_msg.data, dtype=dtype)
        pts = np.stack([arr['x'], arr['y'], arr['z']], axis=-1).astype(np.float32)
        if not pc_msg.is_dense:
            pts = pts[np.isfinite(pts).all(axis=1)]
        return pts

    def cb_init_map(self, msg: PointCloud2):
        pts = self.pc2_to_array(msg)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pcd = self.voxel_down_sample(pcd, self.map_voxel_size)
        # 法向量只在加载时算一次, 之后 crop 按掩码同步裁剪, 供 point-to-plane ICP 使用
        pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=self.map_voxel_size * 3.0, max_nn=30))
        self.global_map  = pcd
        self.map_points  = np.asarray(pcd.points)
        self.map_normals = np.asarray(pcd.normals)
        self.get_logger().info(
            f'Global map received: {len(self.map_points)} points after downsampling, normals precomputed.')
        self.destroy_subscription(self._map_sub)

    def cb_init_pose(self, msg: PoseWithCovarianceStamped):
        if self.global_map is None:
            self.get_logger().warn('Waiting for global map before initial localization.')
            return
        if self.initialized:
            return
        if self.cur_scan is None:
            self.get_logger().warn('Waiting for first scan before initial localization.')
            return

        initial = self.pose_to_mat(msg)
        success = self.global_localization(initial)
        if success:
            self.initialized = True
            period = 1.0 / self.freq_localization
            self.create_timer(period, self.timer_callback)
            self.get_logger().info('Initial global localization succeeded.')

    def cb_save_cur_odom(self, msg: Odometry):
        self.cur_odom = msg

    def cb_save_cur_scan(self, msg: PointCloud2):
        msg.header.frame_id = 'odom'
        self.pub_pc_in_map.publish(msg)

        pts = self.pc2_to_array(msg)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        self.cur_scan = pcd
        # scan 和 odom 在同一回调中原子性地配对，避免转圈时时间不同步导致的角度偏差
        self.cur_scan_odom = copy.deepcopy(self.cur_odom) if self.cur_odom is not None else None

    def timer_callback(self):
        self.global_localization(self.T_map_to_odom)

    def global_localization(self, pose_est):
        self.get_logger().info('Performing global localization via ICP...')
        scan_copy = copy.deepcopy(self.cur_scan)

        # 使用与 scan 时间对齐的里程计，而非可能不同步的 latest odom
        matched_odom = self.cur_scan_odom if self.cur_scan_odom is not None else self.cur_odom
        submap = self.crop_global_map_in_FOV(scan_copy, pose_est, matched_odom)

        vs = self.scan_voxel_size
        # 粗配准: 大体素 + point-to-point, 收敛域大、抗差
        T, _       = self.registration_at_scale(
            self.voxel_down_sample(scan_copy, vs * 5),
            self.voxel_down_sample(submap,    vs * 5),
            initial=pose_est, scale=5, point_to_plane=False)
        # 精配准: 正常体素 + point-to-plane, 精度高
        # submap 已带预计算法向量, 此处不再重采样 (voxel_down_sample 会丢法向量)
        T, fitness = self.registration_at_scale(
            self.voxel_down_sample(scan_copy, vs),
            submap,
            initial=T,        scale=1, point_to_plane=True)
        self.get_logger().info(f'ICP fitness: {fitness:.3f}')

        if fitness > self.localization_th:
            self.T_map_to_odom = T
            odom = Odometry()
            xyz  = tf_transformations.translation_from_matrix(T)
            quat = tf_transformations.quaternion_from_matrix(T)
            # 올바른 Odometry 메시지 필드 설정
            odom.pose.pose.position    = Point(x=xyz[0], y=xyz[1], z=xyz[2])
            odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            odom.header.stamp          = matched_odom.header.stamp if matched_odom is not None else self.cur_odom.header.stamp
            odom.header.frame_id       = 'map'
            self.pub_map_to_odom.publish(odom)
            return True

        self.get_logger().warn('Global localization failed (fitness below threshold).')
        return False

    def crop_global_map_in_FOV(self, scan, pose_est, odom):
        T_scan     = self.pose_to_mat(odom)
        T_map2scan = np.linalg.inv(pose_est @ T_scan)

        pts = self.map_points
        hom = np.hstack([pts, np.ones((pts.shape[0],1))])
        pts_scan = (T_map2scan @ hom.T).T

        if self.FOV >= 2*np.pi:
            mask = (pts_scan[:,0] < self.FOV_FAR)
        else:
            ang  = np.arctan2(pts_scan[:,1], pts_scan[:,0])
            mask = (pts_scan[:,0]>0)&(pts_scan[:,0]<self.FOV_FAR)&(np.abs(ang)<self.FOV/2)

        subpts = pts[mask]
        submap = o3d.geometry.PointCloud()
        submap.points  = o3d.utility.Vector3dVector(subpts)
        submap.normals = o3d.utility.Vector3dVector(self.map_normals[mask])

        header = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        cloud = pc2.create_cloud_xyz32(header, subpts[::10].tolist())
        self.pub_submap.publish(cloud)

        return submap

    def registration_at_scale(self, scan, submap, initial, scale, point_to_plane):
        if point_to_plane and submap.has_normals():
            estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()
        else:
            # point-to-plane 需要 target 法向量, 没有时退回 point-to-point
            estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        reg = o3d.pipelines.registration.registration_icp(
            scan, submap,
            max_correspondence_distance=1.0*scale,
            init=initial,
            estimation_method=estimation,
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
        )
        return reg.transformation, reg.fitness

    @staticmethod
    def pose_to_mat(pose_stamped):
        t = pose_stamped.pose.pose.position
        q = pose_stamped.pose.pose.orientation
        return tf_transformations.translation_matrix([t.x,t.y,t.z]) \
             @ tf_transformations.quaternion_matrix([q.x,q.y,q.z,q.w])

    @staticmethod
    def voxel_down_sample(pcd, vs):
        try:
            return pcd.voxel_down_sample(vs)
        except:
            return o3d.geometry.voxel_down_sample(pcd, vs)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
