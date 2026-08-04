#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2

#这段代码就是单纯的探路先锋。depth_value没有被其他模块调用过，所以它的功能就是单纯的订阅深度图像或点云数据，
# 并从中提取中心像素的深度值进行打印输出。它没有被其他模块调用过，因此它的功能相对独立和简单，主要用于测试和验证深度数据的获取是否正常。

class DepthReader:
    def __init__(self):
        # 选择您需要的数据类型（取消其中一个的注释）
        
        # 选项1: 接收深度图像
        # Image：指定了该话题中流动的数据包类型。这通常是从 sensor_msgs.msg 导入的标准 ROS 传感器图像消息
        # 这个话题不是由某个“设计系统”创建的，而是由 Intel RealSense 相机的 ROS 驱动包（realsense-ros） 在底层硬件驱动与 ROS 系统之间建立的桥梁中自动创建并发布的。
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_image_callback)
        
        # 选项2: 接收点云数据
        # rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.pointcloud_callback)
        
        rospy.loginfo("深度读取器已启动...")

    def depth_image_callback(self, msg):
        """处理深度图像 (sensor_msgs/Image)"""
        try:
            # 直接提取原始字节数据
            raw_data = msg.data
            # 转换为16位数组 (假设是16UC1格式)
            import struct
            depth_image = [struct.unpack_from('H', raw_data, i)[0] 
                          for i in range(0, len(raw_data), 2)]
            
            height = msg.height
            width = msg.width
            center_y = height // 2
            center_x = width // 2
            
            # 获取中心深度值 (单位:mm)
            center_idx = center_y * width + center_x
            depth_value = depth_image[center_idx]
            
            rospy.loginfo(f"中心深度值: {depth_value} mm")
        
        except Exception as e:
            rospy.logerr(f"深度图像处理错误: {str(e)}")

    def pointcloud_callback(self, msg):
        """处理点云数据 (sensor_msgs/PointCloud2)"""
        try:
            # 获取中心点索引
            width = msg.width
            height = msg.height
            center_idx = (height // 2) * width + (width // 2)
            
            # 读取中心点的深度值
            points = pc2.read_points_list(
                msg, 
                field_names=('z'), 
                skip_nans=True
            )
            
            if len(points) > center_idx:
                z_value = points[center_idx][0]  # Z坐标即深度
                rospy.loginfo(f"中心点深度: {z_value:.3f} m")
            else:
                rospy.logwarn("无法获取中心点深度值")
                
        except Exception as e:
            rospy.logerr(f"点云处理错误: {str(e)}")

if __name__ == '__main__':
    rospy.init_node('d435_depth_reader')
    dr = DepthReader()
    rospy.spin()

