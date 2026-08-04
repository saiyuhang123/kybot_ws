#!/usr/bin/env python3
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2


class DepthReader(Node):
    def __init__(self):
        super().__init__('d435_depth_reader')
        self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_image_callback, 1)
        # self.create_subscription(PointCloud2, '/camera/depth/color/points', self.pointcloud_callback, 1)
        self.get_logger().info('深度读取器已启动...')

    def depth_image_callback(self, msg):
        try:
            raw_data = msg.data
            depth_image = [struct.unpack_from('H', raw_data, i)[0]
                          for i in range(0, len(raw_data), 2)]

            height = msg.height
            width = msg.width
            center_y = height // 2
            center_x = width // 2

            center_idx = center_y * width + center_x
            depth_value = depth_image[center_idx]

            self.get_logger().info(f'中心深度值: {depth_value} mm')
        except Exception as e:
            self.get_logger().error(f'深度图像处理错误: {str(e)}')

    def pointcloud_callback(self, msg):
        try:
            width = msg.width
            height = msg.height
            center_idx = (height // 2) * width + (width // 2)

            points = pc2.read_points_list(msg, field_names=('z',), skip_nans=True)

            if len(points) > center_idx:
                z_value = points[center_idx][0]
                self.get_logger().info(f'中心点深度: {z_value:.3f} m')
            else:
                self.get_logger().warn('无法获取中心点深度值')
        except Exception as e:
            self.get_logger().error(f'点云处理错误: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = DepthReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
