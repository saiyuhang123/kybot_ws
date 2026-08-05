import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('qwen_vision'),
        'config', 'qwen_vision.yaml')

    return LaunchDescription([
        Node(
            package='qwen_vision',
            executable='qwen_point_node',
            name='qwen_point',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='qwen_vision',
            executable='qwen_vision_node',
            name='qwen_vision',
            output='screen',
            parameters=[params_file],
        ),
    ])
