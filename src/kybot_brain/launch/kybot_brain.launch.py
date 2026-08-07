import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('kybot_brain'),
        'config', 'brain.yaml')

    return LaunchDescription([
        Node(
            package='kybot_brain',
            executable='brain_node',
            name='brain_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
