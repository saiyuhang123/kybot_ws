import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('robot_aiui'), 'config')
    goals_yaml = os.path.join(config_dir, 'goals.yaml')

    return LaunchDescription([
        Node(
            package='robot_aiui',
            executable='robot_aiui_node',
            name='robot_aiui',
            output='screen',
            parameters=[goals_yaml],
        ),
    ])
