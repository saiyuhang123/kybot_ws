import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    realsense_share = get_package_share_directory('realsense2_camera')

    d435_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'rs_launch.py')),
        launch_arguments={
            'camera_namespace': 'front_camera',
            'enable_color': 'true',
            'enable_depth': 'true',
            'rgb_camera.color_profile': '640x480x30',
            'depth_module.depth_profile': '640x480x30',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'false',
            'initial_reset': 'true',
        }.items(),
    )

    perception_node = Node(
        package='trash_mission',
        executable='front_perception_node',
        name='front_perception_node',
        output='screen',
        parameters=[{
            'model_path': '/home/nvidia/Documents/elite_robot_ws/YOLO/yolov8n.pt',
            'prompts': 'bottle',
            'show': True,
            'color_topic': '/front_camera/camera/color/image_raw',
            'conf': 0.25,
            'infer_every': 1,
        }],
    )

    return LaunchDescription([
        d435_launch,
        perception_node,
    ])
