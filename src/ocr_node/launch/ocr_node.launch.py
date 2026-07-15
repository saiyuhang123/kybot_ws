"""
Launch file for ocr_node.

Usage:
  ros2 launch ocr_node ocr_node.launch.py
  ros2 launch ocr_node ocr_node.launch.py lang:=en processing_interval:=0.2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- Declare launch arguments ----
    lang_arg = DeclareLaunchArgument(
        'lang',
        default_value='ch',
        description='OCR language: ch (Chinese+English) or en (English only)',
    )
    use_gpu_arg = DeclareLaunchArgument(
        'use_gpu',
        default_value='true',
        description='Use GPU acceleration (true/false)',
    )
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.5',
        description='Minimum confidence threshold (0.0 - 1.0)',
    )
    processing_interval_arg = DeclareLaunchArgument(
        'processing_interval',
        default_value='0.5',
        description='Minimum seconds between OCR runs',
    )
    enable_annotated_arg = DeclareLaunchArgument(
        'enable_annotated_img',
        default_value='true',
        description='Publish annotated preview image (true/false)',
    )

    # ---- Node ----
    ocr_node = Node(
        package='ocr_node',
        executable='ocr_node',
        name='ocr_node',
        output='screen',
        parameters=[{
            'lang': LaunchConfiguration('lang'),
            'use_gpu': LaunchConfiguration('use_gpu'),
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'processing_interval': LaunchConfiguration('processing_interval'),
            'enable_annotated_img': LaunchConfiguration('enable_annotated_img'),
        }],
    )

    # ---- Info message ----
    info = LogInfo(
        msg='🚀 OCR Node — listening on /hk_camera/image_raw',
    )

    return LaunchDescription([
        lang_arg,
        use_gpu_arg,
        conf_threshold_arg,
        processing_interval_arg,
        enable_annotated_arg,
        info,
        ocr_node,
    ])
