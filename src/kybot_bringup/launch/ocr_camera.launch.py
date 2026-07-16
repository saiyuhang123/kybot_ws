#!/usr/bin/env python3
"""
ocr_camera.launch.py
====================
联合启动海康相机驱动 + OCR 识别节点。

用法:
    source /home/nvidia/kybot_ws/install/setup.bash

    # 按需识别模式（默认，省算力）
    ros2 launch kybot_bringup ocr_camera.launch.py

    # 流式识别模式（持续识别 + 服务仍可用）
    ros2 launch kybot_bringup ocr_camera.launch.py enable_streaming:=true

    # 指定相机 IP
    ros2 launch kybot_bringup ocr_camera.launch.py ip:=192.168.1.100

    # 英文识别 + GPU 加速
    ros2 launch kybot_bringup ocr_camera.launch.py lang:=en use_gpu:=true

节点关系:
    hk_camera_node ──/hk_camera/image_raw──▶ ocr_node
                                          └── /ocr/recognize (service)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ========================
    # 海康相机参数
    # ========================
    ip_arg = DeclareLaunchArgument(
        'ip', default_value='192.168.1.64',
        description='海康相机 IP 地址'
    )
    port_arg = DeclareLaunchArgument(
        'port', default_value='8000',
        description='海康相机端口'
    )
    username_arg = DeclareLaunchArgument(
        'username', default_value='admin',
        description='设备用户名'
    )
    password_arg = DeclareLaunchArgument(
        'password', default_value='a1234567',
        description='设备密码'
    )
    channel_arg = DeclareLaunchArgument(
        'channel', default_value='1',
        description='通道号'
    )

    # ========================
    # OCR 参数
    # ========================
    lang_arg = DeclareLaunchArgument(
        'lang', default_value='ch',
        description='OCR 语言: ch (中英) / en (英文)'
    )
    use_gpu_arg = DeclareLaunchArgument(
        'use_gpu', default_value='false',
        description='GPU 加速 (Jetson 建议 false)'
    )
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold', default_value='0.5',
        description='置信度阈值 (0.0 - 1.0)'
    )
    enable_streaming_arg = DeclareLaunchArgument(
        'enable_streaming', default_value='false',
        description='开启流式识别 (true/false)'
    )
    processing_interval_arg = DeclareLaunchArgument(
        'processing_interval', default_value='0.5',
        description='流式识别最小间隔 (秒)'
    )
    enable_annotated_arg = DeclareLaunchArgument(
        'enable_annotated_img', default_value='true',
        description='流式模式下发布标注预览图'
    )

    # ========================
    # 节点 1: 海康相机驱动
    # ========================
    hk_camera_node = Node(
        package='hk_camera',
        executable='hk_camera_node',
        name='hk_camera_node',
        output='screen',
        parameters=[{
            'ip': LaunchConfiguration('ip'),
            'port': LaunchConfiguration('port'),
            'username': LaunchConfiguration('username'),
            'password': LaunchConfiguration('password'),
            'channel': LaunchConfiguration('channel'),
        }],
    )

    # ========================
    # 节点 2: OCR 识别
    # ========================
    ocr_node = Node(
        package='ocr_node',
        executable='ocr_node',
        name='ocr_node',
        output='screen',
        parameters=[{
            'lang': LaunchConfiguration('lang'),
            'use_gpu': LaunchConfiguration('use_gpu'),
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'enable_streaming': LaunchConfiguration('enable_streaming'),
            'processing_interval': LaunchConfiguration('processing_interval'),
            'enable_annotated_img': LaunchConfiguration('enable_annotated_img'),
        }],
    )

    # ========================
    # 组装
    # ========================
    return LaunchDescription([
        # 相机参数
        ip_arg,
        port_arg,
        username_arg,
        password_arg,
        channel_arg,
        # OCR 参数
        lang_arg,
        use_gpu_arg,
        conf_threshold_arg,
        enable_streaming_arg,
        processing_interval_arg,
        enable_annotated_arg,
        # 日志
        LogInfo(msg=['📷 启动海康相机 + OCR 识别...']),
        # 节点（相机先启动，确保 OCR 节点能收到帧）
        hk_camera_node,
        ocr_node,
    ])
