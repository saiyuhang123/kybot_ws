#!/usr/bin/env python3
"""
trash_pipeline.launch.py
========================
感知 + 底盘侧一键启动（巡检自动抓瓶全程用）。

包含：
  1. 底盘全系统：kybot_bringup/bringup.launch.py
     （CAN/底盘控制/cmd_vel桥/二维激光/RS-LiDAR/点云转换/IMU/EKF/
       FAST_LIO定位/海康相机/mission_executor/Nav2，自动开独立终端窗口）
  2. 车前感知：trash_mission（D435 + 前方 YOLO 检测 → /trash/target）
  3. 可选 RViz 面板（默认开启，加载 ~/.rviz2/default.rviz 含 MyPanel）

用法：
    source /home/nvidia/kybot_ws/install/setup.bash
    ros2 launch kybot_bringup trash_pipeline.launch.py

参数：
    setup_can:=true      # 配置 CAN（需免密 sudo）
    use_sim_time:=false  # 是否使用仿真时间
    use_ocr:=false       # 是否启动海康相机 + OCR 识别
    use_rviz:=true       # 是否启动 RViz 面板
    rviz_config:=/home/nvidia/.rviz2/default.rviz
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "setup_can", default_value="true",
            description="是否配置 CAN 接口（需要 sudo 免密）",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="是否使用仿真时间",
        ),
        DeclareLaunchArgument(
            "use_ocr", default_value="false",
            description="是否启动海康相机 + OCR 识别",
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="是否启动 RViz 面板（含 MyPanel）",
        ),
        DeclareLaunchArgument(
            "rviz_config", default_value="/home/nvidia/.rviz2/default.rviz",
            description="RViz 配置文件路径",
        ),
    ]

    setup_can = LaunchConfiguration("setup_can")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ocr = LaunchConfiguration("use_ocr")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    # ============================================================
    # 1. 底盘全系统（内部已按 0~26s 分步启动，Nav2/mission_executor
    #    会各自弹出独立终端窗口）
    # ============================================================
    chassis_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("kybot_bringup"), "launch"]
                ),
                "/bringup.launch.py",
            ]
        ),
        launch_arguments={
            "setup_can": setup_can,
            "use_sim_time": use_sim_time,
            "use_ocr": use_ocr,
        }.items(),
    )

    # ============================================================
    # 2. 车前感知：D435 + 前方 YOLO（/trash/target）
    # ============================================================
    trash_mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("trash_mission"), "launch"]
                ),
                "/trash_mission.launch.py",
            ]
        )
    )

    # ============================================================
    # 3. RViz 面板（MyPanel 插件在 default.rviz 里）
    # ============================================================
    rviz_panel = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        declared_arguments
        + [
            LogInfo(msg="=" * 60),
            LogInfo(msg="感知 + 底盘侧一键启动（trash_pipeline）"),
            LogInfo(msg="  底盘/定位/Nav2/mission_executor → 独立终端窗口"),
            LogInfo(msg="  D435 + 前方 YOLO 感知 → /trash/target"),
            LogInfo(msg="=" * 60),
            # 底盘全系统立即开始（内部自带分步延时）
            chassis_bringup,
            # D435 + 感知在底盘起来后启动，避免开机瞬间 CPU 过载
            TimerAction(period=12.0, actions=[trash_mission]),
            # RViz 面板最后起
            TimerAction(period=16.0, actions=[rviz_panel]),
            TimerAction(
                period=28.0,
                actions=[
                    LogInfo(msg="=" * 60),
                    LogInfo(msg="全部启动完成："),
                    LogInfo(msg="  机械臂侧请另开：ros2 launch "
                                "~/Documents/elite_robot_ws/biaoding/"
                                "yolo_grasp_two_finger.launch.py"),
                    LogInfo(msg="  验证：ros2 service list | grep yolo_grasp"),
                    LogInfo(msg="=" * 60),
                ],
            ),
        ]
    )
