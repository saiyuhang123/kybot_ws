#!/usr/bin/env python3
"""
kybot_bringup.launch.py
========================
统一启动 KYBOT 全部节点，只需一个终端窗口。
mission_executor、Nav2、IMU、FAST_LIO 会自动在独立终端窗口运行。

用法:
    source /home/nvidia/kybot_ws/install/setup.bash
    ros2 launch kybot_bringup bringup.launch.py

可选的 launch 参数:
    setup_can:=true      # 是否需要配置 CAN 总线 (默认 true，需免密 sudo)
    use_rviz:=false      # 是否启动 RViz (默认 false)
    use_sim_time:=false  # 是否使用仿真时间 (默认 false)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ========================
    # Launch Arguments
    # ========================
    declare_setup_can = DeclareLaunchArgument(
        "setup_can", default_value="true",
        description="是否配置 CAN1 接口 (需要 sudo 免密)"
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="false",
        description="是否启动 RViz2"
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="是否使用仿真时间"
    )
    declare_use_ocr = DeclareLaunchArgument(
        "use_ocr", default_value="false",
        description="是否启动海康相机 + OCR 识别"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ========================
    # 0. CAN 总线配置 (需要 sudo)
    # ========================
    can_setup_bitrate = ExecuteProcess(
        cmd=["sudo", "ip", "link", "set", "can1", "type", "can", "bitrate", "500000"],
        condition=IfCondition(LaunchConfiguration("setup_can")),
        name="can_setup_bitrate",
        output="screen",
    )
    can_setup_up = ExecuteProcess(
        cmd=["sudo", "ip", "link", "set", "can1", "up"],
        condition=IfCondition(LaunchConfiguration("setup_can")),
        name="can_setup_up",
        output="screen",
    )

    # ========================
    # 1. 底盘控制 (yhs_can_control)
    # ========================
    yhs_can_params = os.path.join(
        get_package_share_directory("yhs_can_control"),
        "params", "cfg.yaml"
    )
    yhs_can_control_node = Node(
        package="yhs_can_control",
        executable="yhs_can_control_node",
        name="yhs_can_control_node",
        output="screen",
        parameters=[yhs_can_params],
    )

    # ========================
    # 2. cmd_vel 桥接 (Nav2 → 底盘)
    # ========================
    cmd_vel_bridge_node = Node(
        package="yhs_can_control",
        executable="cmd_vel_bridge.py",
        name="cmd_vel_bridge",
        output="screen",
    )

    # ========================
    # 3. 二维激光雷达 (hi_ros2) — 引用原有 launch 文件
    # ========================
    hi_ros2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hi_ros2"),
                "launch", "hins_fe_launch.py"
            )
        )
    )

    # ========================
    # 4. 机械激光雷达 (rslidar_sdk)
    # ========================
    rslidar_node = Node(
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        name="rslidar_sdk_node",
        output="screen",
        parameters=[{"config_path": ""}],
    )

    # ========================
    # 5. 点云格式转换 (rslidar → velodyne)
    # ========================
    rs_to_velodyne_node = Node(
        package="rs_to_velodyne",
        executable="rs_to_velodyne",
        name="rs_to_velodyne",
        output="screen",
    )

    # ========================
    # 6. IMU (wit_ros2_imu) — 单独窗口运行，避免刷屏
    # ========================
    imu_node = ExecuteProcess(
        cmd=[
            "gnome-terminal", "--title=IMU", "--",
            "bash", "-c",
            "source /home/nvidia/kybot_ws/install/setup.bash && "
            "ros2 run wit_ros2_imu wit_ros2_imu "
            "--ros-args -p port:=/dev/ttyCH341USB0 -p baudrate:=921600; "
            "exec bash"
        ],
        name="imu_terminal",
    )

    # ========================
    # 7. Robot State Publisher (URDF)
    # ========================
    urdf_path = os.path.join(
        get_package_share_directory("yhs_chassis_description"),
        "urdf", "MK-mid.urdf"
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[urdf_path],
    )

    # ========================
    # 8. 静态 TF
    # ========================
    # base_link → laser_fe
    tf_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_laser_fe",
        arguments=[
            "--x", "0.7295",
            "--y", "0.0",
            "--z", "0.1885",
            "--yaw", "0.0",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "laser_fe",
        ],
    )
    # base_link → velodyne
    tf_velodyne = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_velodyne",
        arguments=[
            "--x", "0.2",
            "--y", "0.0",
            "--z", "0.5",
            "--yaw", "0.0",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "velodyne",
        ],
    )

    # ========================
    # 9. EKF (odom → base_link)
    # ========================
    ekf_config = os.path.join(
        get_package_share_directory("kybot_bringup"),
        "..", "..", "..", "..",
        "src", "robot_location2_ws", "param", "ekf_config.yaml"
    )
    # 更可靠的路径获取方式
    ekf_config_path = "/home/nvidia/kybot_ws/src/robot_location2_ws/param/ekf_config.yaml"
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_path,
                    {"use_sim_time": use_sim_time}],
    )

    # ========================
    # 10. 海康相机 + OCR (可选)
    # ========================
    ocr_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("kybot_bringup"),
                "launch", "ocr_camera.launch.py"
            )
        ),
        condition=IfCondition(LaunchConfiguration("use_ocr")),
    )

    # ========================
    # 11. 定位 (FAST_LIO) — 单独窗口运行，避免刷屏
    # ========================
    fast_lio_terminal = ExecuteProcess(
        cmd=[
            "gnome-terminal", "--title=FAST_LIO_Localization", "--",
            "bash", "-c",
            "source /home/nvidia/kybot_ws/install/setup.bash && "
            "ros2 launch fast_lio_localization velodyne_localization.launch.py "
            "use_sim_time:=false config_file:=velodyne_test.yaml rviz:=false; "
            "exec bash"
        ],
        name="fast_lio_terminal",
    )

    # ========================
    # 12. 海康相机 (hk_camera_node)
    #      use_ocr:=true 时由 ocr_camera.launch.py 接管，这里不重复启动
    # ========================
    hk_camera_node = Node(
        package="hk_camera",
        executable="hk_camera_node",
        name="hk_camera_node",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_ocr")),
        parameters=[{
            "ip": "192.168.1.64",
            "port": 8000,
            "username": "admin",
            "password": "a1234567",
            "channel": 1,
        }],
    )

    # ========================
    # 13. 任务执行器 (mission_executor) — 单独窗口
    # ========================
    mission_executor_terminal = ExecuteProcess(
        cmd=[
            "gnome-terminal", "--title=MissionExecutor", "--",
            "bash", "-c",
            "source /home/nvidia/kybot_ws/install/setup.bash && "
            "ros2 launch my_rviz_panel mission_executor.launch.py; "
            "exec bash"
        ],
        name="mission_executor_terminal",
    )

    # ========================
    # 14. Nav2 (nav2_bringup) — 单独窗口
    # ========================
    nav2_terminal = ExecuteProcess(
        cmd=[
            "gnome-terminal", "--title=Nav2", "--",
            "bash", "-c",
            "source /home/nvidia/kybot_ws/install/setup.bash && "
            "ros2 launch nav2_bringup bringup_launch.py "
            "use_sim_time:=False autostart:=True "
            "map:=/home/nvidia/Documents/PCD/pgm_yaml/test01.yaml "
            "params_file:=/home/nvidia/kybot_ws/src/nav2_params/param_top_akm_bs.yaml; "
            "exec bash"
        ],
        name="nav2_terminal",
    )

    # ========================
    # 组装 LaunchDescription
    # ========================
    ld = LaunchDescription()

    # 参数声明
    ld.add_action(declare_setup_can)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_ocr)

    # CAN 配置
    ld.add_action(can_setup_bitrate)
    ld.add_action(can_setup_up)

    # 静态 TF（轻量，立即启动）
    ld.add_action(tf_laser)
    ld.add_action(tf_velodyne)

    # 逐个延迟 2s 启动，避免 CPU 过载
    ld.add_action(TimerAction(period=2.0,  actions=[yhs_can_control_node]))
    ld.add_action(TimerAction(period=4.0,  actions=[cmd_vel_bridge_node]))
    ld.add_action(TimerAction(period=6.0,  actions=[hi_ros2_launch]))
    ld.add_action(TimerAction(period=8.0,  actions=[rslidar_node]))
    ld.add_action(TimerAction(period=10.0, actions=[rs_to_velodyne_node]))
    ld.add_action(TimerAction(period=12.0, actions=[robot_state_pub_node]))
    ld.add_action(TimerAction(period=14.0, actions=[imu_node]))
    ld.add_action(TimerAction(period=16.0, actions=[ekf_node]))
    ld.add_action(TimerAction(period=18.0, actions=[fast_lio_terminal]))
    ld.add_action(TimerAction(period=20.0, actions=[ocr_camera_launch]))
    ld.add_action(TimerAction(period=21.0, actions=[hk_camera_node]))
    ld.add_action(TimerAction(period=22.0, actions=[mission_executor_terminal]))
    ld.add_action(TimerAction(period=26.0, actions=[nav2_terminal]))

    return ld
