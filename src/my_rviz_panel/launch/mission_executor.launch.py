from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    end_effector_mode = LaunchConfiguration('end_effector_mode')
    return LaunchDescription([
        DeclareLaunchArgument(
            'end_effector_mode', default_value='twofinger',
            choices=['twofinger', 'linkerhand', 'polish'],
            description='实际安装末端: twofinger/linkerhand/polish'),
        Node(
            package='my_rviz_panel',
            executable='mission_executor',
            name='mission_executor',
            output='screen',
            parameters=[{
                # 抓取组合动作: standoff 点到位后的直线逼近/退回距离与速度
                'approach_distance': 1.5,   # 米 (最大走距, 实际由 front_stop_distance 决定停车)
                'approach_speed': 0.1,      # 米/秒
                'front_stop_distance': 0.70,
                'scan_timeout': 0.5,
                'odom_timeout': 0.30,
                'front_scan_min_angle': -10.0,
                'front_scan_max_angle': 10.0,
                # M3 自动捡瓶参数
                'bottle_stop_distance': 0.55,   # D435 停车距离（米，按实测标定）
                'bottle_approach_speed': 0.15,  # 接近速度上限（米/秒）
                'bottle_confirm_timeout': 5.0,  # 停车确认超时（秒）
                'bottle_interrupt_distance': 2.0,  # 运动中检测到瓶子后，距离到这个值才中断停车
                'max_bottle_interrupts_per_waypoint': 10,
                'bottle_candidate_frames': 3,
                'use_map_goal_approach': True,
                'approach_goal_timeout': 3.0,
                # 物理末端模式硬门：不允许打磨头和夹爪动作串用
                'end_effector_mode': end_effector_mode,
                # 命令3含视觉、接触搜索和整段打磨，明显长于普通抓取
                # 桥接内部 900s 工艺超时后还会最多等 90s 安全取消收尾。
                'polish_timeout_sec': 1020.0,
            }],
        ),
    ])
