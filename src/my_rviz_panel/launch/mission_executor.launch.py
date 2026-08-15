from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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
                'max_bottle_interrupts_per_waypoint': 2,
                'bottle_candidate_frames': 3,
            }],
        ),
    ])
