from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qwen_vision',
            executable='qwen_point_node',
            name='qwen_point',
            output='screen',
            parameters=[{
                'api_key': 'sk-f4a060eef3c747fc906b4a49731beb5c',
                'model': 'qwen-vl-max-latest',
                'base_url': 'https://dashscope.aliyuncs.com/compatible-mode/v1',
            }],
        ),
        Node(
            package='qwen_vision',
            executable='qwen_vision_node',
            name='qwen_vision',
            output='screen',
            parameters=[{
                'api_key': 'sk-f4a060eef3c747fc906b4a49731beb5c',
                'model': 'qwen-vl-max-latest',
                'base_url': 'https://dashscope.aliyuncs.com/compatible-mode/v1',
            }],
        ),
    ])
