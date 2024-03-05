from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab1_pkg',
            executable='talker',
            parameters=[
                {'speed': 0.5},
                {'steering_angle': 1.5},
            ],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='lab1_pkg',
            executable='relay',
            output='screen',
            emulate_tty=True
        )
    ])