from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kitware',
            executable='kitbot.py',
            output='screen'
        ),
        Node(
            package='kitware',
            executable='vision.py',
            output='screen'
        ),
    ])

