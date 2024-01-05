from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kitware',
            executable='kitbot',
            output='screen'
        ),
        Node(
            package='kitware',
            executable='kbd_driver',
            output='screen'
        ),
    ])

