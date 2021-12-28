from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kitware',
            executable='kitbot.py',
        ),
        Node(
            package='kitware',
            executable='kbd_driver.py',
        ),
    ])

