from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bt_receiver',
            executable='bt_receiver',
            name='bt_receiver',
            arguments=['--ros-args', '--log-level', 'WARN'],
        ),
    ])