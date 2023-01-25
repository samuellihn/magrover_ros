from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_receiver',
            executable='serial_receiver',
            name='serial_receiver',
            arguments=['--ros-args', '--log-level', 'INFO'],
    ),
    ])