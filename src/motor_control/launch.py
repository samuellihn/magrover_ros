from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_control',
            executable='motor_control',
            name='motor_control',
            arguments=['--ros-args', '--log-level', 'WARN'],
        ),
    ])