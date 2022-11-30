from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mag_array',
            executable='mag_array',
            name='mag_array',
            arguments=['--ros-args', '--log-level', 'DEBUG'],
            # prefix="valgrind",
        ),
    ])