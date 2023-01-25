from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='battery_imu_mon',
            executable='batteryimu',
            name='batteryimu',
            arguments=['--ros-args', '--log-level', 'WARN'],
    ),
    ])