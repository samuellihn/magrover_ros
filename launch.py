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
        Node(
            package='battery_imu_mon',
            executable='batteryimu',
            name='batteryimu',
            arguments=['--ros-args', '--log-level', 'WARN'],
        ),
        Node(
            package='bt_receiver',
            executable='bt_receiver',
            name='bt_receiver',
            arguments=['--ros-args', '--log-level', 'WARN'],
        ),
        Node(
            package='mag_array',
            executable='mag_array',
            name='mag_array',
            arguments=['--ros-args', '--log-level', 'WARN'],
        )
    ])
