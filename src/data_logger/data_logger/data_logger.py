import asyncio
import os
import threading

import rclpy
import websockets

from rclpy.node import Node

# Enables usage of the String message type
from std_msgs.msg import Float64
from sensor_msgs.msg import BatteryState, Imu
from geometry_msgs.msg import Vector3, Twist
from data_logger.messages_pb2 import Command, Response


class DataLogger(Node):

    def __init__(self):
        self.create_subscription()






def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    node = WsReceiver()

    # Spin the node so the callback function is called.
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
