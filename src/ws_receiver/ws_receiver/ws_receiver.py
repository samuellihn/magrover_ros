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
from ws_receiver.messages_pb2 import Command, Response

WS_PORT = 5000
WS_IP = "0.0.0.0"


class WsReceiver(Node):

    def __init__(self):
        super().__init__('battery_imu_pub')

        self.drive_publisher = self.create_publisher(Twist, 'drive', 10)
        self.lift_publisher = self.create_publisher(Float64, 'lift', 10)
        asyncio.run(self.serve())

    async def serve(self):
        async def on_connection(socket):
            async for message in socket:
                command = Command()
                command.ParseFromString(message)
                self.dispatch_message(command)

        async with websockets.serve(on_connection, WS_IP, WS_PORT):
            await asyncio.Future()  # run forever

    def dispatch_message(self, command):
        self.dispatch_teleop(command.teleop)

    previous_drive = Twist()
    previous_lift = Float64()

    def dispatch_teleop(self, teleop):
        drive_packet = Twist()
        lift_packet = Float64()

        drive_packet.linear.x = self.previous_drive.linear.x if teleop.drive_x == -2 else teleop.drive_x
        drive_packet.linear.y = self.previous_drive.linear.y if teleop.drive_y == -2 else teleop.drive_y
        lift_packet.data = self.previous_lift.data if teleop.lift == -2 else teleop.lift

        self.drive_publisher.publish(drive_packet)
        self.lift_publisher.publish(lift_packet)
        self.previous_drive = drive_packet
        self.previous_lift = lift_packet


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    battery_publisher = WsReceiver()

    # Spin the node so the callback function is called.
    rclpy.spin(battery_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    battery_publisher.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
