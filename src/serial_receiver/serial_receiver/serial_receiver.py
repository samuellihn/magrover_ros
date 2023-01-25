import asyncio
import datetime
import os
import threading
from typing import Union, Callable, Any

import rclpy
import serial
import websockets

from rclpy.node import Node

# Enables usage of the String message type
from std_msgs.msg import Float64
from sensor_msgs.msg import BatteryState, Imu
from geometry_msgs.msg import Vector3, Twist

from serial_receiver.messages_pb2 import Command, Response
from serial_receiver.transceiver_base import TransceiverBase



class SerialTransceiver(TransceiverBase):

    socket = serial.Serial()
    def __init__(self, recv_callback: Callable[[bytearray], Any]):
        super().__init__(recv_callback)
        self.socket.baudrate = 115200
        self.socket.port = "/dev/ttyUSB0"
        self.socket.timeout = 0.1

    def recv(self, bytes_to_read: int):
        buffer = bytearray()
        with self.lock:
            buffer = self.socket.read(bytes_to_read)
        return buffer

    def connect(self):
        self.socket.open()
        return True

    def send(self, data: Union[bytearray, str]):
        """Transmits a string or bytearray."""
        data = bytearray(data, "utf8") if isinstance(data, str) else data
        data_length = len(data).to_bytes(4, 'big', signed=False)
        packet = self.header + data_length + data
        checksum = ~(sum(packet)) & 0xFFFF
        packet += checksum.to_bytes(2, "big", signed=False)
        print(packet)
        with self.lock:
            self.socket.write(packet)

    def __del__(self):
        super().__del__()
        self.socket.close()

class SerialTransceiverNode(Node):

    def __init__(self):
        super().__init__('serial_receiver')

        self.drive_publisher = self.create_publisher(Twist, 'drive', 10)
        self.lift_publisher = self.create_publisher(Float64, 'lift', 10)

        self.transceiver = SerialTransceiver(self.recv_callback)
        self.transceiver.connect()
        self.timer = self.create_timer(0.5, self.transceiver.read_packet)
        print("Connected")


    def recv_callback(self, message):
        print(message)

        command = Command()
        command.ParseFromString(bytes(message))
        self.dispatch_message(command)

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
        print(drive_packet)

        self.drive_publisher.publish(drive_packet)
        self.lift_publisher.publish(lift_packet)
        self.previous_drive = drive_packet
        self.previous_lift = lift_packet


def main(args=None):
    # trans = SerialTransceiver(print)
    # trans.connect()
    # trans.start_recv()
    # while True:
    #     msg = input()
    #     trans.send(msg)
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    transceiver = SerialTransceiverNode()

    # Spin the node so the callback function is called.
    rclpy.spin(transceiver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    transceiver.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
