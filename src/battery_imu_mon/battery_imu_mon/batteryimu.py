import os
import threading

import rclpy

from rclpy.node import Node
from ina260.controller import Controller as INA260
from icm20948 import ICM20948
from math import radians
from threading import Lock

# Enables usage of the String message type
from std_msgs.msg import Float64
from sensor_msgs.msg import BatteryState, Imu
from geometry_msgs.msg import Vector3, Twist

BATTERY_POLLING_RATE_HZ = 0.1
IMU_POLLING_RATE_HZ = 10
BATTERY_LOW_THRESHOLD = 6.5
BATTERY_CUTOFF_THRESHOLD = 6.0
SYSTEM_SHUTDOWN_DELAY_MINUTES = 1

IMU_SWITCH_AXES_FN = lambda x, y, z: (-z, y, x)
GRAVITY = 9.80665


def g_to_ms2(g):
    """
    Converts an acceleration value in g to acceleration value in m/s^2
    :param g: Acceleration value in g
    :return: Acceleration value in m/s^2
    """
    return g * GRAVITY


class BatteryImuPub(Node):
    """
    Create a MinimalPublisher class, which is a subclass of the Node class.
    """

    def __init__(self):
        super().__init__('battery_imu_pub')

        self.i2c_lock = Lock()

        # Hardware initialization
        self.imu = ICM20948(i2c_addr=0x69)


        # Node initialization

        self.battery_publisher = self.create_publisher(BatteryState, 'battery', 10)
        self.battery_pub_timer = self.create_timer(1 / BATTERY_POLLING_RATE_HZ, self.battery_callback)

        self.imu_publisher = self.create_publisher(Imu, "imu", 10)
        self.imu_pub_timer = self.create_timer(1 / IMU_POLLING_RATE_HZ, self.imu_callback)

    def imu_callback(self):
        imu_msg = Imu()

        # No orientation estimate, so set covariance matrix to this
        imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.i2c_lock.acquire()
        ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro_data()
        self.i2c_lock.release()

        # Hardware specific, swaps axes in accordance with IMU mounting orientation
        ax, ay, az = IMU_SWITCH_AXES_FN(ax, ay, az)
        gx, gy, gz = IMU_SWITCH_AXES_FN(gx, gy, gz)

        # Perform unit conversion, library returns wrong units for ROS IMU node
        ax, ay, az = map(g_to_ms2, (ax, ay, az))
        gx, gy, gz = map(radians, (gx, gy, gz))

        acc_vec = Vector3()
        acc_vec.x = ax
        acc_vec.y = ay
        acc_vec.z = az
        gyro_vec = Vector3()
        gyro_vec.x = gx
        gyro_vec.y = gy
        gyro_vec.z = gz

        imu_msg.linear_acceleration = acc_vec
        imu_msg.angular_velocity = gyro_vec

        # No covariance estimates, so zero out covariance matrices in accordance to message spec
        imu_msg.linear_acceleration_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        imu_msg.angular_velocity_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.imu_publisher.publish(imu_msg)
        self.get_logger().debug(
            f"Ax: {ax:6.5f}, Ay: {ay:6.5f}, Az: {az:6.5f}. Gx: {gx:6.5f}, Gy: {gy:6.5f}, Gz: {gz:6.5f}.")

    def battery_callback(self):
        """
        Callback function.
        This function gets called every 0.5 seconds.
        """
        # Create a String message
        battery_msg = BatteryState()
        # Battery metadata
        battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_NIMH
        battery_msg.present = True

        self.i2c_lock.acquire()
        ina260 = INA260(address=0x40)
        battery_msg.voltage = ina260.voltage()
        battery_msg.current = ina260.current()
        ina260.__del__()

        self.i2c_lock.release()

        # Publish the message to the topic
        self.battery_publisher.publish(battery_msg)

        # Display the message on the console
        if battery_msg.voltage < BATTERY_CUTOFF_THRESHOLD:
            self.get_logger().error(
                f"Battery voltage below cutoff voltage. System will shut down in {SYSTEM_SHUTDOWN_DELAY_MINUTES} minutes. Voltage: {battery_msg.voltage:5.4f} V. Current: {battery_msg.current:5.4f} A."

            )
            os.system(f"sudo shutdown +{SYSTEM_SHUTDOWN_DELAY_MINUTES}")
        elif battery_msg.voltage < BATTERY_LOW_THRESHOLD:
            self.disable_motors()
            self.get_logger().warn(
                f"Warning: Battery voltage low. Please shut down soon to ensure safe power off. Motors have been disabled. Voltage: {battery_msg.voltage:5.4f} V. Current: {battery_msg.current:5.4f} A."
            )
        else:
            self.get_logger().info(f"Voltage: {battery_msg.voltage:5.4f} V. Current: {battery_msg.current:5.4f} A.")
    def disable_motors(self):
        self.drive_publisher = self.create_publisher(Twist, 'drive', 10)
        self.lift_publisher = self.create_publisher(Float64, 'lift', 10)
        # Zero out drive and lift
        self.drive_publisher.publish(Twist())
        self.lift_publisher.publish(Float64())




def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    battery_publisher = BatteryImuPub()

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
