#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('chandni_safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        self.iTTC = self.declare_parameter('ttc', 1.5).get_parameter_value().double_value


        mode = self.declare_parameter('mode').get_parameter_value().string_value
        if mode == 'sim':
            self.odom_mode = 'ego_racecar/odom'
        elif mode == 'car':
            self.odom_mode = '/odom'
        # else:
        #     self.odom_mode = '/odom'  # don't know what the real name is 'car'?

        # TODO: create ROS subscribers and publishers.
        # scan subscription
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.scan_subscription

        # odom subscription 
        self.odom_subscription = self.create_subscription(
            Odometry,
            # 'ego_racecar/odom',
            self.odom_callback,
            self.odom_callback,
            10)
        self.odom_subscription
        
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        my_ranges = scan_msg.ranges
        threshold = self.iTTC

        for i, range_value in enumerate(my_ranges):
            # Calculate the angle for the current range
            theta = scan_msg.angle_min + i * scan_msg.angle_increment
            range_rate = self.speed * np.cos(theta)

            # Determine time to collision
            if range_rate > 0:
                ttc = range_value / range_rate  # Positive range rate means distance is decreasing
            else:
                ttc = np.inf  # Negative range rate means distance is increasing

            # Publish braking command if time to collision is below the threshold
            if ttc < threshold:
                ack_msg = AckermannDriveStamped()
                ack_msg.drive.speed = 0.0
                self.publisher_.publish(ack_msg)
                self.get_logger().info('CHANDNI-BREAKING!')

        

def main(args=None):
    rclpy.init(args=args)
    chandni_safety_node = SafetyNode()
    rclpy.spin(chandni_safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    chandni_safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
