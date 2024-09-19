#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Bool # added from bree_safety_node


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('allison_safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """

        mode = self.declare_parameter('mode').get_parameter_value().string_value
        if mode == 'sim':
            self.odom_mode = 'ego_racecar/odom'
        elif mode == 'car':
            self.odom_mode = '/odom'
        # else:
        #     self.odom_mode = '/odom'  # don't know what the real name is 'car'?

        # TODO: create ROS subscribers and publishers.
        # Publish to the /drive topic
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        
        # Subscribe to the /ego_racecar/odom topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            # 'ego_racecar/odom',
            self.odom_mode,
            self.odom_callback,
            10)
            
        # Subscribe to the /scan topic
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        # Prevent unused variable warnings
        self.odom_subscription
        self.scan_subscription
        
        # Declare and set parameter so we can choose max permissible ttc from the command line
        self.ttc = self.declare_parameter('ttc', 2.0).get_parameter_value().double_value
        self.speed = 0.
        

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        threshold = self.ttc
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max
        angle_inc = scan_msg.angle_increment
        ranges = scan_msg.ranges
        rangeNum = len(ranges)
        
        # TODO: calculate TTC
        for i in range(rangeNum):
            theta = angle_min + i * angle_inc
            denominator = np.cos(theta) * self.speed
            if denominator <= 0:
                denominator = 0 # {denominator}+ or max(denominator, 0)
                iTTC = np.inf
            else:
                iTTC = ranges[i] / denominator
        
        # TODO: publish command to brake
            if  iTTC < threshold:
                msg = AckermannDriveStamped()
                msg.drive.speed = 0.0
                self.publisher_.publish(msg)
                self.get_logger().info(f'ALLISON EMERGENCY BREAKING')

def main(args=None):
    rclpy.init(args=args)
    allison_safety_node = SafetyNode()
    rclpy.spin(allison_safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    allison_safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
