#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


import numpy as np


from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Bool


class SafetyNode(Node):
   """
   The class that handles emergency braking.
   """
   def __init__(self):
       super().__init__('bree_safety_node')
       """
       One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.


       You should also subscribe to the /scan topic to get the LaserScan messages and
       the /ego_racecar/odom topic to get the current speed of the vehicle.


       The subscribers should use the provided odom_callback and scan_callback as callback methods


       NOTE that the x component of the linear velocity in odom is the speed
       """


       self.speed = 0
       # odometry subscription
       self.odom_subscription = self.create_subscription(
           Odometry,
           'ego_racecar/odom',              
           self.odom_callback,
           10
           )
       self.odom_subscription   


       # LaserScan subscription
       self.scan_subscription = self.create_subscription(
           LaserScan,
           'scan',              
           self.scan_callback,
           10
           )
       self.scan_subscription 


       # create publisher
       self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
  


   def odom_callback(self, odom_msg):
       # longitudinal velocity in the cars x axis (velocity forwards)
       self.speed = odom_msg.twist.twist.linear.x
      
   def scan_callback(self, scan_msg):
       ranges = scan_msg.ranges
       threshold = 0.0


       for i in range(len(ranges)):
           # calculate range rate using v * cos(theta)
           theta = scan_msg.angle_min + i*scan_msg.angle_increment
           range_rate = self.speed * np.cos(theta)
           if range_rate <= 0: # neg range rate means distance is incresing
               range_rate = 0
               ttc = np.inf
           else:
               ttc = ranges[i] / range_rate # calculate the time to collision
          
           # TODO: publish command to brake (set ackermannDriveStamped to 0)
           if  ttc < threshold: # pos range rate means distance is decreasing
               msg = AckermannDriveStamped()
               msg.drive.speed = 0.0
               self.publisher_.publish(msg)
               self.get_logger().info('BREE - BREAKING!')
              


def main(args=None):
   #init ROS2 COMM and FEATURES
   rclpy.init(args=args)
   #create node inherited from node class SafetyNode
   bree_safety_node = SafetyNode()
   #Node spin -> keeps it alive until we kill [communication]
   rclpy.spin(bree_safety_node)


   # Destroy the node explicitly
   # (optional - otherwise it will be done automatically
   # when the garbage collector destroys the node object)
   bree_safety_node.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()


