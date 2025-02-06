# Code to process image from Turtlebot3 to identify center point of object
# Intro to Robotics Research 7785
# Georgia Institute of Technology
# Michael Keehn, Jeannie Zhang, 2025

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float32

import time
import sys

import numpy as np
import cv2
from cv_bridge import CvBridge

class PointSubscriberNode(Node):

	def __init__(self):		
		# Creates the node.
		super().__init__('point_subscriber')
		
		# Create the subscriber
		self.subscription = self.create_subscription(
            Point,                       # Message type
            '/obj_loc',                  # Topic name
            self.point_callback,         # Callback function
            10                           # QoS (queue size)
        )
        
		# Create Twist publisher
		self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        

		# Stop moving after 5 of the same point
		self.last_x = None
		self.same_x_count = 0
		self.max_same_x = 10

	def point_callback(self, msg: Point):
		if msg.x == self.last_x:
			self.same_x_count += 1
			if self.same_x_count >= self.max_same_x:
				self.stop_robot()
				return
		else:
			self.same_x_count = 0
			self.last_x = msg.x

		# Distance between center of robot camera and center of object
		distance = msg.x - (msg.z/2)
		print(f"Distance {distance}, {msg.x}, {msg.z}")
		twist = Twist()

		tolerance = 15
		angular_speed = float(0.5)

		if distance > tolerance:
			twist.angular.z = -angular_speed
		elif distance < -tolerance:
			twist.angular.z = angular_speed
		else:
			twist.angular.z = float(0)
	
		# Publish the Twist message
		self.twist_publisher.publish(twist)

	def stop_robot(self):
		twist = Twist()
		twist.angular.z = 0.0
		self.twist_publisher.publish(twist)
		print("Stopping robot: Same x value received multiple times")


def main(args=None):
    rclpy.init(args=args)
    node = PointSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
