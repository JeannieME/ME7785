# Code to process image from Turtlebot3 to identify center point of object
# Intro to Robotics Research 7785
# Georgia Institute of Technology
# Michael Keehn, Jeannie Zhang, 2025

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point 
from std_msgs.msg import Float32

import sys

import numpy as np
import cv2
from cv_bridge import CvBridge

class RobotVideoSubscriber(Node):

	def __init__(self):		
		# Creates the node.
		super().__init__('robot_video_subscriber')

		# Set Parameters
		#self.declare_parameter('show_image_bool', False)
		#self.declare_parameter('window_name', "Raw Image")

		#Determine Window Showing Based on Input
		#self._display_image = bool(self.get_parameter('show_image_bool').value)

		# Declare some variables
		#self._titleOriginal = self.get_parameter('window_name').value # Image Window Title	
		
		#Only create image frames if we are not running headless (_display_image sets this)
		#if(self._display_image):
		# Set Up Image Viewing
			#cv2.namedWindow(self._titleOriginal, cv2.WINDOW_AUTOSIZE ) # Viewing Window
			#cv2.moveWindow(self._titleOriginal, 50, 50) # Viewing Window Original Location
		
		#Set up QoS Profiles for passing images over WiFi
		image_qos_profile = QoSProfile(
		    reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
		)

		#Declare that the robot_video_subscriber node is subcribing to the /camera/image/compressed topic.
		self._video_subscriber = self.create_subscription(
				CompressedImage,
				'/image_raw/compressed',
				self._image_callback,
				image_qos_profile)
		self._video_subscriber # Prevents unused variable warning.

		#Declare that the robot_video_subscriber node is publishing _____
		self.point_publisher = self.create_publisher(Point,'/obj_loc',10)
		timer_period = 0.1  # seconds
		self.obj_loc = Point()
		self.timer = self.create_timer(timer_period, self.timer_callback)
		
	
    # DO NOT NEED THIS
	# def get_user_input(self):
		#return self._user_input
	
	def _image_callback(self, CompressedImage):	
		# The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
		self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
		self.find_object(self._imgBGR)
        #if(self._display_image):
			# Display the image in a window
			#self.show_image(self._imgBGR)
			

	# DO NOT NEED THIS
    #def show_image(self, img):
		#cv2.imshow(self._titleOriginal, img)
		## Cause a slight delay so image is displayed
		#self._user_input=cv2.waitKey(50) #Use OpenCV keystroke grabber for delay.

	
	def find_object(self, img):
		# Use the image directly (no need for img.read())
		frame = img

		# Convert to HSV color space
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  
		
		# We are tracking a red object in this case
		low_red1 = np.array([0, 120, 70])
		high_red1 = np.array([10, 255, 255])
		low_red2 = np.array([170, 120, 70])
		high_red2 = np.array([180, 255, 255])

		# Mask makes red to white and all other colors black
		mask1 = cv2.inRange(hsv, low_red1, high_red1)
		mask2 = cv2.inRange(hsv, low_red2, high_red2)
		mask = mask1 | mask2
		result = cv2.bitwise_and(frame, frame, mask=mask)

		# Find contours of the red objects
		contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		for contour in contours:
			if cv2.contourArea(contour) > 1000:
				x, y, w, h = cv2.boundingRect(contour)
				cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

				# Get pixel locations
				pixel_loc = np.column_stack(np.where(mask == 255))
				center_x = x + w / 2
				center_y = y + h / 2
				print(f"Bounding box at ({x}, {y}, {w}, {h}) with center at ({center_x}, {center_y})")
				self.obj_loc.x = center_x
				self.obj_loc.y = center_y
				self.obj_loc.z = float(self._imgBGR.shape[1])
		# Display the result image
		#cv2.imshow(self._titleOriginal, frame)

	def timer_callback(self):
		self.point_publisher.publish(self.obj_loc)
		print(f"Publish Point {self.obj_loc}")

def main():
	rclpy.init() #init routine needed for ROS2.
	video_subscriber = RobotVideoSubscriber() #Create class object to be used.

	while rclpy.ok():
		rclpy.spin_once(video_subscriber) # Trigger callback processing.
		#if(video_subscriber._display_image):	
		#	if video_subscriber.get_user_input() == ord('q'):
		#		cv2.destroyAllWindows()
		#		break
	rclpy.logging.get_logger("Camera Viewer Node Info...").info("Shutting Down")
	#Clean up and shutdown.
	video_subscriber.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()