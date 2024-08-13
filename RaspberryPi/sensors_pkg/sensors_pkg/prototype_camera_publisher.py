#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class PrototyoeCameraPublisher(Node):
	def __init__(self):
		super().__init__("prototype_camera_publisher")
		self.pub_ = self.create_publisher(Image, 'prototype/image_raw', 10)
		self.timer_ = self.create_timer(0.1, self.timer_callback)
		self.cap = cv2.VideoCapture(2) # for usb cam           # ----- remember to install cv2 on the RPi ----- #
		self.cv_bridge = CvBridge()
		

	def timer_callback(self):
		ret, frame = self.cap.read()
		if ret:
			msg = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
			self.get_logger().info("Publishing camera data...")
			self.pub_.publish(msg)
		else:
			self.get_logger().error("Failed to capture image")

	def destroy_node(self):
		self.cap.release()
		super().destroy_node()


def main(args=None):
	rclpy.init(args=args)
	node = PrototyoeCameraPublisher()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	
if __name__== "__main__":
	main()