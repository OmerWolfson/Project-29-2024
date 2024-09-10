#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
import ultralytics
from std_msgs.msg import Bool

bridge = CvBridge()

class PrototypeYOLONode(Node):
	def __init__(self):
		super().__init__('prototype_people_detector')
		self.get_logger().info("Node initiated, YOLO details:")
		ultralytics.checks()

		self.model = YOLO('yolov8n.pt')
		self.stopper_msg = Bool()
		self.stopper_msg = True

		self.timer_ = self.create_timer(0.1, self.timer_callback)
		self.cap = cv2.VideoCapture(2) # for usb cam
		self.cv_bridge = CvBridge()
		
		self.yolo_pub_ = self.create_publisher(Image, 'prototype/yolo_detect', 10)
		self.safety_control_pub_ = self.create_publisher(Bool, 'prototype/drive_override', 10)

	def timer_callback(self):
		ret, frame = self.cap.read()
		if ret:
			#results = self.model(source=image, show=False, conf=0.45, save=False)
			results = self.model.predict(source=frame, show=False, conf=0.45, save=False, classes=[0, 15, 16])
			msg = self.cv_bridge.cv2_to_imgmsg(results, "bgr8")
			self.get_logger().info("Getting camera data...")
			self.yolo_pub_.publish(msg)
			boxes = results[0].boxes.xywh.cpu()
			for box in boxes:
				x, y, w, h = box
				self.get_logger().info(f"width: {w}, hieght: {h}")
				if h > 375 and w > 100: # should change to height of feet (h>375 and w > 100)
					self.get_logger().warning("Danger\nDanger\nDanger")
					self.safety_control_pub_.publish(self.stopper_msg)
					
					
			# Can be written differently:
			# result = self.model(image)
			# results = self.model.predict(source=image, show=True, stream=True, device='0')
			
		else:
			self.get_logger().error("Failed to capture image")

		

	def destroy_node(self):
		self.cap.release()
		super().destroy_node()

		

def main(args=None):
	rclpy.init(args=None)
	node = PrototypeYOLONode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()