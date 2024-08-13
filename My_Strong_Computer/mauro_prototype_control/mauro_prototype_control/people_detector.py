#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
#import torch
#from ultralytics import YOLO

class YOLOv8Node(Node):
    def __init__(self):
        super().__init__('people_detector')
        self.subscription_ = self.create_subscription(Image, 'prototype/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        # self.device
        # self.bottom_y_value = None

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            self.get_logger().info("trying")
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Display the image using OpenCV
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)  # Add a short delay to allow image display
        except Exception as e:
            self.get_logger().error(f"Error displaying image: {e}")


        # # Convert ROS Image message to OpenCV image
        # cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # # Perform YOLOv8 detection
        # results = self.model(cv_image)

        # # Process the results
        # for detection in results.xyxy[0]:  # detections in xyxy format
        #     x1, y1, x2, y2, conf, cls = detection
        #     self.bottom_y_value = y2.item()  # Save the bottom y-coordinate of the bounding box
        #     self.get_logger().info(f'Detected object with bottom y-coordinate: {self.bottom_y_value}')

        # # Display the image with bounding boxes (for debugging purposes)
        # annotated_image = results.render()[0]
        # cv2.imshow('YOLOv8 Detection', annotated_image)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8Node()
    rclpy.spin(node)

    # Destroy the node explicitly (optional)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
