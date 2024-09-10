#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
import ultralytics
import torch
from std_msgs.msg import Empty

bridge = CvBridge()

class cameraSubNode(Node):
    def __init__(self):
        super().__init__('test1')
        self.get_logger().info("Node initiated, YOLO details:")
        ultralytics.checks()

        self.model = YOLO('yolov8n.pt')
        if torch.cuda.is_available():
            self.model.to('cuda')
            self.get_logger().info(f"{self.model.device}")
            self.model.info()
        else:
            self.get_logger().info("cuda not available")

        self.sub_ = self.create_subscription(Image, '/g4_robot2/image_raw', self.object_detect, 10)
        self.pub_ = self.create_publisher(Empty, '/person_proximity', 10)
        self.cv_bridge = CvBridge()       

    def object_detect(self, msg):
        image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        if image is not None:
            #results = self.model(source=image, show=False, conf=0.45, save=False)
            results = self.model.predict(source=image, show=False, conf=0.45, save=False, classes=[0, 15, 16])
            boxes = results[0].boxes.xywh.cpu()
            for box in boxes:
                x, y, w, h = box
                self.get_logger().info(f"width: {w}, hieght: {h}")
                if h > 375 and w > 100: # should change to height of feet (h>375 and w > 100)
                    # person_size = Float64MultiArray()
                    # person_size = (h, w)
                    self.get_logger().warning("Danger\nDanger\nDanger")
                    bipboop = Empty()
                    self.pub_.publish(bipboop)
                    
                    
            # Can be written differently:
            # result = self.model(image)
            # results = self.model.predict(source=image, show=True, stream=True, device='0')
        else:
            self.get_logger().info("Not recieving images")

def main(args=None):
    rclpy.init(args=None)
    node = cameraSubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()