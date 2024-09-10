#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

class cameraPubNode(Node):

    def __init__(self):
        super().__init__('cv_vid_pub')
        self.publisher_ = self.create_publisher(Image, '/g4_robot2/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.cv_bridge = CvBridge()


    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret == True:
            self.publisher_.publish(self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8'))
            self.get_logger().info('Publishing video...')
            #cv2.imshow('Image Window', frame)
            #cv2.waitKey(10)
        else:
            self.get_logger().error('Error converting ROS Image to OpenCV Image')


def main(args=None):
    rclpy.init(args=None)
    node = cameraPubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()