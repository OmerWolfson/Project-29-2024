# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# from ultralytics import YOLO
# from std_msgs.msg import String

# class YoloPersonDetection(Node):
#     def __init__(self):
#         super().__init__('yolo_person_detection')
#         self.declare_parameter('input_topic', '/g4_robot1/image_raw')
#         self.declare_parameter('output_topic', '/g4_robot1/image_yolo_person')
#         self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
#         self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        
#         self.bridge = CvBridge()
#         self.model = YOLO('runs/detect/train_person/weights/best.pt')  # Update the path to your person detection model
        
#         self.image_subscription = self.create_subscription(
#             Image,
#             self.input_topic,
#             self.listener_callback,
#             10
#         )
#         self.detection_publisher = self.create_publisher(String, self.output_topic, 10)
#         self.image_publisher = self.create_publisher(Image, self.output_topic + '_image', 10)

#     def listener_callback(self, msg):
#         try:
#             # Convert ROS Image message to OpenCV image
#             cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

#             # Perform detection
#             results = self.model(cv_image)

#             # Annotate image with detections
#             annotated_image = cv_image.copy()
#             detected_objects = []
#             for result in results:
#                 for det in result.boxes:
#                     conf = det.conf.item() if hasattr(det.conf, 'item') else det.conf  # Ensure conf is a number
#                     if det.cls == 0 and conf >= 0.5:  # Assuming class index 0 is for people
#                         xyxy = det.xyxy[0].tolist()
#                         label = f'Person {conf:.2f}'  # Format confidence value
#                         detected_objects.append(label)
#                         # Draw bounding box and label on image
#                         cv2.rectangle(annotated_image, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 2)
#                         cv2.putText(annotated_image, label, (int(xyxy[0]), int(xyxy[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

#             # Publish detection results
#             detection_msg = String()
#             detection_msg.data = '\n'.join(detected_objects)
#             self.detection_publisher.publish(detection_msg)

#             # Convert annotated image to ROS message and publish
#             ros_image_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
#             self.image_publisher.publish(ros_image_msg)
        
#         except Exception as e:
#             self.get_logger().error(f'Error in listener_callback: {e}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = YoloPersonDetection()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from std_msgs.msg import String

class YoloPersonDetection(Node):
    def __init__(self):
        super().__init__('yolo_person_detection')
        self.declare_parameter('input_topic', '/g4_robot2/image_raw')
        self.declare_parameter('output_topic', '/g4_robot2/image_yolo_person')
        self.declare_parameter('model_path', '/home/ubuntu/yolov8n.pt')  # Update this path to your model
        
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        
        self.bridge = CvBridge()
        self.model = YOLO(self.model_path)  # Load the YOLO model
        
        self.image_subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.listener_callback,
            10
        )
        self.detection_publisher = self.create_publisher(String, self.output_topic, 10)
        self.image_publisher = self.create_publisher(Image, self.output_topic + '_image', 10)

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Perform detection
            results = self.model(cv_image)

            # Annotate image with detections
            annotated_image = cv_image.copy()
            detected_objects = []
            for result in results:
                for det in result.boxes:
                    conf = det.conf.item() if hasattr(det.conf, 'item') else det.conf  # Ensure conf is a number
                    if det.cls == 0 and conf >= 0.5:  # Assuming class index 0 is for people
                        xyxy = det.xyxy[0].tolist()
                        label = f'Person {conf:.2f}'  # Format confidence value
                        detected_objects.append(label)
                        # Draw bounding box and label on image
                        cv2.rectangle(annotated_image, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 2)
                        cv2.putText(annotated_image, label, (int(xyxy[0]), int(xyxy[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            # Publish detection results
            detection_msg = String()
            detection_msg.data = '\n'.join(detected_objects)
            self.detection_publisher.publish(detection_msg)

            # Convert annotated image to ROS message and publish
            ros_image_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            self.image_publisher.publish(ros_image_msg)
        
        except Exception as e:
            self.get_logger().error(f'Error in listener_callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloPersonDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
