import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs
import numpy as np
import math

# import open3d as o3d
# from scipy.spatial import procrustes
from calc_odom.icp import Icp

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_sub = self.create_subscription(Odometry, '/odom_imu', self.broadcast_transform, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        

    def broadcast_transform(self, msg):
        odom = Odometry()
        #odom = msg
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        self.odom_pub.publish(odom)
        
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
