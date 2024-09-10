import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs
import numpy as np
import math

# import open3d as o3d
from scipy.spatial import procrustes

class LidarOdometryNode(Node):
    def __init__(self):
        super().__init__('lidar_odometry_node')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.prev_scan = None
        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_footprint"
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def scan_callback(self, msg):
        if self.prev_scan is None:
            self.prev_scan = msg
            return

        # Compute odometry from current and previous scan
        transformation, orientation = self.compute_odometry(self.prev_scan, msg)
        
        delta_x = transformation[0, 1] # 2nd elemnt used to be 3 when was 3d
        delta_y = transformation[1, 1]
        delta_theta = np.arctan2(transformation[1, 0], transformation[0, 0])
        # self.get_logger().info(f"Matrix:\n{orientation}")
        self.get_logger().info(f"dx: {delta_x}\ndy: {delta_x}\n\n{transformation}")
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        self.odom.header.stamp = self.get_clock().now().to_msg()
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        self.odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        self.odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        self.odom_pub.publish(self.odom)
        self.broadcast_transform()

        self.prev_scan = msg

    def compute_odometry(self, prev_msg, msg):
        prev_ranges_array = np.array(prev_msg.ranges)
        prev_intensities_array = np.array(prev_msg.intensities)
        prev_scan = np.vstack((prev_ranges_array, prev_intensities_array)).T

        ranges_array = np.array(msg.ranges)
        intensities_array = np.array(msg.intensities)
        curr_scan = np.vstack((ranges_array, intensities_array)).T

        # Perform 2D ICP using Procrustes analysis
        transformation, orientation, _ = procrustes(prev_scan, curr_scan)

        return transformation, orientation


    # def compute_odometry(self, prev_msg, msg, init_transformation=np.identity(4), max_correspondence_distance=1.0, max_iterations=30):
    #     """
    #     Perform ICP (Iterative Closest Point) on 2D LiDAR scan data to compute the transformation.

    #     :param prev_scan: Previous scan data as a 2D numpy array of shape (N, 2).
    #     :param curr_scan: Current scan data as a 2D numpy array of shape (N, 2).
    #     :param init_transformation: Initial transformation as a 4x4 numpy array.
    #     :param max_correspondence_distance: Maximum correspondence distance for ICP.
    #     :param max_iterations: Maximum number of iterations for ICP.
    #     :return: Transformation matrix as a 4x4 numpy array.
    #     """
    #     prev_ranges_array = np.array(prev_msg.ranges)
    #     prev_intensities_array = np.array(prev_msg.intensities)
    #     prev_scan = np.vstack((prev_ranges_array, prev_intensities_array)).T

    #     ranges_array = np.array(msg.ranges)
    #     intensities_array = np.array(msg.intensities)
    #     curr_scan = np.vstack((ranges_array, intensities_array)).T
        
    #     # Convert 2D scans to 3D point clouds (open3d requires 3D point clouds)
    #     prev_points = np.hstack((prev_scan, np.zeros((prev_scan.shape[0], 1))))
    #     curr_points = np.hstack((curr_scan, np.zeros((curr_scan.shape[0], 1))))

    #     # Create Open3D PointCloud objects
    #     prev_pcd = o3d.geometry.PointCloud()
    #     curr_pcd = o3d.geometry.PointCloud()
    #     prev_pcd.points = o3d.utility.Vector3dVector(prev_points)
    #     curr_pcd.points = o3d.utility.Vector3dVector(curr_points)

    #     # Perform ICP
    #     result_icp = o3d.pipelines.registration.registration_icp(
    #         curr_pcd, prev_pcd, max_correspondence_distance, init_transformation,
    #         o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    #         o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iterations)
    #     )

    #     return result_icp.transformation

    def broadcast_transform(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = LidarOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
