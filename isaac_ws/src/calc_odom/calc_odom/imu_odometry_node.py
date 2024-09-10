import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math
import numpy as np

class ImuOdometryNode(Node):
    def __init__(self):
        super().__init__('imu_odometry_node')
        
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.odom_frame_ = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame_ = self.get_parameter('base_frame').get_parameter_value().string_value

        # self.joint_state_sub_ = self.create_subscription(JointState, 'g4_robot2/isaac_joint_states', self.joint_state_callback, 10)
        self.imu_sub_ = self.create_subscription(Imu, 'g4_robot2/imu', self.imu_callback, 10)

        self.odom_pub_ = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster_ = tf2_ros.TransformBroadcaster(self)
        
        self.translation = np.zeros(2)
        self.velocities = np.zeros(2)
    
        # self.th = 0.0
        self.prev_time = self.get_clock().now()

    def imu_callback(self, msg: Imu):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time

        # self.get_logger().info(f'My log message {current_time}', once=True)
        # self.get_logger().info(f"prev time: {self.prev_time}", once=True)
        # self.get_logger().info(f"dt: {dt}", once=True)

        # Integrate the angular velocities to get orientation
        #self.th += msg.angular_velocity.z * dt

        # Use the accelerometer to get linear velocities (simplified)
        self.velocities[0] +=  msg.linear_acceleration.x * dt
        self.velocities[1] +=  msg.linear_acceleration.y * dt

        # Integrate linear velocities to get position
        q = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        R = self.rotation_matrix_quaterion(q)
        global_velocities = np.dot(R, self.velocities) 
        self.translation += self.velocities * dt # instead of global velocities to test

        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame_
        odom_msg.child_frame_id = self.base_frame_

        odom_msg.pose.pose.position.x = self.translation[0]
        odom_msg.pose.pose.position.y = self.translation[1]
        odom_msg.pose.pose.position.z = 0.0

        # quaternion = quaternion_from_euler(0, 0, self.th)
        odom_msg.pose.pose.orientation.w = msg.orientation.x
        odom_msg.pose.pose.orientation.x = msg.orientation.y
        odom_msg.pose.pose.orientation.y = msg.orientation.z
        odom_msg.pose.pose.orientation.z = msg.orientation.w

        odom_msg.twist.twist.linear.x = self.velocities[0]
        odom_msg.twist.twist.linear.y = self.velocities[1]
        odom_msg.twist.twist.angular.z = msg.angular_velocity.z

        self.odom_pub_.publish(odom_msg)

        # Create transform message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame_
        t.child_frame_id = self.base_frame_

        t.transform.translation.x = self.translation[0]
        t.transform.translation.y = self.translation[1]
        t.transform.translation.z = 0.0
        t.transform.rotation.w = msg.orientation.x
        t.transform.rotation.x = msg.orientation.y
        t.transform.rotation.y = msg.orientation.z
        t.transform.rotation.z = msg.orientation.w

        self.tf_broadcaster_.sendTransform(t)

    # def joint_state_callback(self, msg: JointState):
        # joint name:          index:
        # front_hinge_joint     0
        # left_wheel_joint      1
        # rear_hinge_joint      2
        # right_wheel_joint     3
        # front_wheel_joint     4
        # rear_wheel_joint      5
        # shoulder_pan_joint    6
        # shoulder_lift_joint   7
        # elbow_joint           8
        # wrist_1_joint         9
        # wrist_2_joint         10
        # wrist_3_joint         11
        
    # def quaternion_from_euler(self, roll, pitch, yaw):
    
    #     cy = math.cos(yaw * 0.5)
    #     sy = math.sin(yaw * 0.5)
    #     cp = math.cos(pitch * 0.5)
    #     sp = math.sin(pitch * 0.5)
    #     cr = math.cos(roll * 0.5)
    #     sr = math.sin(roll * 0.5)

    #     x = sr * cp * cy - cr * sp * sy
    #     y = cr * sp * cy + sr * cp * sy
    #     z = cr * cp * sy - sr * sp * cy
    #     w = cr * cp * cy + sr * sp * sy

    #     return(x, y, z, w)


    def rotation_matrix_quaterion(self, q): 
        x, y, z, w = q             
        return np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z],
            [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2]
        ])

def main(args=None):
    rclpy.init(args=args)
    node = ImuOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
