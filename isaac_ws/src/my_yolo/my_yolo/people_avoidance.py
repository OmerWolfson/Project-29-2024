#!/usr/bin/env python3

import rclpy
import rclpy.action
from rclpy.node import Node
import numpy as np
import time
#from robot_navigator import BasicNavigator, NavigationResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from builtin_interfaces.msg import Duration

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from nav2_msgs.action import NavigateToPose, Wait
from rclpy.executors import SingleThreadedExecutor

class peopleAvoidNode(Node):
    def __init__(self):
        super().__init__('people_avoidance')
        #self.navigate_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.wait_client = ActionClient(self, Wait, '/wait')

        self.get_logger().warning("Careful! Avoidance will not work until nav2 is online!")
        self.wait_client.wait_for_server()
        self.get_logger().info("OK! Nav2 is online!\nAvoidance can take place.")

        # self.goal_data = None
        # self.goal_handle_ = None
        
        #self.cancel_client = self.create_client(Empty, '/navigate_to_pose/cancel')
        #self.current_goal_handle = None?

        #self.navigator = BasicNavigator()

        # self.goal_sub_ = self.create_subscription(PoseStamped, '/goal_pose', self.goal_getter, 10)

        self.proximity_sub_ = self.create_subscription(Empty, '/person_proximity', self.big_person_detect, 10)
        #self.cancel_request_ = self.create_service(Empty, 'cancel_goal', self.cancel_goal_callback)

    def big_person_detect(self, msg):
        self.get_logger().info("big boi in proximity")
        goal_msg = Wait.Goal()
        dur = Duration()
        dur.sec = 5
        dur.nanosec = 0
        goal_msg.time = dur
        self.get_logger().info(f"requesting {dur.sec} seconds to wait")

        self.wait_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future): 
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future): 
        result = future.result().result
        self.get_logger().info("Result is: " + str(result.total_elapsed_time))

def main(args=None):
    rclpy.init(args=None)
    node = peopleAvoidNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()