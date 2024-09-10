"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'True')

    ## ***** File paths ******
    pkg_share = FindPackageShare('g3_cartographer').find('g3_cartographer')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'g3_robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  '/home/ubuntu/isaac_ws/src/g3_cartographer', 'config'))

    ## ***** Nodes *****
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output = 'screen'
        )

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments = [
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'g3_3d.lua'],
        remappings = [('points2', 'lidar_link/point_cloud'), ('imu', 'g3_robot/imu')],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': True},
            {'resolution': 0.05}],
        )

    return LaunchDescription([
        use_sim_time_arg,
        # Nodes
        robot_state_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
