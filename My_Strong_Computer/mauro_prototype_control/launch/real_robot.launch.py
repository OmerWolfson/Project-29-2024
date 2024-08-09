import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # package_share_directory = get_package_share_directory('mauro_prototype_description') # or control instead of description
    # config_file_path = os.path.join(package_share_directory, 'config', 'joint_controller.yaml')

    config_file_path = os.path.join(get_package_share_directory('mauro_prototype_description'), 'config', 'joint_controller.yaml')

    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('mauro_prototype_description'), 'urdf', 'mauro_prototype.urdf.xacro'
        ])
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}], 
        output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'], 
        output='screen'
    )

    load_joint_velocity_controller = ExecuteProcess( 
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_velocity_controller'],  
        output='screen'
    )

    load_my_diff_controller = Node(
        package="mauro_prototype_control",
        executable='my_diff_drive_controller'
    )

    load_controller_manager = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[config_file_path],
            output='screen',
        )

    return LaunchDescription([
        robot_state_publisher_node,
        load_controller_manager,
        load_my_diff_controller,
        load_joint_velocity_controller,
        load_joint_state_broadcaster #remember to add a ,
    ])