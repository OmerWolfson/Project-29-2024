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

    # package_share_directory = get_package_share_directory('g4_robot_description')

    config_file_path = os.path.join(get_package_share_directory('g4_robot_description'), 'config', 'joint_controller.yaml')

    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('g4_robot_description'), 'urdf', 'g4_robot.urdf.xacro'
        ])
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}], 
        output='screen'
    )
 
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'isaac_joint_states'],  #joint_state_broadcaster
        output='screen'
    )

    load_joint_velocity_controller = ExecuteProcess( 
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_velocity_controller'],  
        output='screen'
    )

    load_my_diff_controller = Node(
        package="g4_robot_description",
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
        load_joint_velocity_controller,
        load_joint_state_broadcaster,
        load_controller_manager,
        load_my_diff_controller    #remember to add a ,
        #load_joint_position_controller 
        # load_my_controller
    ])