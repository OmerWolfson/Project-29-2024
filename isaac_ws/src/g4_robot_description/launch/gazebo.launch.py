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
#from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # package_share_directory = get_package_share_directory('g4_robot_description')

    #config_file_path = os.path.join(get_package_share_directory('g4_robot_description'), 'config', 'joint_controller.yaml')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'),
                                  'launch', 'gazebo.launch.py'])
        ])
    )

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

    robot_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot'],
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
        package="g4_robot_description",
        executable='my_diff_drive_controller'
    )

    # load_controller_manager = Node(
    #         package='controller_manager',
    #         executable='ros2_control_node',
    #         parameters=[config_file_path],
    #         output='screen',
    #     )

    # diff_drive_spawner = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_controller'],
    #     output='screen'
    # )

    

    # diff_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["diff_drive_controller"], #pay attention to the , might want to remove
    # )

    # load_joint_position_controller = ExecuteProcess( 
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'], # position_controller or joint_trajectory_controller
    #     output='screen'
    # )

    # load_my_controller = Node(
    #     package='g4_robot_description',
    #     executable='joint_publisher'
    # )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        load_my_diff_controller,
        load_joint_velocity_controller,
        robot_spawn_node,
        load_joint_state_broadcaster #remember to add a ,
        #load_joint_position_controller 
        # load_my_controller
    ])