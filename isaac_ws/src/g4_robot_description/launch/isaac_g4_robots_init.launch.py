from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur10e_controller',
            executable='ur10e_position_controller',
            namespace='ur10e1',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='ur10e_controller',
            executable='ur10e_position_controller',
            namespace='ur10e2',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='g4_robot_description',
            executable='isaac_diff_drive_controller',
            namespace='g4_robot1',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='g4_robot_description',
            executable='isaac_diff_drive_controller',
            # namespace='g4_robot2',  
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='joy',
            executable='joy_node',
            namespace='g4_robot1',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            namespace='g4_robot1',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{
        #         'robot_description': open(get_package_share_directory('g4_robot_description') + '/urdf/g4_robot.urdf').read() # removed the .xml after urdf
        #     }],
        #     remappings=[
        #         ('joint_states', '/g4_robot2/isaac_joint_states')
        #     ]
        # )

        # Node(
        #     package='teleop_twist_keyboard',
        #     executable='teleop_twist_keyboard',
        #     # name='teleop_twist_keyboard',          # Inappropriate place for this node? run in different terminal
        #     namespace='g4_robot1',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}]
        # )
    ])

if __name__ == '__main__':
    generate_launch_description()
