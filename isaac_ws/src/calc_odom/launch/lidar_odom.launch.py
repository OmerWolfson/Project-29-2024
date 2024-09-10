import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization',
            output='screen',
            parameters=[{
                'sensor_timeout': 0.1,
                'two_d_mode': True,
                'odom_frame': 'odom',
                'base_link_frame': 'base_footprint',
                'world_frame': 'map',
                'publish_tf': True,
                'frequency': 30.0,
                'odom0': '/scan',
                'odom0_config': '[false, false, false,  true,  true,  true,  false]',
                'odom0_differential': False,
                'odom0_queue_size': 10,
                'odom0_pose_rejection_threshold': 0.8,
                'odom0_twist_rejection_threshold': 0.8,
                'odom0_pose_rejection_threshold': 0.5,
                'odom0_twist_rejection_threshold': 0.5,
                'process_noise_covariance': '[0.05, 0,    0,    0,    0,    0,    0,\
                                              0,    0.05, 0,    0,    0,    0,    0,\
                                              0,    0,    0.06, 0,    0,    0,    0,\
                                              0,    0,    0,    0.03, 0,    0,    0,\
                                              0,    0,    0,    0,    0.03, 0,    0,\
                                              0,    0,    0,    0,    0,    0.06, 0,\
                                              0,    0,    0,    0,    0,    0,    0.025]',
            }]
        )
    ])

