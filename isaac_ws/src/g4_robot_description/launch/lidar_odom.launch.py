import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to robot_localization package's share directory
    robot_localization_share_dir = get_package_share_directory('robot_localization')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization',
            output='screen',
            parameters=[
                robot_localization_share_dir + '/config/ekf.yaml',
                {
                    'sensor_timeout': 0.1,
                    'two_d_mode': True,
                    'odom_frame': 'odom',
                    'base_link_frame': 'base_link',
                    'world_frame': 'map',
                    'publish_tf': True,
                    'frequency': 30.0,
                    'odom0': '/scan',
                    'odom0_config': [False, False, False,  True,  True,  True],
                    'odom0_differential': False,
                    'odom0_queue_size': 10,
                    'process_noise_covariance': [0.05, 0,    0,    0,    0,    0,    0,
                                                 0,    0.05, 0,    0,    0,    0,    0,
                                                 0,    0,    0.06, 0,    0,    0,    0,
                                                 0,    0,    0,    0.03, 0,    0,    0,
                                                 0,    0,    0,    0,    0.03, 0,    0,
                                                 0,    0,    0,    0,    0,    0.06, 0,
                                                 0,    0,    0,    0,    0,    0,    0.025],
                }
            ]
        )
    ])
