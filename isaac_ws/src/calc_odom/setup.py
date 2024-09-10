from setuptools import find_packages, setup

package_name = 'calc_odom'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy>=1.17.3,<2.0.0', 'open3d', 'scipy', 'scikit-learn'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_odometry_node = calc_odom.lidar_odometry_node:main',
            'imu_odometry_node = calc_odom.imu_odometry_node:main'
        ],
    },
)
