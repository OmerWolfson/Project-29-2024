from setuptools import find_packages, setup

package_name = 'my_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cam_pub = my_yolo.cam_pub:main",
            "test1 = my_yolo.test1:main",
            "people_avoidance = my_yolo.people_avoidance:main",
            "robot_navigator = my_yolo.robot_navigator:main"
        ],
    },
)
