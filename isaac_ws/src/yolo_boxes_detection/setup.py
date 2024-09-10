from setuptools import setup, find_packages

setup(
    name='yolo_boxes_detection',
    version='0.1.0',
    packages=find_packages(
        include=['yolo_boxes_detection', 'yolo_boxes_detection.*']
    ),
    install_requires=[
    'setuptools',
    'rclpy',
    'sensor_msgs',
    'cv_bridge',
    'ultralytics',
    'opencv-python-headless',
    ],
    entry_points={
        'console_scripts': [
            'yolo_boxes_detection_node = yolo_boxes_detection.yolo_boxes_detection:main',
        ],
    },
    zip_safe=True,
)
