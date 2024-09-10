from setuptools import setup

package_name = 'yolo_person_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'opencv-python', 'ultralytics'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email',
    description='YOLOv8 person detection node',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_person_detection_node = yolo_person_detection.yolo_person_detection_node:main'
        ],
    },
)
