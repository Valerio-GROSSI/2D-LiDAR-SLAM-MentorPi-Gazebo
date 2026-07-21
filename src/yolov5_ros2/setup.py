from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'yolov5_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), 
            glob(os.path.join('models', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='valerio',
    maintainer_email='valerio1.grossi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolov5_node = yolov5_ros2.yolov5_detect.yolov5_node:main',
            'tracking_node = yolov5_ros2.object_tracking.tracking_node:main'
        ],
    },
)
