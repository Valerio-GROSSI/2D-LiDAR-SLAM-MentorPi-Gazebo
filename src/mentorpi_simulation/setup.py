from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'mentorpi_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/'+ package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'meshes', 'acker'),
            glob(os.path.join('meshes', 'acker', '*'))),
        (os.path.join('share', package_name, 'meshes', 'common'),
            glob(os.path.join('meshes', 'common', '*'))),
        (os.path.join('share', package_name, 'meshes', 'mecanum'),
            glob(os.path.join('meshes', 'mecanum', '*'))),
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*'))),
        (os.path.join('share', package_name, 'world'),
            glob(os.path.join('world', '*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='valerio',
    maintainer_email='valerio@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ground_truth_tf_odom_broadcaster = ground_truth_tf_odom_broadcaster.ground_truth_tf_odom_broadcaster:main'
            ],
        },
)
