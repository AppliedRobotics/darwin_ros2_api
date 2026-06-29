import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'darwin_ros2_api'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rodion',
    maintainer_email='rodion_anisimov@mail.ru',
    description='ROS2 driver for Darwin Simulator WebSocket API',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'darwin_node = darwin_ros2_api.darwin_node:main',
            'example_takeoff_land = '
            'darwin_ros2_api.examples.example_takeoff_land:main',
            'example_square_flight = '
            'darwin_ros2_api.examples.example_square_flight:main',
            'example_maze_right_hand = '
            'darwin_ros2_api.examples.example_maze_right_hand:main',
        ],
    },
)
