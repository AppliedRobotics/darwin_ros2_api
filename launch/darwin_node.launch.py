#!/usr/bin/env python3
"""Launch-файл для запуска ROS2-драйвера Darwin Simulator.

Запускает ноду ``darwin_node`` и по умолчанию подтягивает все параметры
из ``config/darwin_params.yaml`` (host, port, frame-ы, TF и т.д.).
Чтобы поменять настройки — правьте YAML или передайте свой файл через
аргумент ``params_file``.

Примеры запуска::

    ros2 launch darwin_ros2_api darwin_node.launch.py
    ros2 launch darwin_ros2_api darwin_node.launch.py params_file:=/path/to/my_params.yaml
    ros2 launch darwin_ros2_api darwin_node.launch.py rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('darwin_ros2_api')
    default_params = os.path.join(pkg_share, 'config', 'darwin_params.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'darwin.rviz')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Путь к YAML-файлу с параметрами ноды.',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Запускать ли RViz2 с конфигом проекта (true/false).',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz,
        description='Путь к .rviz конфигу для RViz2.',
    )

    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')

    darwin_node = Node(
        package='darwin_ros2_api',
        executable='darwin_node',
        name='darwin_node',
        output='screen',
        parameters=[params_file],
    )

    # RViz2 запускается только при rviz:=true. Используется наш конфиг
    # (rviz/darwin.rviz) без плагинов nav2 — только rviz_default_plugins.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        params_file_arg,
        rviz_arg,
        rviz_config_arg,
        darwin_node,
        rviz_node,
    ])
