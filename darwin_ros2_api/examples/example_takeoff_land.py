#!/usr/bin/env python3
"""Пример: взлёт -> зависание -> посадка.

Перед запуском должен работать darwin_node::

    ros2 launch darwin_ros2_api darwin_node.launch.py

Запуск примера::

    ros2 run darwin_ros2_api example_takeoff_land
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


def wait(node, seconds):
    """Ждём несколько секунд, не блокируя ROS."""
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def main():
    rclpy.init()
    node = Node('example_takeoff_land')

    takeoff = node.create_publisher(Empty, 'takeoff', 10)
    hover = node.create_publisher(Empty, 'hover', 10)
    land = node.create_publisher(Empty, 'land', 10)

    wait(node, 2)  # даём darwin_node подключиться

    node.get_logger().info('Взлёт')
    takeoff.publish(Empty())
    wait(node, 3)

    node.get_logger().info('Зависание 5 секунд')
    hover.publish(Empty())
    wait(node, 5)

    node.get_logger().info('Посадка')
    land.publish(Empty())
    wait(node, 2)

    node.get_logger().info('Готово')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
