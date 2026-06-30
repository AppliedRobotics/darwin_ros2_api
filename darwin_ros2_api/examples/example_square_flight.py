#!/usr/bin/env python3
"""Пример: полёт по квадрату.

Перед запуском должен работать darwin_node::

    ros2 launch darwin_ros2_api darwin_node.launch.py

Запуск примера::

    ros2 run darwin_ros2_api example_square_flight

cmd_vel нужно отправлять часто (~10 раз в секунду), иначе дрон
останавливается через 0.1 с после каждой команды.
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

# --- настройки (можно менять прямо здесь) ---
SPEED = 0.5          # скорость вперёд, м/с
SIDE_TIME = 2.0      # секунд лететь по стороне квадрата
TURN_SPEED = 45.0    # поворот, град/с
TURN_TIME = 2.1      # секунд крутиться на месте
SIDES = 4            # сколько сторон квадрата


def wait(node, seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def move_forward(node, pub, speed, seconds):
    """Летим вперёд заданное время."""
    twist = Twist()
    twist.linear.x = speed
    end = time.time() + seconds
    while time.time() < end:
        pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.1)
    pub.publish(Twist())  # стоп


def turn_left(node, pub, deg_per_sec, seconds):
    """Крутимся на месте влево."""
    twist = Twist()
    twist.angular.z = math.radians(deg_per_sec)
    end = time.time() + seconds
    while time.time() < end:
        pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.1)
    pub.publish(Twist())  # стоп


def main():
    rclpy.init()
    node = Node('example_square_flight')

    cmd = node.create_publisher(Twist, 'cmd_vel', 10)
    takeoff = node.create_publisher(Empty, 'takeoff', 10)
    hover = node.create_publisher(Empty, 'hover', 10)
    land = node.create_publisher(Empty, 'land', 10)

    wait(node, 2)

    node.get_logger().info('Взлёт')
    takeoff.publish(Empty())
    wait(node, 3)

    node.get_logger().info('Полёт по квадрату')
    for i in range(SIDES):
        node.get_logger().info(f'Сторона {i + 1}')
        move_forward(node, cmd, SPEED, SIDE_TIME)
        turn_left(node, cmd, TURN_SPEED, TURN_TIME)

    node.get_logger().info('Зависание и посадка')
    hover.publish(Empty())
    land.publish(Empty())
    wait(node, 2)

    node.get_logger().info('Готово')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
