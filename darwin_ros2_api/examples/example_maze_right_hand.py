#!/usr/bin/env python3
"""Пример: лабиринт по правилу правой руки (лидар).

Дрон держит правую стену рядом с собой:
  - стена справа далеко  -> поворачиваем направо
  - препятствие впереди   -> поворачиваем налево
  - иначе                -> летим вперёд

Перед запуском должен работать darwin_node::

    ros2 launch darwin_ros2_api darwin_node.launch.py

Запуск примера::

    ros2 run darwin_ros2_api example_maze_right_hand

Остановка: Ctrl+C
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String

# --- настройки (можно менять прямо здесь) ---
FLIGHT_HEIGHT = 0.3   # высота полёта, м
ENTRY_TIME = 2.0      # секунд лететь вперёд, чтобы влететь в лабиринт
SPEED = 0.4           # скорость вперёд, м/с
WALL_DIST = 0.3      # желаемое расстояние до правой стены, м
FRONT_STOP = 0.4      # если ближе — поворачиваем налево, м
RIGHT_LOST = 1.0      # если правой стены нет — поворачиваем направо, м

# последний скан лидара (обновляется в callback)
scan = None


def on_scan(msg):
    global scan
    scan = msg


def wait(node, seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def move_forward(node, pub, speed, seconds):
    twist = Twist()
    twist.linear.x = speed
    end = time.time() + seconds
    while time.time() < end:
        pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.1)
    pub.publish(Twist())


def distance_at_deg(msg, deg):
    """Расстояние в направлении deg градусов (0 = вперёд, -90 = справа)."""
    angle = math.radians(deg)
    i = int((angle - msg.angle_min) / msg.angle_increment)
    i = max(0, min(i, len(msg.ranges) - 1))
    r = msg.ranges[i]
    if math.isfinite(r) and r > msg.range_min:
        return r
    return 999.0


def distance_front(msg):
    """Минимальное расстояние впереди (сектор ±25°)."""
    return min(distance_at_deg(msg, d) for d in range(-10, 10, 5))


def distance_right(msg):
    """Минимальное расстояние справа (сектор около -90°)."""
    return min(distance_at_deg(msg, d) for d in range(-100, -80, 5))


def main():
    rclpy.init()
    node = Node('example_maze_right_hand')

    cmd = node.create_publisher(Twist, 'cmd_vel', 10)
    takeoff = node.create_publisher(Empty, 'takeoff', 10)
    command = node.create_publisher(String, 'command', 10)
    node.create_subscription(LaserScan, 'scan', on_scan, 10)

    wait(node, 2)

    # --- подготовка ---
    node.get_logger().info('Взлёт')
    takeoff.publish(Empty())
    wait(node, 4)

    node.get_logger().info(f'Высота {FLIGHT_HEIGHT} м')
    msg = String()
    msg.data = json.dumps({'method': 'setHeight', 'params': FLIGHT_HEIGHT})
    command.publish(msg)
    wait(node, 2)

    node.get_logger().info('Влетаем в лабиринт')
    move_forward(node, cmd, SPEED, ENTRY_TIME)

    # --- основной цикл ---
    node.get_logger().info('Правило правой руки (Ctrl+C для остановки)')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if scan is None:
                continue

            front = scan.ranges[0]
            right = scan.ranges[270]
            print("right: ",right , " front: ", front)
            twist = Twist()

            if front < FRONT_STOP:
                print("front < FRONT_STOP")
                # стена впереди — поворот налево
                twist.angular.z = math.radians(60)
            elif right > RIGHT_LOST:
                print("right > RIGHT_LOST")
                # стены справа нет — поворот направо
                twist.linear.x = SPEED * 0.6
                twist.angular.z = -math.radians(48)
            else:
                print("near right wall")
                # едем вдоль правой стены
                twist.linear.x = SPEED
                err = right - WALL_DIST
                twist.angular.z = -0.5*err

            cmd.publish(twist)
    except KeyboardInterrupt:
        node.get_logger().info('Остановка')

    cmd.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
