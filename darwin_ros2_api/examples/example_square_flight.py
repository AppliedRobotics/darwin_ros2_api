#!/usr/bin/env python3
"""Пример полёта дрона по квадрату через топик ``cmd_vel``.

Драйвер ``darwin_node`` преобразует ``geometry_msgs/Twist`` из топика
``cmd_vel`` в вызов ``setVelXYYaw`` симулятора. Эта команда действует
всего ~0.1 с, поэтому, чтобы движение было непрерывным, скорость нужно
публиковать с достаточной частотой (здесь — 10 Гц).

Сценарий:
1. взлёт;
2. четыре раза: лететь вперёд, затем повернуть на ~90°;
3. зависнуть и сесть.

Запуск (драйвер должен быть запущен заранее)::

    ros2 run darwin_ros2_api darwin_node --ros-args -p host:=127.0.0.1 -p port:=8765
    ros2 run darwin_ros2_api example_square_flight

Параметры (необязательно)::

    -p side_speed:=0.5     # скорость движения вперёд, м/с
    -p side_time:=2.0      # длительность движения по одной стороне, с
    -p turn_speed:=45.0    # угловая скорость поворота, град/с
    -p turn_time:=2.0      # длительность поворота, с
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist


class SquareFlightExample(Node):
    """Облёт по квадрату с помощью непрерывной публикации cmd_vel."""

    def __init__(self):
        super().__init__('example_square_flight')

        self.side_speed = float(self.declare_parameter('side_speed', 0.5).value)
        self.side_time = float(self.declare_parameter('side_time', 2.0).value)
        self.turn_speed = float(self.declare_parameter('turn_speed', 45.0).value)
        self.turn_time = float(self.declare_parameter('turn_time', 2.0).value)
        self.sides = int(self.declare_parameter('sides', 4).value)

        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', cmd_qos)
        self._takeoff_pub = self.create_publisher(Empty, 'takeoff', cmd_qos)
        self._land_pub = self.create_publisher(Empty, 'land', cmd_qos)
        self._hover_pub = self.create_publisher(Empty, 'hover', cmd_qos)

        # Описание сценария как списка фаз: (тип, длительность_с).
        # 'move' — лететь вперёд, 'turn' — поворот на месте.
        self._phases = []
        for _ in range(self.sides):
            self._phases.append(('move', self.side_time))
            self._phases.append(('turn', self.turn_time))

        self._phase_idx = -1          # -1 => ещё не взлетели
        self._phase_elapsed = 0.0
        self._dt = 0.1                 # период публикации, с (10 Гц)
        self._started = False

        # Перед стартом даём драйверу подключиться к симулятору.
        self._init_timer = self.create_timer(2.0, self._start)

    def _start(self):
        self._init_timer.cancel()
        self.get_logger().info('Взлёт...')
        self._takeoff_pub.publish(Empty())
        # Через несколько секунд после взлёта начинаем облёт квадрата.
        self._init_timer = self.create_timer(3.0, self._begin_path)

    def _begin_path(self):
        self._init_timer.cancel()
        self._started = True
        self._phase_idx = 0
        self._phase_elapsed = 0.0
        self.get_logger().info('Полёт по квадрату...')
        self._tick_timer = self.create_timer(self._dt, self._tick)

    def _tick(self):
        if self._phase_idx >= len(self._phases):
            self._finish()
            return

        phase, duration = self._phases[self._phase_idx]
        twist = Twist()
        if phase == 'move':
            twist.linear.x = self.side_speed
        elif phase == 'turn':
            # Драйвер ждёт rad/s (по умолчанию) и сам переведёт в град/с.
            twist.angular.z = math.radians(self.turn_speed)
        self._cmd_pub.publish(twist)

        self._phase_elapsed += self._dt
        if self._phase_elapsed >= duration:
            self._phase_idx += 1
            self._phase_elapsed = 0.0
            if self._phase_idx < len(self._phases):
                nxt = self._phases[self._phase_idx][0]
                self.get_logger().info(f'Фаза: {nxt}')

    def _finish(self):
        self._tick_timer.cancel()
        self.get_logger().info('Зависание и посадка...')
        self._hover_pub.publish(Empty())
        self._land_pub.publish(Empty())
        self._end_timer = self.create_timer(3.0, self._shutdown)

    def _shutdown(self):
        self._end_timer.cancel()
        self.get_logger().info('Сценарий завершён.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SquareFlightExample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
