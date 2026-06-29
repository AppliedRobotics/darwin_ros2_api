#!/usr/bin/env python3
"""Простой пример управления дроном: взлёт -> зависание -> посадка.

Пример работает поверх ноды ``darwin_node`` и использует её командные
топики:

* ``takeoff`` (``std_msgs/Empty``) — взлёт на высоту по умолчанию;
* ``hover``   (``std_msgs/Empty``) — зависание (обнуление скоростей);
* ``land``    (``std_msgs/Empty``) — посадка.

Запуск (сначала должен быть запущен сам драйвер ``darwin_node``)::

    ros2 run darwin_ros2_api darwin_node --ros-args -p host:=127.0.0.1 -p port:=8765
    ros2 run darwin_ros2_api example_takeoff_land

Параметры (необязательно)::

    -p hover_time:=5.0   # сколько секунд висеть перед посадкой
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Empty


class TakeoffLandExample(Node):
    """Выполняет последовательность взлёт -> зависание -> посадка."""

    def __init__(self):
        super().__init__('example_takeoff_land')

        self.hover_time = float(
            self.declare_parameter('hover_time', 5.0).value)

        # Команды должны доставляться гарантированно — используем RELIABLE.
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._takeoff_pub = self.create_publisher(Empty, 'takeoff', cmd_qos)
        self._hover_pub = self.create_publisher(Empty, 'hover', cmd_qos)
        self._land_pub = self.create_publisher(Empty, 'land', cmd_qos)

        # Состояние простого "конечного автомата" сценария.
        self._stage = 0
        # Даём драйверу время установить соединение с /control и подписчикам
        # появиться, после чего запускаем сценарий по таймеру.
        self._timer = self.create_timer(2.0, self._run_sequence)

    def _run_sequence(self):
        if self._stage == 0:
            self.get_logger().info('Взлёт...')
            self._takeoff_pub.publish(Empty())
            # Ждём, пока дрон наберёт высоту, затем зависаем.
            self._stage = 1
            self._timer.cancel()
            self._timer = self.create_timer(3.0, self._run_sequence)

        elif self._stage == 1:
            self.get_logger().info(
                f'Зависание {self.hover_time:.1f} с...')
            self._hover_pub.publish(Empty())
            self._stage = 2
            self._timer.cancel()
            self._timer = self.create_timer(self.hover_time, self._run_sequence)

        elif self._stage == 2:
            self.get_logger().info('Посадка...')
            self._land_pub.publish(Empty())
            self._stage = 3
            self._timer.cancel()
            self._timer = self.create_timer(3.0, self._run_sequence)

        else:
            self.get_logger().info('Сценарий завершён.')
            self._timer.cancel()
            # Завершаем работу ноды.
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffLandExample()
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
