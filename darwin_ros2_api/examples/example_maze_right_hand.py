#!/usr/bin/env python3
"""Прохождение лабиринта дроном по правилу правой руки (лидар).

Реактивный контроллер следования вдоль правой стены: дрон держит
правую стену на заданном расстоянии, поворачивает налево перед
препятствием спереди и доворачивает направо, когда стена справа
"теряется" (открытый проход) — это и есть правило правой руки.

Используются:

* подписка на ``scan`` (``sensor_msgs/LaserScan``) — данные 2D-лидара;
* публикация ``cmd_vel`` (``geometry_msgs/Twist``) — скорости (драйвер
  переводит их в ``setVelXYYaw``); ``angular.z`` задаётся в рад/с;
* ``takeoff`` / ``hover`` / ``land`` (``std_msgs/Empty``).

Запуск (драйвер должен быть запущен заранее)::

    ros2 run darwin_ros2_api darwin_node --ros-args -p host:=127.0.0.1 -p port:=8765
    ros2 run darwin_ros2_api example_maze_right_hand

Полезные параметры::

    -p forward_speed:=0.4      # крейсерская скорость вперёд, м/с
    -p desired_right:=0.6      # желаемая дистанция до правой стены, м
    -p front_clearance:=0.8    # дистанция спереди, ниже которой крутим влево, м
    -p max_yaw_rate:=60.0      # макс. угловая скорость, град/с
    -p right_angle_deg:=-90.0  # направление "вправо" в кадре лидара, град
    -p front_angle_deg:=0.0    # направление "вперёд" в кадре лидара, град

Замечание о геометрии лидара: соответствие индексов массива дальностей
направлениям зависит от симулятора. По умолчанию принято, что 0° — это
"вперёд", а угол растёт против часовой стрелки (право = -90°). Если дрон
едет "не туда", подстройте ``front_angle_deg`` / ``right_angle_deg``.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


class MazeRightHandExample(Node):
    """Следование вдоль правой стены лабиринта по данным лидара."""

    def __init__(self):
        super().__init__('example_maze_right_hand')

        # --- Параметры движения -----------------------------------------
        self.forward_speed = float(
            self.declare_parameter('forward_speed', 0.4).value)
        self.desired_right = float(
            self.declare_parameter('desired_right', 0.6).value)
        self.front_clearance = float(
            self.declare_parameter('front_clearance', 0.8).value)
        self.max_yaw_rate = float(
            self.declare_parameter('max_yaw_rate', 60.0).value)  # град/с
        # П-коэффициент регулятора дистанции до правой стены (рад/с на метр).
        self.kp_wall = float(self.declare_parameter('kp_wall', 1.5).value)
        # Порог "потери" правой стены: дальше — считаем, что стены нет.
        self.right_lost = float(
            self.declare_parameter('right_lost', 1.2).value)

        # --- Геометрия лидара -------------------------------------------
        self.front_angle_deg = float(
            self.declare_parameter('front_angle_deg', 0.0).value)
        self.right_angle_deg = float(
            self.declare_parameter('right_angle_deg', -90.0).value)
        # Полуширина секторов усреднения (берём минимум в секторе), град.
        self.front_half_deg = float(
            self.declare_parameter('front_half_deg', 25.0).value)
        self.side_half_deg = float(
            self.declare_parameter('side_half_deg', 25.0).value)

        # --- Тайминги ----------------------------------------------------
        self.control_rate = float(
            self.declare_parameter('control_rate', 10.0).value)  # Гц
        self.takeoff_wait = float(
            self.declare_parameter('takeoff_wait', 4.0).value)

        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', cmd_qos)
        self._takeoff_pub = self.create_publisher(Empty, 'takeoff', cmd_qos)
        self._hover_pub = self.create_publisher(Empty, 'hover', cmd_qos)
        self._land_pub = self.create_publisher(Empty, 'land', cmd_qos)

        self._scan = None
        self.create_subscription(LaserScan, 'scan', self._on_scan, sensor_qos)

        self._active = False
        # Сначала взлетаем, даём набрать высоту, затем включаем контроллер.
        self._init_timer = self.create_timer(2.0, self._takeoff)

    # ------------------------------------------------------------------
    def _takeoff(self):
        self._init_timer.cancel()
        self.get_logger().info('Взлёт...')
        self._takeoff_pub.publish(Empty())
        self._init_timer = self.create_timer(self.takeoff_wait, self._begin)

    def _begin(self):
        self._init_timer.cancel()
        self._active = True
        self.get_logger().info(
            'Прохождение лабиринта по правилу правой руки...')
        self._ctrl_timer = self.create_timer(
            1.0 / self.control_rate, self._control_step)

    # ------------------------------------------------------------------
    def _on_scan(self, msg: LaserScan):
        self._scan = msg

    def _sector_min(self, scan: LaserScan, center_deg: float,
                    half_deg: float) -> float:
        """Минимальная валидная дальность в секторе вокруг center_deg.

        Возвращает ``inf``, если в секторе нет валидных измерений
        (нулевые/отрицательные значения трактуются как "нет препятствия").
        """
        center = math.radians(center_deg)
        half = math.radians(half_deg)
        best = float('inf')
        ang = scan.angle_min
        inc = scan.angle_increment
        for r in scan.ranges:
            a = ang
            ang += inc
            if not math.isfinite(r) or r <= 0.0 or r <= scan.range_min:
                continue
            # Разница углов, нормированная в [-pi, pi].
            diff = math.atan2(math.sin(a - center), math.cos(a - center))
            if abs(diff) <= half and r < best:
                best = r
        return best

    def _control_step(self):
        if not self._active:
            return
        scan = self._scan
        if scan is None or not scan.ranges:
            # Нет данных лидара — зависаем на месте.
            self._cmd_pub.publish(Twist())
            return

        front = self._sector_min(scan, self.front_angle_deg,
                                 self.front_half_deg)
        right = self._sector_min(scan, self.right_angle_deg,
                                 self.side_half_deg)

        max_yaw = math.radians(self.max_yaw_rate)
        twist = Twist()

        if front < self.front_clearance:
            # Препятствие спереди — поворачиваем налево на месте (CCW, +).
            twist.linear.x = 0.0
            twist.angular.z = max_yaw
        elif right > self.right_lost:
            # Стена справа потеряна (открытый проход) — доворачиваем
            # направо (CW, -) и одновременно движемся вперёд, чтобы
            # "обогнуть" угол по правилу правой руки.
            twist.linear.x = self.forward_speed * 0.6
            twist.angular.z = -max_yaw * 0.8
        else:
            # Едем вдоль правой стены, П-регулятор держит дистанцию.
            # err > 0 — слишком далеко от стены => поворот направо (-).
            err = right - self.desired_right
            yaw = -self.kp_wall * err
            twist.linear.x = self.forward_speed
            twist.angular.z = _clamp(yaw, -max_yaw, max_yaw)

        self._cmd_pub.publish(twist)

    # ------------------------------------------------------------------
    def stop_and_land(self):
        self._active = False
        try:
            self._hover_pub.publish(Empty())
            self._land_pub.publish(Empty())
        except Exception:  # noqa: BLE001
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MazeRightHandExample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Остановка: зависание и посадка.')
        node.stop_and_land()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
