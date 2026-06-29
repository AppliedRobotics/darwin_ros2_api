#!/usr/bin/env python3
"""ROS2-драйвер для Darwin Simulator (WebSocket API).

Нода подключается к WebSocket-серверу симулятора, определяет тип робота и
доступные потоки через ``/capabilities``, после чего:

* поднимает отдельное WebSocket-соединение на каждый телеметрический поток
  и транслирует данные в соответствующие ROS2-топики;
* открывает соединение с ``/control`` и принимает управляющие команды из
  ROS2-топиков, преобразуя их в вызовы методов API.

Запуск::

    ros2 run darwin_ros2_api darwin_node --ros-args \
        -p host:=127.0.0.1 -p port:=8765
"""

import base64
import math
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from std_msgs.msg import (
    ColorRGBA,
    Float32MultiArray,
    Header,
    Int32MultiArray,
    String,
)
from sensor_msgs.msg import Image, Imu, LaserScan, Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

try:
    from cv_bridge import CvBridge
    import cv2
    _HAVE_CV = True
except Exception:  # noqa: BLE001
    _HAVE_CV = False

from darwin_ros2_api.ws_client import WSStream, query_capabilities


# Полный набор известных телеметрических потоков (без индексированных,
# которые добавляются динамически по capabilities).
KNOWN_STREAMS = [
    '/imu',
    '/position',
    '/distance_sensor',
    '/range',
    '/barometer',
    '/light',
    '/touch',
    '/encoder',
    '/color',
    '/black_line',
]


def euler_deg_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """Конвертирует углы Эйлера (в градусах) в кватернион (x, y, z, w)."""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def quat_mul(a, b):
    """Произведение кватернионов a * b. Формат (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conjugate(q):
    """Сопряжённый кватернион (для единичного — обратный). Формат (x,y,z,w)."""
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_rotate_vec(q, v):
    """Поворачивает вектор v=(x,y,z) кватернионом q=(x,y,z,w)."""
    qv = (v[0], v[1], v[2], 0.0)
    r = quat_mul(quat_mul(q, qv), quat_conjugate(q))
    return [r[0], r[1], r[2]]


def quat_to_rotmat(q):
    """Кватернион (x,y,z,w) -> матрица поворота 3x3 (numpy)."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotmat_to_quat(R):
    """Матрица поворота 3x3 -> кватернион (x,y,z,w)."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


# Матрица смены базиса из системы симулятора (Unity-стиль: левая тройка,
# ось "вверх" = Y, "вперёд" = Z) в систему ROS REP-103 (правая тройка,
# "вперёд" = X, "влево" = Y, "вверх" = Z). Соотношения осей:
#   ros_x =  sim_z   (вперёд)
#   ros_y = -sim_x   (влево)
#   ros_z =  sim_y   (вверх)
# Матрица ортогональна и имеет det = -1 (отражение, меняющее хиральность),
# поэтому поворот преобразуется через сопряжение  R_ros = M R_sim M^T.
SIM_TO_ROS = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])


def sim_pose_to_ros(pos, quat):
    """Переводит позицию и кватернион из СК симулятора в СК ROS.

    :param pos: (x, y, z) в системе симулятора.
    :param quat: (x, y, z, w) в системе симулятора.
    :return: (pos_ros: list, quat_ros: tuple).
    """
    p = np.array([pos[0], pos[1], pos[2]], dtype=float)
    p_ros = SIM_TO_ROS @ p
    r = quat_to_rotmat(quat)
    r_ros = SIM_TO_ROS @ r @ SIM_TO_ROS.T
    return p_ros.tolist(), rotmat_to_quat(r_ros)


def extract_response(data):
    """Достаёт полезную нагрузку из ответа симулятора.

    Сервер присылает либо ``{"response": <payload>}``, либо
    ``{"image": "<base64>"}``, либо ошибку ``{"error": ...}``.
    """
    if not isinstance(data, dict):
        return data, None
    if 'error' in data:
        return None, data['error']
    if 'response' in data:
        return data['response'], None
    if 'image' in data:
        return data['image'], None
    return data, None


def as_float_list(value) -> List[float]:
    """Приводит ответ к списку float (скаляр -> список из одного элемента)."""
    if value is None:
        return []
    if isinstance(value, (int, float)):
        return [float(value)]
    if isinstance(value, (list, tuple)):
        out = []
        for v in value:
            try:
                out.append(float(v))
            except (TypeError, ValueError):
                out.append(0.0)
        return out
    return []


class DarwinNode(Node):
    """Главная нода-драйвер Darwin Simulator."""

    def __init__(self):
        super().__init__('darwin_node')

        # --- Параметры ---------------------------------------------------
        self.host = self.declare_parameter('host', '127.0.0.1').value
        self.port = int(self.declare_parameter('port', 8765).value)
        self.auto_discover = bool(
            self.declare_parameter('auto_discover', True).value)
        # Принудительный список потоков (если auto_discover=False или
        # capabilities недоступны).
        self.streams_override = list(
            self.declare_parameter('streams', []).value or [])
        # Принудительный тип робота: '', 'Drone', 'ForWdCar'.
        self.robot_type_param = str(
            self.declare_parameter('robot_type', '').value)
        self.frame_id = str(self.declare_parameter('frame_id', 'base_link').value)
        self.world_frame_id = str(
            self.declare_parameter('world_frame_id', 'map').value)
        self.laser_frame_id = str(
            self.declare_parameter('laser_frame_id', 'laser').value)
        # --- TF -----------------------------------------------------------
        # Публиковать ли преобразования TF (дефолтная обвязка для ROS).
        self.publish_tf = bool(
            self.declare_parameter('publish_tf', True).value)
        # Кадр одометрии — родитель base_link в дереве TF.
        self.odom_frame_id = str(
            self.declare_parameter('odom_frame_id', 'odom').value)
        # Публиковать ли топик одометрии (nav_msgs/Odometry).
        self.publish_odom = bool(
            self.declare_parameter('publish_odom', True).value)
        # Смещение лидара над центром дрона (base_link), метры.
        self.laser_z_offset = float(
            self.declare_parameter('laser_z_offset', 0.10).value)
        # Смещение камеры вперёд от центра дрона (base_link), метры.
        self.camera_x_offset = float(
            self.declare_parameter('camera_x_offset', 0.10).value)
        # Параметры лидара (симулятор отдаёт только массив дистанций).
        self.laser_angle_min = float(
            self.declare_parameter('laser_angle_min', 0.0).value)
        self.laser_angle_max = float(
            self.declare_parameter('laser_angle_max', 2.0 * math.pi).value)
        self.laser_range_min = float(
            self.declare_parameter('laser_range_min', 0.0).value)
        self.laser_range_max = float(
            self.declare_parameter('laser_range_max', 30.0).value)
        # Множитель для cmd_vel.angular.z (rad/s -> град/с) для дрона.
        self.cmd_vel_yaw_is_degrees = bool(
            self.declare_parameter('cmd_vel_yaw_is_degrees', False).value)
        # Масштаб скорости для колёсной платформы: cmd_vel (м/с) -> проценты.
        self.car_speed_scale = float(
            self.declare_parameter('car_speed_scale', 100.0).value)
        # Конвертировать ли позу из СК симулятора (Unity-стиль, ось вверх Y)
        # в СК ROS REP-103 (ось вверх Z). Если выключить — поза идёт как есть.
        self.convert_sim_to_ros = bool(
            self.declare_parameter('convert_sim_to_ros', True).value)

        self._bridge = CvBridge() if _HAVE_CV else None
        self._streams: List[WSStream] = []
        self._control: Optional[WSStream] = None
        self.robot_type = self.robot_type_param

        # --- TF broadcasters ---------------------------------------------
        self._tf_broadcaster: Optional[TransformBroadcaster] = None
        self._static_tf_broadcaster: Optional[StaticTransformBroadcaster] = None
        # Накопленные статические трансформы (base_link -> сенсоры),
        # отправляются один раз после настройки потоков.
        self._static_transforms: List[TransformStamped] = []
        if self.publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self)
            self._static_tf_broadcaster = StaticTransformBroadcaster(self)

        # --- Одометрия от точки старта -----------------------------------
        # Первая полученная позиция/ориентация запоминается как "ноль".
        # Далее одометрия — перемещение относительно этого старта.
        self._odom_origin_pos: Optional[List[float]] = None
        self._odom_origin_quat_inv: Optional[tuple] = None
        self._odom_pub = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sensor_qos = sensor_qos

        # Отдельный QoS для лидара: стандартный профиль "sensor data"
        # (BEST_EFFORT + VOLATILE + KEEP_LAST, глубина 5). Многие потребители
        # сканов (RViz, slam_toolbox, nav2) ожидают именно его, иначе из-за
        # несовместимости QoS сообщения не доходят.
        self._laser_qos = QoSProfile(
            # reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        if self.publish_odom:
            self._odom_pub = self.create_publisher(
                Odometry, 'odom', self._sensor_qos)

        # --- Определение доступных потоков -------------------------------
        streams = self._discover_streams()
        self.get_logger().info(
            f'Тип робота: {self.robot_type or "неизвестно"}; '
            f'потоков для подписки: {len(streams)}')

        # --- Публикаторы и потоки телеметрии -----------------------------
        self._stream_pubs = {}
        for path in streams:
            self._setup_stream(path)

        # --- Управление --------------------------------------------------
        self._setup_control()

        # --- Статические TF (base_link -> сенсоры) -----------------------
        self._publish_static_transforms()

        self.get_logger().info('Darwin ROS2 драйвер запущен.')

    # ------------------------------------------------------------------
    # Capabilities / discovery
    # ------------------------------------------------------------------
    def _discover_streams(self) -> List[str]:
        if self.streams_override:
            self.get_logger().info('Используется параметр streams (override).')
            return self.streams_override

        if self.auto_discover:
            caps = query_capabilities(self.host, self.port)
            if caps is not None:
                if not self.robot_type:
                    self.robot_type = str(caps.get('robotType', ''))
                supported = caps.get('supportedStreams')
                if isinstance(supported, list) and supported:
                    return [str(s) for s in supported]
                self.get_logger().warn(
                    'capabilities не содержит supportedStreams, '
                    'используется набор по умолчанию.')
            else:
                self.get_logger().warn(
                    'Не удалось получить /capabilities, '
                    'используется набор по умолчанию.')

        # Набор по умолчанию: общие потоки + камера и лидар с индексом 0.
        return KNOWN_STREAMS + ['/image/0', '/laser_distance/0']

    # ------------------------------------------------------------------
    # Настройка телеметрических потоков
    # ------------------------------------------------------------------
    def _topic_name(self, path: str) -> str:
        return path.strip('/').replace('/', '_')

    def _setup_stream(self, path: str) -> None:
        handler = self._make_handler(path)
        if handler is None:
            self.get_logger().warn(f'Поток {path} не поддерживается драйвером, пропуск.')
            return

        url = f'ws://{self.host}:{self.port}{path}'
        name = self._topic_name(path)
        stream = WSStream(
            url,
            on_json=handler,
            on_status=lambda msg, p=path: self.get_logger().debug(f'[{p}] {msg}'),
        )
        stream.start()
        self._streams.append(stream)
        pub = self._stream_pubs.get(path)
        topic = pub.topic_name if pub is not None else name
        self.get_logger().info(f'Подписка на поток {path} -> топик {topic}')

    def _make_handler(self, path: str):
        """Создаёт обработчик входящих сообщений для конкретного потока."""
        if path == '/imu':
            pub = self.create_publisher(Imu, 'imu/data', self._sensor_qos)
            self._stream_pubs[path] = pub
            return lambda data: self._on_imu(pub, data)

        if path == '/position':
            pub = self.create_publisher(PoseStamped, 'pose', self._sensor_qos)
            self._stream_pubs[path] = pub
            return lambda data: self._on_position(pub, data)

        if path == '/range':
            pub = self.create_publisher(Range, 'range', self._sensor_qos)
            self._stream_pubs[path] = pub
            return lambda data: self._on_range(pub, data)

        if path.startswith('/image/'):
            if not _HAVE_CV:
                self.get_logger().warn(
                    'cv_bridge/cv2 недоступны — поток изображений пропущен.')
                return None
            idx = path.rsplit('/', 1)[-1]
            topic = f'camera_{idx}/image_raw'
            pub = self.create_publisher(Image, topic, self._sensor_qos)
            self._stream_pubs[path] = pub
            # Камера смотрит вперёд, смещена на camera_x_offset от base_link.
            cam_frame = f'camera_{idx}_link'
            self._register_static_tf(cam_frame, x=self.camera_x_offset)
            return lambda data: self._on_image(pub, data, cam_frame)

        if path.startswith('/laser_distance/'):
            idx = path.rsplit('/', 1)[-1]
            # Основной лидар (индекс 0) — без индекса в топике/фрейме.
            single = idx in ('0', '')
            topic = 'scan' if single else f'scan_{idx}'
            pub = self.create_publisher(LaserScan, topic, self._laser_qos)
            self._stream_pubs[path] = pub
            # Лидар приподнят на laser_z_offset над base_link.
            laser_frame = (self.laser_frame_id if single
                           else f'{self.laser_frame_id}_{idx}')
            self._register_static_tf(laser_frame, z=self.laser_z_offset)
            return lambda data: self._on_laser(pub, data, laser_frame)

        # Целочисленные массивы (0/1 датчики).
        if path in ('/touch', '/black_line'):
            topic = self._topic_name(path)
            pub = self.create_publisher(Int32MultiArray, topic, self._sensor_qos)
            self._stream_pubs[path] = pub
            return lambda data: self._on_int_array(pub, data)

        # Остальные числовые массивы.
        if path in ('/distance_sensor', '/barometer', '/light',
                    '/encoder', '/color'):
            topic = self._topic_name(path)
            pub = self.create_publisher(
                Float32MultiArray, topic, self._sensor_qos)
            self._stream_pubs[path] = pub
            return lambda data: self._on_float_array(pub, data)

        return None

    # ------------------------------------------------------------------
    # Обработчики телеметрии (вызываются из WS-потоков)
    # ------------------------------------------------------------------
    def _header(self, frame_id: str) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame_id
        return h

    def _on_imu(self, pub, data) -> None:
        payload, err = extract_response(data)
        if err is not None:
            return
        rpy = as_float_list(payload)
        if len(rpy) < 3:
            return
        msg = Imu()
        msg.header = self._header(self.frame_id)
        qx, qy, qz, qw = euler_deg_to_quaternion(rpy[0], rpy[1], rpy[2])
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        # Данные угловой скорости и ускорения симулятор не отдаёт.
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        pub.publish(msg)

    def _on_position(self, pub, data) -> None:
        payload, err = extract_response(data)
        if err is not None or not isinstance(payload, dict):
            return
        pos = as_float_list(payload.get('position'))
        rot = as_float_list(payload.get('rotation'))

        # Приводим позу из СК симулятора в СК ROS (ось вверх Z), иначе
        # вращение по yaw в симуляторе выглядит как pitch в TF.
        if len(pos) >= 3:
            quat = tuple(rot) if len(rot) >= 4 else (0.0, 0.0, 0.0, 1.0)
            if self.convert_sim_to_ros:
                pos, quat = sim_pose_to_ros(pos, quat)
            rot = list(quat)

        msg = PoseStamped()
        msg.header = self._header(self.world_frame_id)
        if len(pos) >= 3:
            msg.pose.position.x = pos[0]
            msg.pose.position.y = pos[1]
            msg.pose.position.z = pos[2]
        if len(rot) >= 4:
            msg.pose.orientation.x = rot[0]
            msg.pose.orientation.y = rot[1]
            msg.pose.orientation.z = rot[2]
            msg.pose.orientation.w = rot[3]
        else:
            msg.pose.orientation.w = 1.0
        pub.publish(msg)

        # Одометрия относительно точки старта + TF odom -> base_link.
        if len(pos) >= 3:
            self._publish_odometry(pos, rot)

    def _publish_odometry(self, pos: List[float], rot: List[float]) -> None:
        """Считает позу относительно старта и публикует odom-топик и TF.

        Первое сообщение фиксирует "ноль" одометрии: текущую позицию и
        ориентацию. Далее публикуется перемещение и поворот относительно
        этого старта, так что в кадре ``odom`` робот стартует из начала
        координат с нулевым поворотом.
        """
        if self._odom_pub is None and self._tf_broadcaster is None:
            return

        quat = tuple(rot) if len(rot) >= 4 else (0.0, 0.0, 0.0, 1.0)

        # Фиксируем точку старта при первом валидном сообщении.
        if self._odom_origin_pos is None:
            self._odom_origin_pos = [pos[0], pos[1], pos[2]]
            self._odom_origin_quat_inv = quat_conjugate(quat)

        # Смещение в мировых осях, повёрнутое в оси кадра старта (odom).
        dp = [
            pos[0] - self._odom_origin_pos[0],
            pos[1] - self._odom_origin_pos[1],
            pos[2] - self._odom_origin_pos[2],
        ]
        rel_pos = quat_rotate_vec(self._odom_origin_quat_inv, dp)
        rel_quat = quat_mul(self._odom_origin_quat_inv, quat)

        stamp = self.get_clock().now().to_msg()

        if self._odom_pub is not None:
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = self.odom_frame_id
            odom.child_frame_id = self.frame_id
            odom.pose.pose.position.x = rel_pos[0]
            odom.pose.pose.position.y = rel_pos[1]
            odom.pose.pose.position.z = rel_pos[2]
            odom.pose.pose.orientation.x = rel_quat[0]
            odom.pose.pose.orientation.y = rel_quat[1]
            odom.pose.pose.orientation.z = rel_quat[2]
            odom.pose.pose.orientation.w = rel_quat[3]
            # Скорости симулятор не отдаёт — оставляем нулевыми.
            self._odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame_id
            tf.child_frame_id = self.frame_id
            tf.transform.translation.x = rel_pos[0]
            tf.transform.translation.y = rel_pos[1]
            tf.transform.translation.z = rel_pos[2]
            tf.transform.rotation.x = rel_quat[0]
            tf.transform.rotation.y = rel_quat[1]
            tf.transform.rotation.z = rel_quat[2]
            tf.transform.rotation.w = rel_quat[3]
            self._tf_broadcaster.sendTransform(tf)

    def _on_range(self, pub, data) -> None:
        payload, err = extract_response(data)
        if err is not None:
            return
        values = as_float_list(payload)
        if not values:
            return
        msg = Range()
        msg.header = self._header(self.frame_id)
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.1
        msg.min_range = self.laser_range_min
        msg.max_range = self.laser_range_max
        msg.range = values[0]
        pub.publish(msg)

    def _on_image(self, pub, data, frame_id=None) -> None:
        payload, err = extract_response(data)
        if err is not None or not isinstance(payload, str):
            return
        try:
            raw = base64.b64decode(payload)
            arr = np.frombuffer(raw, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except Exception:  # noqa: BLE001
            return
        if frame is None:
            return
        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header = self._header(frame_id or self.frame_id)
        pub.publish(msg)

    def _on_laser(self, pub, data, frame_id=None) -> None:
        payload, err = extract_response(data)
        if err is not None:
            return
        ranges = as_float_list(payload)
        if not ranges:
            return
        msg = LaserScan()
        msg.header = self._header(frame_id or self.laser_frame_id)
        n = len(ranges)
        msg.angle_min = self.laser_angle_min
        msg.angle_max = self.laser_angle_max
        span = self.laser_angle_max - self.laser_angle_min
        msg.angle_increment = span / float(n) if n > 0 else 0.0
        msg.range_min = self.laser_range_min
        msg.range_max = self.laser_range_max
        msg.ranges = [float(r) for r in ranges]
        pub.publish(msg)

    def _on_float_array(self, pub, data) -> None:
        payload, err = extract_response(data)
        if err is not None:
            return
        values = as_float_list(payload)
        msg = Float32MultiArray()
        msg.data = values
        pub.publish(msg)

    def _on_int_array(self, pub, data) -> None:
        payload, err = extract_response(data)
        if err is not None:
            return
        values = [int(round(v)) for v in as_float_list(payload)]
        msg = Int32MultiArray()
        msg.data = values
        pub.publish(msg)

    # ------------------------------------------------------------------
    # TF
    # ------------------------------------------------------------------
    def _register_static_tf(self, child_frame_id: str,
                            x: float = 0.0, y: float = 0.0,
                            z: float = 0.0) -> None:
        """Добавляет статический TF base_link -> child_frame_id."""
        if not self.publish_tf:
            return
        tf = TransformStamped()
        tf.header = self._header(self.frame_id)
        tf.child_frame_id = child_frame_id
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.w = 1.0
        self._static_transforms.append(tf)

    def _publish_static_transforms(self) -> None:
        """Разово публикует все накопленные статические трансформы."""
        if (self._static_tf_broadcaster is not None
                and self._static_transforms):
            self._static_tf_broadcaster.sendTransform(self._static_transforms)
            self.get_logger().info(
                f'Опубликовано статических TF: {len(self._static_transforms)}')

    # ------------------------------------------------------------------
    # Управление: ROS2-топики -> /control
    # ------------------------------------------------------------------
    def _setup_control(self) -> None:
        url = f'ws://{self.host}:{self.port}/control'
        self._control = WSStream(
            url,
            on_json=self._on_control_response,
            on_status=lambda msg: self.get_logger().debug(f'[/control] {msg}'),
            send_on_open=None,  # /control ничего не ждёт при подключении
        )
        self._control.start()

        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Универсальный канал — сырой JSON, гарантирует доступ ко всем методам.
        self.create_subscription(
            String, 'command', self._on_command_raw, cmd_qos)
        # Скоростное управление.
        self.create_subscription(
            Twist, 'cmd_vel', self._on_cmd_vel, cmd_qos)
        # Удобные команды.
        self.create_subscription(
            ColorRGBA, 'set_led_color', self._on_set_led_color, cmd_qos)

        from std_msgs.msg import Empty
        self.create_subscription(
            Empty, 'takeoff', lambda _m: self._send_method('takeoff'), cmd_qos)
        self.create_subscription(
            Empty, 'land', lambda _m: self._send_method('boarding'), cmd_qos)
        self.create_subscription(
            Empty, 'hover', lambda _m: self._send_method('hover'), cmd_qos)
        self.create_subscription(
            Empty, 'stop', self._on_stop, cmd_qos)

    def _send_method(self, method: str, params=None) -> None:
        if self._control is None:
            return
        cmd = {'method': method}
        if params is not None:
            cmd['params'] = params
        ok = self._control.send(cmd)
        if not ok:
            self.get_logger().warn(
                f'Команда {method} не отправлена (нет соединения с /control).')

    def _on_control_response(self, data) -> None:
        if isinstance(data, dict) and 'error' in data:
            self.get_logger().warn(f'/control ответил ошибкой: {data["error"]}')

    def _on_command_raw(self, msg: String) -> None:
        import json
        try:
            cmd = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn(f'Невалидный JSON в ~/command: {msg.data[:80]}')
            return
        if self._control is None or not self._control.send(cmd):
            self.get_logger().warn('Не удалось отправить команду (нет соединения).')

    def _on_set_led_color(self, msg: ColorRGBA) -> None:
        self._send_method('setLedColor', {
            'r': int(msg.r),
            'g': int(msg.g),
            'b': int(msg.b),
        })

    def _on_stop(self, _msg) -> None:
        if self.robot_type == 'Drone':
            self._send_method('hover')
        else:
            self._send_method('stopMotors')

    def _on_cmd_vel(self, msg: Twist) -> None:
        if self.robot_type == 'Drone':
            self._cmd_vel_drone(msg)
        elif self.robot_type == 'ForWdCar':
            self._cmd_vel_car(msg)
        else:
            # Тип неизвестен — пробуем дроновый вариант как наиболее общий.
            self._cmd_vel_drone(msg)

    def _cmd_vel_drone(self, msg: Twist) -> None:
        # setVelXYYaw ожидает [vx, vy, wz], где wz в град/с.
        yaw = msg.angular.z
        if not self.cmd_vel_yaw_is_degrees:
            yaw = math.degrees(yaw)
        self._send_method('setVelXYYaw', [
            float(msg.linear.x),
            float(msg.linear.y),
            float(yaw),
        ])

    def _cmd_vel_car(self, msg: Twist) -> None:
        # Колёсная платформа управляется процентами скорости. Преобразуем
        # доминирующую компоненту cmd_vel в дискретную команду движения.
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        scale = self.car_speed_scale

        def pct(v):
            return max(0.0, min(100.0, abs(v) * scale))

        # Приоритет: поворот, затем продольное, затем поперечное движение.
        if abs(wz) >= abs(vx) and abs(wz) >= abs(vy) and abs(wz) > 1e-6:
            method = 'rotateLeft' if wz > 0 else 'rotateRight'
            self._send_method(method, {'speed': pct(wz)})
        elif abs(vx) >= abs(vy) and abs(vx) > 1e-6:
            method = 'moveForward' if vx > 0 else 'moveBackward'
            self._send_method(method, {'speed': pct(vx)})
        elif abs(vy) > 1e-6:
            method = 'moveLeft' if vy > 0 else 'moveRight'
            self._send_method(method, {'speed': pct(vy)})
        else:
            self._send_method('stopMotors')

    # ------------------------------------------------------------------
    def destroy_node(self):
        for stream in self._streams:
            stream.stop()
        if self._control is not None:
            self._control.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DarwinNode()
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
