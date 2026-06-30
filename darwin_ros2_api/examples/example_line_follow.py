#!/usr/bin/env python3
"""Пример: полёт по чёрной линии на белом фоне (камера).

Берём кадр с камеры, ищем чёрную линию в нижней части изображения
и поворачиваем дрон так, чтобы линия была по центру.

Перед запуском должен работать darwin_node::

    ros2 launch darwin_ros2_api darwin_node.launch.py

Запуск примера::

    ros2 run darwin_ros2_api example_line_follow

Посмотреть отладочную картинку (маска + центр линии)::

    ros2 run rqt_image_view rqt_image_view
    # выбрать топик /line_follow/debug_image

Или включи SHOW_WINDOW = True — откроется окно cv2.imshow.

Остановка: Ctrl+C

Топик камеры: ``/camera_1/image_raw`` (камера с индексом 1).
"""

import json
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String

# --- настройки (можно менять прямо здесь) ---
CAMERA_TOPIC = 'camera_1/image_raw'
DEBUG_TOPIC = 'line_follow/debug_image'
FLIGHT_HEIGHT = 0.3   # высота полёта, м
SPEED = 0.1           # скорость вперёд, м/с
TURN_GAIN = 1.0       # насколько сильно поворачивать при смещении линии
THRESHOLD = 80        # порог яркости: ниже = чёрная линия
PROCESS_WIDTH = 160   # ширина кадра для обработки (меньше = быстрее)
SHOW_WINDOW = False   # True — показать окно OpenCV на экране

bridge = CvBridge()
frame = None


def shrink(img):
    """Уменьшает кадр для быстрой обработки."""
    h, w = img.shape[:2]
    if w <= PROCESS_WIDTH:
        return img
    scale = PROCESS_WIDTH / w
    new_h = max(1, int(h * scale))
    return cv2.resize(img, (PROCESS_WIDTH, new_h), interpolation=cv2.INTER_AREA)


def on_image(msg):
    global frame
    try:
        frame = shrink(bridge.imgmsg_to_cv2(msg, 'bgr8'))
    except Exception:
        pass


def detect_line(img):
    """Ищет линию в нижней трети кадра.

    Возвращает (err, cx, mask, y0):
      err — смещение линии от центра (-1..+1) или None;
      cx  — x-координата центра линии или None;
      mask — маска ROI (для отладки);
      y0  — верхняя граница зоны поиска.
    """
    h, w = img.shape[:2]
    y0 = h * 2 // 3

    # обрабатываем только нижнюю треть — меньше пикселей
    gray = cv2.cvtColor(img[y0:, :], cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, THRESHOLD, 255, cv2.THRESH_BINARY_INV)

    # порог площади линии масштабируем под размер кадра
    min_area = max(15, int(500 * (w / 640.0) ** 2))
    moments = cv2.moments(mask)
    if moments['m00'] < min_area:
        return None, None, mask, y0

    cx = moments['m10'] / moments['m00']
    err = (cx - w / 2) / (w / 2)
    return err, cx, mask, y0


def draw_debug(img, err, cx, mask, y0):
    """Рисует зону поиска, центр кадра и найденную линию."""
    debug = img.copy()
    h, w = debug.shape[:2]

    cv2.rectangle(debug, (0, y0), (w - 1, h - 1), (0, 255, 0), 1)
    cv2.line(debug, (w // 2, y0), (w // 2, h - 1), (255, 0, 0), 1)

    if err is not None:
        cy = y0 + mask.shape[0] // 2
        cv2.circle(debug, (int(cx), cy), 4, (0, 0, 255), -1)
        cv2.putText(
            debug, f'err={err:.2f}', (5, 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    else:
        cv2.putText(
            debug, 'NO LINE', (5, 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

    # маска ROI в углу
    mh, mw = mask.shape[:2]
    preview = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    pw, ph = w // 3, mh
    preview = cv2.resize(preview, (pw, ph))
    debug[0:ph, w - pw:w] = preview

    return debug


def wait(node, seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def main():
    rclpy.init()
    node = Node('example_line_follow')

    cmd = node.create_publisher(Twist, 'cmd_vel', 10)
    takeoff = node.create_publisher(Empty, 'takeoff', 10)
    command = node.create_publisher(String, 'command', 10)
    debug_pub = node.create_publisher(Image, DEBUG_TOPIC, 10)
    node.create_subscription(Image, CAMERA_TOPIC, on_image, 10)

    wait(node, 2)

    node.get_logger().info('Взлёт')
    takeoff.publish(Empty())
    wait(node, 4)

    node.get_logger().info(f'Высота {FLIGHT_HEIGHT} м')
    msg = String()
    msg.data = json.dumps({'method': 'setHeight', 'params': FLIGHT_HEIGHT})
    command.publish(msg)
    wait(node, 2)

    node.get_logger().info(
        f'Следование по линии ({PROCESS_WIDTH}px), '
        f'отладка: /{DEBUG_TOPIC} (Ctrl+C для остановки)')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)

            if frame is None:
                continue

            err, cx, mask, y0 = detect_line(frame)
            debug = draw_debug(frame, err, cx, mask, y0)
            debug_pub.publish(bridge.cv2_to_imgmsg(debug, 'bgr8'))

            if SHOW_WINDOW:
                cv2.imshow('line_follow debug', debug)
                cv2.waitKey(1)

            twist = Twist()
            if err is None:
                node.get_logger().warn('Линия не найдена', throttle_duration_sec=1.0)
            else:
                twist.linear.x = SPEED
                twist.angular.z = -err * TURN_GAIN

            cmd.publish(twist)
            time.sleep(0.02)
    except KeyboardInterrupt:
        node.get_logger().info('Остановка')

    if SHOW_WINDOW:
        cv2.destroyAllWindows()
    cmd.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
