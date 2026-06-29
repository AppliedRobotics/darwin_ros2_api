# ROS2-топики ноды `darwin_node`

Нода `darwin_node` (пакет `darwin_ros2_api`) — драйвер, который связывает
WebSocket API симулятора Darwin с ROS2. Она **публикует** телеметрию из
симулятора в топики и **принимает** управляющие команды из топиков,
транслируя их в вызовы API.

> Какие именно телеметрические топики будут созданы, зависит от типа
> робота и ответа `/capabilities` (при `auto_discover: true`) либо от
> параметра `streams`. Команды (`cmd_vel`, `takeoff` и т.д.) доступны
> всегда.

Имена топиков указаны относительно пространства имён ноды (по умолчанию
без namespace, т.е. `/imu/data`, `/cmd_vel` и т.д.).

---

## 1. Публикуемые топики (телеметрия)

**QoS:** `BEST_EFFORT`, `KEEP_LAST`, depth = 10 (профиль для сенсоров).
Исключение — топик `scan`, у которого свой профиль *sensor data* (см. ниже).

| Топик | Тип сообщения | Источник (WS-поток) | Описание |
|-------|---------------|---------------------|----------|
| `imu/data` | `sensor_msgs/Imu` | `/imu` | Ориентация робота. Углы Эйлера (roll, pitch, yaw в градусах) переводятся в кватернион `orientation`. Угловая скорость и линейное ускорение симулятором **не отдаются** (covariance[0] = -1 — данные недоступны). |
| `pose` | `geometry_msgs/PoseStamped` | `/position` | Позиция `(x, y, z)` и кватернион поворота робота в мировой системе координат. `frame_id` = `world_frame_id` (по умолчанию `map`). Только для дрона. |
| `odom` | `nav_msgs/Odometry` | `/position` | Одометрия **относительно точки старта**: первая позиция фиксируется как ноль. `frame_id` = `odom_frame_id` (`odom`), `child_frame_id` = `frame_id` (`base_link`). Скорости (twist) не заполняются — симулятор их не отдаёт. |
| `range` | `sensor_msgs/Range` | `/range` | Расстояние до ближайшего препятствия от одиночного дальномера. `radiation_type = INFRARED`, `field_of_view = 0.1` рад. |
| `camera_{N}/image_raw` | `sensor_msgs/Image` | `/image/{N}` | Изображение с камеры `N` (`0`, `1`, …). Кодировка `bgr8` (JPEG из симулятора декодируется в кадр). `frame_id` = `camera_{N}_link`. Требует `cv_bridge`/`cv2`. |
| `scan` | `sensor_msgs/LaserScan` | `/laser_distance/0` | Данные 2D-лидара. Углы и пределы дальности задаются параметрами `laser_*`. `frame_id` = `laser_frame_id` (`laser`). Доп. лидары (если есть) публикуются как `scan_{N}` с кадром `laser_{N}`. **Отдельный QoS** — профиль *sensor data* (`BEST_EFFORT` + `VOLATILE` + `KEEP_LAST`, depth = 5). |
| `distance_sensor` | `std_msgs/Float32MultiArray` | `/distance_sensor` | Массив показаний всех дальномеров (метры). |
| `barometer` | `std_msgs/Float32MultiArray` | `/barometer` | Показания барометра (высота, метры). |
| `light` | `std_msgs/Float32MultiArray` | `/light` | Массив значений освещённости. |
| `encoder` | `std_msgs/Float32MultiArray` | `/encoder` | Показания энкодеров колёс (для колёсной платформы). |
| `color` | `std_msgs/Float32MultiArray` | `/color` | Значения цветовых сенсоров (для колёсной платформы). |
| `touch` | `std_msgs/Int32MultiArray` | `/touch` | Массив тактильных датчиков (0/1). |
| `black_line` | `std_msgs/Int32MultiArray` | `/black_line` | Датчики линии (0/1, для колёсной платформы). |

### Системы координат (frame_id)

| Параметр | По умолчанию | Используется в |
|----------|--------------|----------------|
| `frame_id` | `base_link` | `imu/data`, `range`, центр дрона |
| `world_frame_id` | `map` | `pose` |
| `laser_frame_id` | `laser` | кадр `scan` (доп. лидары → `laser_{N}`) |
| `odom_frame_id` | `odom` | родитель `base_link` в TF и кадр `/odom` |

---

## TF (дерево преобразований)

По умолчанию нода публикует TF (`publish_tf: true`), чтобы стандартные
инструменты ROS (RViz, navigation и т.д.) работали «из коробки».

Дерево кадров:

```
odom
 └── base_link            (центр дрона)
      ├── laser           (+0.10 м по Z, лидар над дроном)
      └── camera_0_link   (+0.10 м по X, камера спереди)
```

| Преобразование | Тип | Источник | Описание |
|----------------|-----|----------|----------|
| `odom` → `base_link` | динамический | `/position` | **Одометрия относительно старта.** Первая позиция фиксируется как ноль, далее публикуется перемещение/поворот относительно неё. Так дрон стартует из начала кадра `odom`. |
| `base_link` → `laser` | статический | параметр | Смещение лидара вверх на `laser_z_offset` (по умолчанию 0.10 м). Доп. лидары → `laser_{N}`. |
| `base_link` → `camera_{N}_link` | статический | параметр | Смещение камеры вперёд на `camera_x_offset` (по умолчанию 0.10 м). |

Статические преобразования публикуются один раз при старте (latched) для
каждого обнаруженного лидара/камеры. Отключить генерацию TF целиком можно
параметром `publish_tf: false`.

> Примечание: топик `/pose` отдаёт **абсолютную** позицию из симулятора в
> кадре `world_frame_id` (`map`), тогда как `/odom` и TF `odom → base_link`
> отсчитываются **от точки старта**. Дерево TF строится от кадра `odom`.

---

## 2. Подписки (команды управления)

**QoS:** `RELIABLE`, `KEEP_LAST`, depth = 10.

| Топик | Тип сообщения | Вызов API / эффект |
|-------|---------------|--------------------|
| `cmd_vel` | `geometry_msgs/Twist` | **Дрон:** `setVelXYYaw([vx, vy, wz])`, где `vx = linear.x`, `vy = linear.y` (м/с), `wz = angular.z`. **Колёсная платформа:** доминирующая компонента переводится в команду движения (`moveForward`/`rotateLeft`/…) в процентах скорости. |
| `command` | `std_msgs/String` | Универсальный канал: содержимое — сырой JSON, отправляемый напрямую в `/control`. Даёт доступ ко **всем** методам API. |
| `set_led_color` | `std_msgs/ColorRGBA` | `setLedColor({r, g, b})`. Компоненты `r`, `g`, `b` берутся как целые (0–255). |
| `takeoff` | `std_msgs/Empty` | `takeoff` — взлёт. |
| `land` | `std_msgs/Empty` | `boarding` — посадка. |
| `hover` | `std_msgs/Empty` | `hover` — зависание (обнуление скоростей). |
| `stop` | `std_msgs/Empty` | **Дрон:** `hover`. **Колёсная платформа:** `stopMotors`. |

### Особенности `cmd_vel`

- `angular.z` трактуется как **рад/с** и переводится в град/с для API.
  Если задать параметр `cmd_vel_yaw_is_degrees: true`, значение
  передаётся как есть (град/с).
- `setVelXYYaw` в симуляторе действует ~0.1 с, поэтому для непрерывного
  движения публикуйте `cmd_vel` периодически (например, 10 Гц).
- Для колёсной платформы скорость масштабируется параметром
  `car_speed_scale` (cmd_vel м/с → проценты).

### Пример отправки сырой команды через `command`

```bash
ros2 topic pub --once /command std_msgs/String \
  "{data: '{\"method\":\"gotoXYWorld\",\"params\":{\"x\":2,\"y\":3}}'}"
```

---

## 3. Параметры ноды

Полный список параметров с описанием и значениями по умолчанию см. в
`config/darwin_params.yaml`. Кратко:

| Параметр | Тип | По умолчанию | Назначение |
|----------|-----|--------------|------------|
| `host` | string | `127.0.0.1` | Хост WebSocket-сервера симулятора. |
| `port` | int | `8765` | Порт WebSocket-сервера. |
| `auto_discover` | bool | `true` | Автоопределение потоков и типа робота через `/capabilities`. |
| `streams` | string[] | `[]` | Принудительный список потоков (переопределяет автоопределение). |
| `robot_type` | string | `""` | Принудительный тип робота: `Drone` / `ForWdCar`. |
| `frame_id` | string | `base_link` | Кадр робота (центр дрона). |
| `world_frame_id` | string | `map` | Кадр мировой позиции (`/pose`). |
| `laser_frame_id` | string | `laser` | Базовое имя кадра лидара (→ `laser_N`). |
| `publish_tf` | bool | `true` | Публиковать ли TF-преобразования. |
| `odom_frame_id` | string | `odom` | Кадр одометрии (родитель `base_link`). |
| `publish_odom` | bool | `true` | Публиковать топик `/odom` (относительно старта). |
| `convert_sim_to_ros` | bool | `true` | Конвертировать позу из СК симулятора (ось вверх Y) в СК ROS (ось вверх Z). |
| `laser_z_offset` | float | `0.10` | Высота лидара над `base_link` (м). |
| `camera_x_offset` | float | `0.10` | Смещение камеры вперёд от `base_link` (м). |
| `laser_angle_min` | float | `0.0` | Начальный угол лидара (рад). |
| `laser_angle_max` | float | `6.2831853` | Конечный угол лидара (рад, 2π). |
| `laser_range_min` | float | `0.0` | Мин. дальность лидара (м). |
| `laser_range_max` | float | `30.0` | Макс. дальность лидара (м). |
| `cmd_vel_yaw_is_degrees` | bool | `false` | Трактовать `cmd_vel.angular.z` как град/с. |
| `car_speed_scale` | float | `100.0` | Масштаб скорости для колёсной платформы. |

---

## 4. Запуск

Через launch-файл (рекомендуется — подтягивает `config/darwin_params.yaml`):

```bash
ros2 launch darwin_ros2_api darwin_node.launch.py host:=127.0.0.1 port:=8765
```

Напрямую:

```bash
ros2 run darwin_ros2_api darwin_node --ros-args -p host:=127.0.0.1 -p port:=8765
```

Просмотр доступных топиков после запуска:

```bash
ros2 topic list
ros2 topic echo /imu/data
```
