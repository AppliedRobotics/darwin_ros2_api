# Python API для дрона (Darwin Simulator)

Данный API реализует интерфейс, совместимый с **ARA-EDU `drone_control_api`** (класс `Drone` из модуля `Drone.py`). Это позволяет запускать одни и те же пользовательские скрипты как на реальном дроне, так и в симуляторе Darwin.

## Особенности симулятора

| Особенность | Описание |
|-------------|----------|
| **Аппаратные функции** | Пищалка, сервопривод, электромагнит, стрельба — не поддерживаются, генерируют исключение. |

---

## Класс `Drone`

Главный класс для управления дроном.

### `__init__()`

Создаёт объект управления дроном. Автоматически определяет контекст выполнения (симулятор) и инициализирует внутренние прокси.

**Атрибуты:**

| Атрибут | Описание |
|---------|----------|
| `platform` (`DronePlatform`) | Доступ к движению и светодиодам (legacy). |
| `sensors` (`Sensors`) | Доступ ко всем сенсорам (legacy). |

---

### Основные команды полёта (ARA-EDU naming)

| Метод | Описание |
|-------|----------|
| `takeoff() -> None` | Взлёт на высоту по умолчанию (задана в симуляторе, обычно 2 м). |
| `boarding() -> None` | Посадка дрона (опускание на стартовую высоту). |
| `setZeroOdomOpticflow() -> None` | Обнуляет одометрию оптического потока (сбрасывает накопленные координаты). |

---

### Телеметрия и сенсоры

Методы телеметрии возвращают данные текущего состояния дрона в симуляторе.

| Метод | Возвращает | Описание |
|-------|------------|----------|
| `getOdomOpticflow() -> (float, float)` | `(x, y)` | Накопленное смещение по осям X и Z в метрах (одометрия оптического потока). |
| `getLidar() -> float` | `float` | Расстояние до ближайшего препятствия в метрах (0 — нет препятствия / бесконечность). |
| `getRPY() -> (float, float, float)` | `(roll, pitch, yaw)` | Углы в **градусах** (крен, тангаж, рыскание). |
| `getHeightBarometer() -> float` | `float` | Высота по барометру (метры). В симуляторе равна абсолютной высоте над уровнем земли. |
| `getHeightRange() -> float` | `float` | Высота по дальномеру (метры). В симуляторе идентична `getHeightBarometer()`. |
| `getArm() -> bool` | `bool` | `True`, если дрон взведён (armed) — в симуляторе всегда `True` после `takeoff()`. |
| `getArucos() -> list` | `list[dict]` | Список обнаруженных ArUco-маркеров. Каждый элемент: `{"id": int, "x": float, "y": float}`. |
| `getCameraPoseAruco() -> dict` | `dict` | Позиция и поворот камеры относительно ArUco-маркера: `{"x", "y", "z", "roll", "pitch", "yaw"}`. |
| `getLight() -> list` | `list[float]` | Массив значений освещённости от всех датчиков (0..1 или люкс). |
| `getUltrasonic() -> list` | `list[float]` | Список расстояний (метры) от всех ультразвуковых датчиков. |
| `getBlobs() -> list` | `list[dict]` | Список цветовых пятен (blob). Каждый элемент: позиция, размер, цвет. |
| `getImage() -> bytes \\| None` | `bytes` \\| `None` | JPEG-изображение с первой камеры (байты). `None`, если камеры нет. |
| `getUtilsData() -> dict` | `dict` | Заглушка: `{"response": True}`. |

---

### Управление движением и светодиодами

Эти методы проксируют вызовы к `DronePlatform`.

| Метод | Описание |
|-------|----------|
| `setYaw(yaw: float) -> None` | Поворот на угол рыскания (градусы). Вращение по кратчайшей дуге. |
| `setVelXY(x: float, y: float) -> None` | Линейная скорость по X (вперёд/назад) и Y (влево/вправо) в м/с. Длительность — 0.1 с. |
| `setVelXYYaw(x: float, y: float, yaw: float) -> None` | Скорость по X, Y и угловая скорость рыскания (град/с). Длительность — 0.1 с. |
| `gotoXYdrone(x: float, y: float) -> None` | Перемещение в мировые координаты X и Z (метры). Дрон летит к точке и останавливается. |
| `gotoXYodom(x: float, y: float) -> None` | Перемещение относительно текущей одометрии (метры). |
| `setHeight(height: float) -> None` | Установить целевую высоту (метры). |
| `setDiod(r: float, g: float, b: float) -> None` | Цвет светодиода (RGB, 0–255 или 0–1 — адаптер приводит к byte). |
| `setBeeper(power: float, freq: float) -> None` | **Не поддерживается.** Генерирует исключение: `"error: not supported in simulator"`. |
| `setShoot(time_shoot: float) -> None` | **Не поддерживается.** Генерирует исключение. |
| `setServoAngle(angle: float) -> None` | **Не поддерживается.** Генерирует исключение. |
| `setMagnet(val: bool) -> None` | **Не поддерживается.** Генерирует исключение. |

---

## Класс `DronePlatform`

Доступ через `drone.platform` (или через `proxy.Platform` внутри API). Предоставляет низкоуровневые команды движения и управления светодиодами.

| Метод | Описание |
|-------|----------|
| `takeOff(height: float = None) -> None` | Взлёт на `height` (метры). Если `None` — высота по умолчанию. |
| `land() -> None` | Посадка. |
| `moveForward(speed: float, time: float = None) -> None` | Вперёд. `speed` — % от макс. (0.1–1). `time` — секунды (`None` = бесконечно). |
| `moveBackward(speed: float, time: float = None) -> None` | Назад. |
| `moveLeft(speed: float, time: float = None) -> None` | Стрейф влево. |
| `moveRight(speed: float, time: float = None) -> None` | Стрейф вправо. |
| `turnLeft(speed: float, time: float = None) -> None` | Поворот налево (0–100 % от макс. угловой скорости). |
| `turnRight(speed: float, time: float = None) -> None` | Поворот направо. |
| `hover() -> None` | Зависание — обнуление всех скоростей. |
| `turnOnLed() -> None` | Включить светодиод. |
| `turnOffLed() -> None` | Выключить светодиод. |
| `setLedColor(r: int, g: int, b: int) -> None` | Цвет светодиода (RGB, 0–255). |
| `setLedIntensity(intensity: float) -> None` | Яркость (0 — выкл, 1 — макс). |
| `setVelocity(vx: float, vy: float, wyaw: float, time: float) -> None` | `vx` — вперёд/назад (м/с), `vy` — влево/вправо (м/с), `wyaw` — рыскание (град/с), `time` — секунды. |
| `goToXYWorld(x: float, y: float) -> None` | Перемещение в мировые координаты (X, Z). Блокирующая операция. |
| `goToXYOdometry(x: float, y: float) -> None` | Перемещение относительно текущей одометрии (смещение X, Z). |
| `setHeight(height: float) -> None` | Установить высоту полёта. |
| `setYaw(yaw: float) -> None` | Установить угол рыскания (градусы, 0–360). |
| `setVelXY(x: float, y: float) -> None` | Скорости по X и Y (м/с) на ~0.1 с (рывок). |
| `setVelXYYaw(x: float, y: float, yaw: float) -> None` | Скорости X, Y и угловая скорость рыскания. |
| `setDiod(r: float, g: float, b: float) -> None` | Цвет диода (0–255 или 0..1 — автоматическая адаптация). |

---

## Класс `Sensors`

Доступ через `drone.sensors` (или `proxy.Sensors`). Обеспечивает чтение всех сенсорных данных.

> **Примечание:** Реализация `Sensors` в симуляторе повторяет методы `drone.py`.

| Метод | Возвращает | Описание |
|-------|------------|----------|
| `getOdomOpticflow() -> (float, float)` | `(x, y)` | Одометрия оптического потока. |
| `getLidar() -> float` | `float` | Расстояние от лидара до ближайшего препятствия. |
| `getRPY() -> (float, float, float)` | `(roll, pitch, yaw)` | Углы Эйлера в **градусах**. |
| `getHeightBarometer() -> float` | `float` | Высота (барометр). |
| `getHeightRange() -> float` | `float` | Высота (дальномер). |
| `getArm() -> bool` | `bool` | Статус взведения. |
| `getArucos() -> list` | `list[dict]` | Список ArUco-маркеров. |
| `getCameraPoseAruco() -> dict` | `dict` | Позиция камеры относительно ArUco. |
| `getLight() -> list` | `list[float]` | Массив освещённости. |
| `getUltrasonic() -> list` | `list[float]` | Данные ультразвуковых дальномеров. |
| `getBlobs() -> list` | `list[dict]` | Цветовые пятна. |
| `getImage() -> bytes` | `bytes` | JPEG-изображение с первой камеры. |
| `getCameraCount() -> int` | `int` | Количество камер на дроне. |
| `getCameraImageBytes(camera_index: int, format: str) -> bytes` | `bytes` | Сырые байты изображения. `camera_index` — индекс камеры, `format` — формат (например, `"jpeg"`). |
| `getLightArray() -> list` | `list[float]` | Синоним `getLight()`. |
| `setZeroOdomOpticflow() -> None` | `–` | Сброс одометрии оптического потока. |

---

## Примеры использования

```python
from drone import Drone

drone = Drone()
drone.takeoff()
drone.setHeight(3.0)                 # подняться на 3 метра
drone.gotoXYdrone(-1.0, -1.0)          # лететь в точку (5, 2)
drone.setVelXY(2.0, 0.0)             # лететь вперёд 2 м/с

print("Высота:", drone.getHeightBarometer())
print("Координаты одометрии:", drone.getOdomOpticflow())

drone.setDiod(255, 0, 0)             # красный диод
drone.boarding()                     # посадка

# Квадратный полёт с использованием moveForward и turnLeft

from drone import Drone
import time

drone = Drone()

# Взлетаем
drone.takeoff()
time.sleep(2)

# Рисуем квадрат
for _ in range(4):
    drone.platform.moveForward(1, 1)   # 100% скорости, 1 секунды → 1 метра
    drone.platform.turnLeft(0.4, 1)      # поворот

# Зависаем на 2 секунды
drone.platform.hover()
time.sleep(2)

drone.boarding()

# Управление скоростями через setVelocity

from drone import Drone
import time

drone = Drone()
drone.takeoff()
time.sleep(2)

# Летим вперёд со скоростью 2 м/с в течение 3 секунд
drone.platform.setVelocity(vx=2.0, vy=0.0, wyaw=0.0, time=1.0)
time.sleep(3.5)  # ждём окончания команды

# Летим по диагонали: вперёд 1.5 м/с + вправо 1.5 м/с
drone.platform.setVelocity(vx=1.0, vy=1.0, wyaw=0.0, time=1.0)
time.sleep(2.5)

# Вращаемся на месте: угловая скорость 60 град/с, 2 секунды
drone.platform.setVelocity(vx=0.0, vy=0.0, wyaw=60.0, time=1.0)
time.sleep(2.5)

drone.boarding()

# Перемещение по координатам: goToXYWorld и setYaw

from drone import Drone
import time

drone = Drone()
drone.takeoff()
time.sleep(2)

# Летим в точку (5, 0) в мировых координатах
drone.platform.goToXYWorld(-2.0, -1.0)
time.sleep(1)  # ожидание достижения точки

# Разворачиваемся на 90 градусов
drone.platform.setYaw(90.0)
time.sleep(2)

# Летим в точку (5, 5)
drone.platform.goToXYWorld(-3.0, -3.0)
time.sleep(1)

# Возвращаемся домой (0, 0)
drone.platform.goToXYOdometry(0.0, 0.0)

drone.boarding()
```