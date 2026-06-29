# Python API для колёсной платформы (FourWDCar) в Darwin Simulator

Данный API предназначен для управления роботом с механум-колёсами (или аналогичной платформой) в симуляторе Darwin. Класс `FourWDCar` предоставляет унифицированный интерфейс для управления движением, а также доступ к сенсорам и (опционально) манипулятору.

---

## Класс `FourWDCar`

**Атрибуты:**

| Атрибут | Описание |
|---------|----------|
| `platform` (`CarPlatform`) | Доступ ко всем командам движения (основной интерфейс). |
| `manipulator` (`Manipulator`) | Управление манипулятором (если присутствует; в симуляторе может быть заглушкой). |
| `sensors` (`Sensors`) | Доступ к сенсорной системе (аналогично дрону). |

```python
from fourWDCar import FourWDCar

robot = FourWDCar()
robot.platform.moveForward(50)   # ехать вперёд на 50% скорости
```

---

## Класс `CarPlatform`

Доступ через `robot.platform`. Содержит все методы управления движением.

### Общие параметры

| Параметр | Описание |
|----------|----------|
| `speed` | Скорость движения в процентах от максимальной (0–100). Положительное значение задаёт направление, соответствующее названию метода (например, `moveForward(50)` едет вперёд на 50%). |
| `ms` (опционально) | Время движения в миллисекундах. Если указано, робот движется заданное время, после чего автоматически останавливается. Если не указано – движение продолжается до вызова `stop()` или следующей команды. |

---

### Базовое движение (вперёд / назад)

| Метод | Описание |
|-------|----------|
| `moveForward(speed: float, ms: float = None) -> None` | Движение вперёд с заданной скоростью. |
| `moveBackward(speed: float, ms: float = None) -> None` | Движение назад. |

```python
# ехать вперёд бесконечно
robot.platform.moveForward(70)

# ехать назад 2 секунды (2000 мс)
robot.platform.moveBackward(50, ms=2000)

# остановить
robot.platform.stop()
```

---

### Управление отдельными двигателями (колёсами)

Платформа имеет 4 двигателя (колеса). Индексация:

- **0** – переднее левое
- **1** – переднее правое
- **2** – заднее левое
- **3** – заднее правое

| Метод | Описание |
|-------|----------|
| `moveForward_engine(speed: float, engine: int, ms: float = None) -> None` | Вращает указанный двигатель вперёд (колесо крутится в направлении движения платформы вперёд). |
| `moveBackward_engine(speed: float, engine: int, ms: float = None) -> None` | Вращает указанный двигатель назад. |

```python
# крутить только переднее левое колесо вперёд на 80% скорости
robot.platform.moveForward_engine(80, engine=0)

# крутить заднее правое колесо назад 1 секунду
robot.platform.moveBackward_engine(60, engine=3, ms=1000)
```

---

### Стрейф (боковое движение)

| Метод | Описание |
|-------|----------|
| `moveLeft(speed: float, ms: float = None) -> None` | Движение влево (поперечное смещение без поворота). |
| `moveRight(speed: float, ms: float = None) -> None` | Движение вправо. |

```python
robot.platform.moveRight(40)           # уехать вправо на 40% скорости
robot.platform.moveLeft(30, ms=1500)   # влево на 1.5 секунды
```

---

### Диагональное движение

| Метод | Описание |
|-------|----------|
| `moveRightUp(speed: float, ms: float = None) -> None` | Движение по диагонали вправо-вверх (вперёд + вправо). |
| `moveRightDown(speed: float, ms: float = None) -> None` | Диагональ вправо-вниз (назад + вправо). |
| `moveLeftUp(speed: float, ms: float = None) -> None` | Диагональ влево-вверх (вперёд + влево). |
| `moveLeftDown(speed: float, ms: float = None) -> None` | Диагональ влево-вниз (назад + влево). |

```python
robot.platform.moveRightUp(60)          # ехать по диагонали вперёд-вправо
robot.platform.moveLeftDown(50, ms=800) # диагональ назад-влево 0.8 сек
```

---

### Вращение на месте (поворот вокруг центра)

| Метод | Описание |
|-------|----------|
| `rotateLeft(speed: float, ms: float = None) -> None` | Поворот против часовой стрелки (вид сверху). |
| `rotateRight(speed: float, ms: float = None) -> None` | Поворот по часовой стрелке. |

```python
robot.platform.rotateLeft(100)          # крутиться на месте влево на полной скорости
robot.platform.rotateRight(70, ms=500)  # повернуться вправо за 0.5 секунды
```

---

### Вращение вокруг заданных точек платформы

Эти методы имитируют вращение робота вокруг определённого колеса или точки. Полезны для парковки, объезда препятствий и точных манёвров.

**Общие параметры:**

- `speed` – скорость вращения в процентах (0–100).
- `cw` (bool) – направление: `True` = по часовой стрелке, `False` = против часовой.
- `ms` – длительность в миллисекундах (опционально).

| Метод | Описание |
|-------|----------|
| `rotateAround_self(speed: float, cw: bool = True, ms: float = None)` | Вращение вокруг центра платформы (аналог `rotateLeft`/`rotateRight`, но направление задаётся явно). |
| `rotateAround_fl(speed: float, cw: bool = True, ms: float = None)` | Вращение вокруг переднего левого колеса. |
| `rotateAround_fr(speed: float, cw: bool = True, ms: float = None)` | Вращение вокруг переднего правого колеса. |
| `rotateAround_rl(speed: float, cw: bool = True, ms: float = None)` | Вращение вокруг заднего левого колеса. |
| `rotateAround_rr(speed: float, cw: bool = True, ms: float = None)` | Вращение вокруг заднего правого колеса. |
| `rotateAround_rear_mid(speed: float, cw: bool = True, ms: float = None)` | Вращение вокруг середины задней оси (между задними колёсами). |

```python
# развернуться вокруг переднего правого колеса по часовой стрелке
robot.platform.rotateAround_fr(60, cw=True, ms=1200)

# вращаться вокруг задней оси против часовой стрелки
robot.platform.rotateAround_rear_mid(50, cw=False)
```

---

### Остановка

| Метод | Описание |
|-------|----------|
| `stop() -> None` | Немедленно останавливает все двигатели. Обнуляет скорости и угловые скорости платформы (жёсткая остановка). |

```python
robot.platform.stop()
```

---

## Сенсоры (`Sensors`)

Доступны через `robot.sensors`. Номера датчиков по индексу):
- `getTouch(int index)` - датчик касания
- `getDistance(int index)` – расстояние до препятствия
- `getColor(int index)` – цвет
- `getBlackLine(int index)` - датчик чёрной линии
- `getEncoder(int index)` - энкодер


```python
dist = robot.sensors.getDistance(2)
if dist < 0.5:
    robot.platform.stop()
```

---

## Манипулятор (`Manipulator`)

Если робот оснащён манипулятором (рукой), управление доступно через `robot.manipulator`. Методы зависят от конкретной модели; в симуляторе могут быть заглушками.

---

## Примечания по симулятору

| Пункт | Описание |
|-------|----------|
| **Скорость** | Указана в процентах от максимальной (максимум 100%). Реальная линейная скорость (м/с) зависит от настроек платформы (обычно 1–2 м/с при 100%). |
| **Время `ms`** | Задаётся в миллисекундах. При указании `ms` команда блокирует выполнение Python-скрипта до окончания движения (через `time.sleep`). Будьте внимательны с длительными задержками – используйте отдельные потоки или асинхронность, если нужно параллельное выполнение. |
| **Остановка `stop()`** | Немедленная. Двигатели перестают получать управляющие сигналы, платформа останавливается с возможным проскальзыванием по инерции (в симуляторе инерция моделируется). |
| **Манипулятор** | В текущей версии симулятора может быть не реализован – вызовы его методов не будут иметь эффекта. |

---

## Полный список методов `CarPlatform`

| Метод | Назначение | Параметры |
|-------|------------|-----------|
| `moveForward(speed, ms=None)` | Движение вперёд | `speed`, `ms` |
| `moveBackward(speed, ms=None)` | Движение назад | `speed`, `ms` |
| `moveForward_engine(speed, engine, ms=None)` | Вращение отдельного двигателя вперёд | `speed`, `engine`, `ms` |
| `moveBackward_engine(speed, engine, ms=None)` | Вращение отдельного двигателя назад | `speed`, `engine`, `ms` |
| `moveLeft(speed, ms=None)` | Стрейф влево | `speed`, `ms` |
| `moveRight(speed, ms=None)` | Стрейф вправо | `speed`, `ms` |
| `moveRightUp(speed, ms=None)` | Диагональ вправо-вверх | `speed`, `ms` |
| `moveRightDown(speed, ms=None)` | Диагональ вправо-вниз | `speed`, `ms` |
| `moveLeftUp(speed, ms=None)` | Диагональ влево-вверх | `speed`, `ms` |
| `moveLeftDown(speed, ms=None)` | Диагональ влево-вниз | `speed`, `ms` |
| `rotateLeft(speed, ms=None)` | Поворот на месте против часовой | `speed`, `ms` |
| `rotateRight(speed, ms=None)` | Поворот на месте по часовой | `speed`, `ms` |
| `stop()` | Остановка всех двигателей | – |

---

## Примеры использования

### Пример 1. Движение по квадрату (с поворотами на месте)

```python
from fourWDCar import FourWDCar
import time

car = FourWDCar()

# Движение по квадрату
for _ in range(4):
    car.platform.moveForward(100, ms=1000)   # 1 секунды вперёд
    car.platform.rotateLeft(95, ms=2000)    # поворот 2 секунды
time.sleep(1)
car.platform.stop()
```

### Пример 2. Диагональное движение с остановкой по таймеру

```python
from fourWDCar import FourWDCar

car = FourWDCar()
car.platform.moveRightUp(60, ms=1500)   # диагональ вправо-вперёд 1.5 сек
car.platform.moveLeftDown(60, ms=1500)  # затем обратно
```

### Пример 3. Использование отдельного двигателя для тестирования

```python
from fourWDCar import FourWDCar
import time

car = FourWDCar()
# Проверка каждого колеса по очереди
for engine in range(4):
    print(f"Тест двигателя {engine}")
    car.platform.moveForward_engine(60, engine, ms=1000)
    time.sleep(1)
```

### Пример 4. Комбинация с датчиком расстояния

```python
from fourWDCar import FourWDCar
import time

car = FourWDCar()
car.platform.moveForward(40)
while True:
    dist = car.sensors.getDistance(1)
    if dist < 0.3:
        car.platform.stop()
        break
    time.sleep(0.05)
```

---