# WebSocket API для управления роботами в Darwin Simulator

Полное руководство по WebSocket-интерфейсу для управления дроном и колёсной платформой, а также для получения телеметрии.

---

## 1. Введение

WebSocket-сервер Darwin Simulator предоставляет унифицированный API для внешних клиентов (Python, Node.js, панели оператора и т.д.).

**Базовый адрес:** `ws://127.0.0.1:8765`

**Доступные сервисы:**

- Управление (`/control`) — отправка команд роботу.
- Capabilities (`/capabilities`) — получение информации о текущем роботе.
- Телеметрические потоки — подписка на сенсорные данные (позиция, IMU, изображения, дальномеры и т.д.).

---

## 2. Подключение и endpoint'ы

| Путь | Назначение |
|------|------------|
| `/control` | Отправка команд управления (движение, LED и т.д.) |
| `/capabilities` | Запрос возможностей текущего робота |
| `/position` | Позиция и поворот робота (только для дрона) |
| `/imu` | Углы Эйлера (roll, pitch, yaw) |
| `/distance_sensor` | Массив показаний всех дальномеров |
| `/range` | Ближайшее расстояние (один датчик) |
| `/barometer` | Показания барометра (высота) |
| `/light` | Массив значений освещённости |
| `/touch` | Массив тактильных датчиков |
| `/encoder` | Показания энкодеров колёс (для машины) |
| `/color` | Цветовые датчики (для машины) |
| `/black_line` | Датчики линии (для машины) |
| `/image/{index}` | Изображение с камеры (`0`, `1`, …) в формате JPEG (base64) |
| `/laser_distance/{index}` | Данные 2D-лидара (массив расстояний) |

---

## 3. Получение capabilities

```bash
wscat -c ws://127.0.0.1:8765/capabilities -x "{}" -w 2
```

Пример ответа для дрона:

```json
{
  "response": {
    "robotType": "Drone",
    "supportedMethods": ["takeoff", "boarding", "hover", "moveForward", "..."],
    "supportedStreams": ["/position", "/imu", "/image/0", "/laser_distance/0", "..."]
  }
}
```

- `robotType`: `Drone` или `ForWdCar`.
- `supportedMethods`: список команд, которые можно отправлять в `/control`.
- `supportedStreams`: список телеметрических путей, доступных для подписки.

---

## 4. Управление через `/control`

### Общий формат запроса

```json
{
  "method": "methodName",
  "commandId": "optional-unique-id",
  "params": { ... }
}
```

`params` может быть объектом, массивом или скаляром — в зависимости от метода.

### Ответы

- Успешная немедленная команда: `{"response":"OK"}`
- Успешная timed-команда: `{"response":"OK","commandId":"..."}`
- Прерывание предыдущей timed-команды: `{"error":"interrupted","commandId":"..."}`
- Ошибка: `{"error":"<error_code>"}`

### Возможные ошибки

| Код | Описание |
|-----|----------|
| `invalid_json` | Неверный формат JSON |
| `method_not_recognized` | Поле `method` отсутствует или неизвестно |
| `invalid_params` | Параметры не соответствуют контракту |
| `method_not_supported_for_current_robot` | Метод недоступен для данного типа робота |
| `api_input_mode_required` | Робот находится не в режиме API |
| `interrupted` | Предыдущая timed-команда была прервана новой командой |

---

## 5. Команды для дрона

### 5.1 Non-timed команды

| Метод | Параметры | Пример |
|------|-----------|--------|
| `takeoff` | нет | `{"method":"takeoff"}` |
| `boarding` | нет | `{"method":"boarding"}` |
| `hover` | нет | `{"method":"hover"}` |
| `moveForward` | `{ "speed": v }` или `v` | `{"method":"moveForward","params":30}` |
| `moveBackward` | аналогично | — |
| `moveLeft` | аналогично | — |
| `moveRight` | аналогично | — |
| `turnLeft` | аналогично | — |
| `turnRight` | аналогично | — |
| `setVelXYYaw` | `[vx, vy, wz]` | `{"method":"setVelXYYaw","params":[0.5,0,30]}` |
| `setHeight` | height (скаляр) | `{"method":"setHeight","params":2.5}` |
| `turnOnLed` | нет | — |
| `turnOffLed` | нет | — |
| `setLedColor` | `{ "r":0-255, "g":0-255, "b":0-255 }` | `{"method":"setLedColor","params":{"r":255,"g":0,"b":0}}` |
| `setLedIntensity` | `{ "intensity": i }` или скаляр | `{"method":"setLedIntensity","params":0.7}` |
| `gotoXYWorld` | `{ "x": x, "y": y }` | `{"method":"gotoXYWorld","params":{"x":2,"y":3}}` |
| `gotoXYOdometry` | аналогично | — |
| `gotoArUcoDrone` | `{ "id": id }` или скаляр | `{"method":"gotoArUcoDrone","params":5}` |

### Примеры через `wscat`

```bash
wscat -c ws://127.0.0.1:8765/control -x "{\"method\":\"takeoff\"}" -w 2
wscat -c ws://127.0.0.1:8765/control -x "{\"method\":\"moveForward\",\"params\":50}" -w 2
wscat -c ws://127.0.0.1:8765/control -x "{\"method\":\"setVelXYYaw\",\"params\":[1.0,0.0,45]}" -w 2
```

### 5.2 Timed-команды

Для методов движения и поворота существуют версии с суффиксом `Timed`.

**Параметры:** `{ "speed": v, "durationMs": ms }` или массив `[v, ms]`.

**Доступные timed-методы:**

- `moveForwardTimed`
- `moveBackwardTimed`
- `moveLeftTimed`
- `moveRightTimed`
- `turnLeftTimed`
- `turnRightTimed`

`setVelXYYaw` также может принимать `[vx, vy, wz, durationMs]`.

Пример: движение вперёд на 20% скорости в течение 1.5 секунд

```bash
wscat -c ws://127.0.0.1:8765/control -x "{\"method\":\"moveForwardTimed\",\"commandId\":\"drone-fwd1\",\"params\":{\"speed\":20,\"durationMs\":1500}}" -w 8
```

Ответ сервера:

```json
{"response":"OK","commandId":"drone-fwd1"}
```

По окончании движения сервер автоматически остановит дрон и закроет команду без дополнительного уведомления — только через `interrupted` в случае прерывания.

---

## 6. Команды для колёсной платформы

### 6.1 Non-timed команды

| Метод | Параметры | Примечание |
|------|-----------|------------|
| `stopMotors` | нет | Немедленная остановка |
| `pauseMovement` | нет | Приостановка движения (запоминает состояние) |
| `resumeMovement` | нет | Возобновление |
| `moveForward` / `moveBackward` | `{ "speed": v }` | Скорость в процентах (0–100) |
| `moveLeft` / `moveRight` | аналогично | Стрейф |
| `moveRightUp` / `moveRightDown` / `moveLeftUp` / `moveLeftDown` | аналогично | Диагональное движение |
| `rotateLeft` / `rotateRight` | аналогично | Поворот на месте |
| `rotateAroundSelf` / `rotateAroundFrontLeft` / `rotateAroundFrontRight` / `rotateAroundRearLeft` / `rotateAroundRearRight` / `rotateAroundRearMidpoint` | `{ "speed": v, "clockwise": true/false }` | Вращение вокруг заданной точки (по умолчанию `clockwise=true`) |
| `moveForwardByEngine` / `moveBackwardByEngine` | `{ "speed": v, "engineIndex": i }` | Управление отдельным колесом (индекс `0..3`) |

Примеры:

```bash
wscat -c ws://127.0.0.1:8765/control -x "{\"method\":\"moveForward\",\"params\":40}" -w 2
wscat -c ws://127.0.0.1:8765/control -x "{\"method\":\"rotateAroundRearLeft\",\"params\":{\"speed\":30,\"clockwise\":false}}" -w 2
wscat -c ws://127.0.0.1:8765/control -x "{\"method\":\"moveForwardByEngine\",\"params\":{\"speed\":60,\"engineIndex\":0}}" -w 2
```

### 6.2 Timed-команды

Для базовых движений и поворотов доступны те же суффиксы `Timed`.

**Параметры:** `{ "speed": v, "durationMs": ms }` или массив `[v, ms]`.

Для вращения вокруг точек — дополнительное поле `clockwise`.

Пример: поворот направо на 50% скорости в течение 1 секунды

```bash
wscat -c ws://127.0.0.1:8765/control -x "{\"method\":\"rotateRightTimed\",\"commandId\":\"car-turn1\",\"params\":{\"speed\":50,\"durationMs\":1000}}" -w 8
```

Для управления отдельным двигателем с таймером:

```bash
wscat -c ws://127.0.0.1:8765/control -x "{\"method\":\"moveForwardByEngineTimed\",\"params\":{\"speed\":80,\"engineIndex\":2,\"durationMs\":1200}}" -w 8
```

---

## 7. Телеметрические потоки

Для получения данных достаточно открыть WebSocket на соответствующем пути и отправить пустой запрос `{}`. Сервер будет периодически присылать обновления; частота зависит от конфигурации симулятора.

### 7.1 Общие потоки (Drone и Car)

| Путь | Тип данных | Пример ответа |
|------|------------|---------------|
| `/imu` | `[roll, pitch, yaw]` | `{"response":[1.2,-0.3,45.0]}` |
| `/distance_sensor` | массив `float` | `{"response":[0.5,1.2,0.8]}` |
| `/range` | одно значение | `{"response":0.75}` |
| `/barometer` | массив (высота) | `{"response":[2.34]}` |
| `/light` | массив освещённости | `{"response":[0.1,0.3,0.9]}` |
| `/touch` | массив тактильных датчиков | `{"response":[0,0,1]}` |
| `/encoder` | показания энкодеров (Car) | `{"response":[123.4,125.1,120.0,122.3]}` |
| `/color` | значения цветовых сенсоров (Car) | `{"response":[0.2,0.5,0.1]}` |
| `/black_line` | датчики линии (Car) | `{"response":[0,1,0]}` |

### 7.2 Специфичные для дрона

| Путь | Описание | Пример ответа |
|------|----------|---------------|
| `/position` | позиция `(x,y,z)` и кватернион поворота | `{"response":{"position":[1.0,2.0,3.0],"rotation":[0,0,0,1]}}` |
| `/laser_distance/0` | массив расстояний 2D-лидара (360 значений) | `{"response":[2.3,2.4,2.5,...]}` |
| `/image/0` | JPEG-изображение в base64 | `{"image":"/9j/4AAQSkZJRg..."}` |

### 7.3 Примеры получения телеметрии

```bash
wscat -c ws://127.0.0.1:8765/imu -x "{}" -w 1
wscat -c ws://127.0.0.1:8765/position -x "{}" -w 1
wscat -c ws://127.0.0.1:8765/black_line -x "{}" -w 1
```

Если робот не поддерживает запрошенный поток, сервер вернёт `404` или `{"error":"method_not_supported_for_current_robot"}` — зависит от реализации.

---

## 8. Примеры сценариев

### 8.1 Простая последовательность для дрона

```bash
# 1. Взлёт
wscat -c ws://127.0.0.1:8765/control -x "{\"method\":\"takeoff\"}" -w 2

# 2. Полёт вперёд 2 секунды
wscat -c ws://127.0.0.1:8765/control -x "{\"method\":\"moveForwardTimed\",\"params\":{\"speed\":40,\"durationMs\":2000}}" -w 4

# 3. Посадка
wscat -c ws://127.0.0.1:8765/control -x "{\"method\":\"boarding\"}" -w 2
```

### 8.2 Движение машины по квадрату

```bash
wscat -c ws://127.0.0.1:8765/control -x "{\"method\":\"moveForwardTimed\",\"params\":{\"speed\":50,\"durationMs\":2000}}" -w 4
wscat -c ws://127.0.0.1:8765/control -x "{\"method\":\"rotateLeftTimed\",\"params\":{\"speed\":50,\"durationMs\":1000}}" -w 4
wscat -c ws://127.0.0.1:8765/control -x "{\"method\":\"moveForwardTimed\",\"params\":{\"speed\":50,\"durationMs\":2000}}" -w 4
wscat -c ws://127.0.0.1:8765/control -x "{\"method\":\"rotateLeftTimed\",\"params\":{\"speed\":50,\"durationMs\":1000}}" -w 4
```

### 8.3 Прерывание timed-команды

Если во время выполнения timed-команды отправить другую команду, предыдущая будет прервана с ошибкой `interrupted`.

**Терминал A** — долгая команда:

```bash
wscat -c ws://127.0.0.1:8765/control -x "{\"method\":\"moveForwardTimed\",\"commandId\":\"long\",\"params\":{\"speed\":30,\"durationMs\":10000}}" -w 20
```

**Терминал B** — через 1 секунду:

```bash
wscat -c ws://127.0.0.1:8765/control -x "{\"method\":\"stopMotors\"}" -w 2
```

В терминале A появится:

```json
{"error":"interrupted","commandId":"long"}
```

---

## 9. Примечания и ограничения

| Пункт | Описание |
|------|------------|
| **Режим API** | Команды выполняются только когда робот находится в режиме API (в симуляторе переключается через PlayerData). Попытка отправить команду в ручном режиме вернёт `api_input_mode_required`. |
| **Параметры скорости** | Для дрона и машины `speed` задаётся в процентах (0–100). Значения за пределами `[0,100]` могут быть автоматически ограничены адаптером. |
| **Единицы времени** | Везде используются миллисекунды (`durationMs`). Внутри они преобразуются в секунды. |
| **Одновременные подключения** | Сервер поддерживает несколько клиентов, но управляющие команды направляются единственному активному роботу (выбранному в сцене). Multi-robot сценарии требуют доработки (привязка к `robotId`). |
| **Формат параметров** | Большинство методов принимают как объект `{ "speed": 30 }`, так и массив `[30]` или просто число `30`. В документации приведён объектный стиль для ясности. Исключение — `setVelXYYaw`, где всегда ожидается массив. |

---

## 10. Коды ошибок

| Код ошибки | Описание |
|------------|------------|
| `invalid_json` | Невалидный JSON в сообщении |
| `method_not_recognized` | Поле `method` отсутствует или содержит неизвестную строку |
| `invalid_params` | Неверный формат, тип или отсутствие обязательных параметров |
| `method_not_supported_for_current_robot` | Вызванный метод не поддерживается текущим типом робота |
| `api_input_mode_required` | Робот не в режиме API (ручное управление) |
| `interrupted` | Предыдущая timed-команда была прервана новой командой |

---

## Заключение

WebSocket API Darwin Simulator предоставляет полный набор команд для управления дроном и колёсной платформой, а также потоковые сенсорные данные. Используйте `wscat` для быстрого тестирования или подключайте собственных клиентов на любом языке, поддерживающем WebSocket.

Для углублённой работы рекомендуется ознакомиться с исходными адаптерами (`DroneControlAdapter`, `ForWdCarControlAdapter`) и конфигурацией сервера.