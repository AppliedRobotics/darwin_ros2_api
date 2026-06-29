#!/usr/bin/env python3
"""Вспомогательные WebSocket-клиенты для Darwin Simulator.

Симулятор отдаёт каждый телеметрический поток и канал управления на
отдельном WebSocket-endpoint'е (например ``ws://host:port/imu``). Здесь
реализованы две сущности:

* :func:`query_capabilities` — одноразовый блокирующий запрос ``/capabilities``.
* :class:`WSStream` — постоянное соединение с авто-переподключением,
  работающее в собственном потоке. Используется как для подписки на
  телеметрию, так и для отправки команд в ``/control``.
"""

import json
import threading
import time
from typing import Callable, Optional

import websocket


def query_capabilities(host: str, port: int, timeout: float = 3.0) -> Optional[dict]:
    """Однократно запрашивает ``/capabilities`` и возвращает словарь ответа.

    Возвращает ``None``, если соединение не удалось или ответ невалиден.
    """
    url = f'ws://{host}:{port}/capabilities'
    try:
        conn = websocket.create_connection(url, timeout=timeout)
    except Exception:
        return None

    try:
        conn.send('{}')
        raw = conn.recv()
    except Exception:
        return None
    finally:
        try:
            conn.close()
        except Exception:
            pass

    try:
        data = json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        return None

    # Ответ может быть как {"response": {...}}, так и плоским словарём.
    if isinstance(data, dict) and isinstance(data.get('response'), dict):
        return data['response']
    return data if isinstance(data, dict) else None


class WSStream:
    """Постоянное WebSocket-соединение с авто-переподключением.

    Соединение работает в выделенном демоническом потоке. При обрыве
    связи поток будет пытаться переподключиться с интервалом
    ``reconnect_interval`` секунд, пока не будет вызван :meth:`stop`.

    :param url: полный адрес endpoint'а (``ws://host:port/path``).
    :param on_json: callback, вызываемый с распарсенным JSON каждого
        входящего сообщения. Вызывается из потока соединения.
    :param on_status: необязательный callback со строковым статусом
        (подключение/обрыв/ошибка) — удобно для логирования.
    :param send_on_open: строка, отправляемая сразу после открытия
        соединения. Для телеметрии симулятор ожидает пустой JSON ``{}``.
    """

    def __init__(
        self,
        url: str,
        on_json: Optional[Callable[[dict], None]] = None,
        on_status: Optional[Callable[[str], None]] = None,
        send_on_open: Optional[str] = '{}',
        reconnect_interval: float = 2.0,
        ping_interval: float = 20.0,
        ping_timeout: float = 10.0,
    ):
        self.url = url
        self._on_json = on_json
        self._on_status = on_status
        self._send_on_open = send_on_open
        self._reconnect_interval = reconnect_interval
        self._ping_interval = ping_interval
        self._ping_timeout = ping_timeout

        self._lock = threading.Lock()
        self._ws: Optional[websocket.WebSocketApp] = None
        self._connected = False
        self._running = False
        self._thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------
    # Жизненный цикл
    # ------------------------------------------------------------------
    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        with self._lock:
            ws = self._ws
        if ws is not None:
            try:
                ws.close()
            except Exception:
                pass

    @property
    def connected(self) -> bool:
        return self._connected

    def send(self, payload) -> bool:
        """Отправляет сообщение. ``payload`` может быть str/bytes или dict.

        Возвращает ``True`` при успешной отправке, ``False`` если
        соединение неактивно или произошла ошибка.
        """
        if isinstance(payload, (dict, list)):
            payload = json.dumps(payload)
        with self._lock:
            ws = self._ws
            connected = self._connected
        if not connected or ws is None:
            return False
        try:
            ws.send(payload)
            return True
        except Exception:
            return False

    # ------------------------------------------------------------------
    # Внутренняя логика потока
    # ------------------------------------------------------------------
    def _status(self, msg: str) -> None:
        if self._on_status is not None:
            try:
                self._on_status(msg)
            except Exception:
                pass

    def _run_loop(self) -> None:
        while self._running:
            ws = websocket.WebSocketApp(
                self.url,
                on_open=self._handle_open,
                on_message=self._handle_message,
                on_error=self._handle_error,
                on_close=self._handle_close,
            )
            with self._lock:
                self._ws = ws
            try:
                ws.run_forever(
                    ping_interval=self._ping_interval,
                    ping_timeout=self._ping_timeout,
                )
            except Exception as exc:  # noqa: BLE001
                self._status(f'run_forever exception: {exc}')

            self._connected = False
            if not self._running:
                break
            # Пауза перед переподключением.
            time.sleep(self._reconnect_interval)

    def _handle_open(self, ws: websocket.WebSocketApp) -> None:
        self._connected = True
        self._status('connected')
        if self._send_on_open is not None:
            try:
                ws.send(self._send_on_open)
            except Exception:
                pass

    def _handle_message(self, _ws: websocket.WebSocketApp, message: str) -> None:
        if self._on_json is None:
            return
        try:
            data = json.loads(message)
        except (json.JSONDecodeError, TypeError):
            return
        try:
            self._on_json(data)
        except Exception as exc:  # noqa: BLE001
            self._status(f'on_json exception: {exc}')

    def _handle_error(self, _ws: websocket.WebSocketApp, error: Exception) -> None:
        self._connected = False
        self._status(f'error: {error}')

    def _handle_close(self, _ws, status_code, msg) -> None:
        self._connected = False
        self._status(f'closed ({status_code}): {msg}')
