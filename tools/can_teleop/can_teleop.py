#!/usr/bin/env python3
"""GIGAROVER: запасной телеоп без ROS — VESC-ходовая напрямую по CAN
+ веб-морда для телефона + HTTP API.

Зачем: ROS 2 поверх этой CAN-шины показал себя ненадёжно, поэтому нужен
автономный фолбэк. Один процесс, два потока:

  * CAN-поток — единственный владелец адаптера. Цикл повторяет боевой
    драйвер rover_vesc_driver: TX SET_RPM по расписанию 50 Гц, дренаж RX
    ограниченными пачками (≤32 кадров), скид-стир микширование, слю-лимиты
    разгона, deadman (коаст при молчании команд), подпор min_erpm, жёсткий
    предел max_erpm, переоткрытие шины с бэкоффом, ток 0 всем при любом
    выходе. Телеметрия STATUS 1/2/4/5 копится по колёсам.
  * HTTP-сервер (stdlib, без зависимостей) — отдаёт web/index.html
    (телеоп с телефона) и JSON API, совместимое по формам запросов с
    /api/drive* веб-морды ROS-стека (см. README.md).

Владение адаптером: перед запуском остановите rover-bringup/rover-setup-web
(systemd-юнит rover-can-teleop делает это сам через Conflicts=). Скрипт
отказывается стартовать, пока эти службы активны (обход: --force).

Конфиг колёс и шины подхватывается из ~/rover_config/motors.yaml — тот же
файл и те же правила слияния, что у ROS-стека (ROVER_CONFIG_DIR учитывается,
волатильные /dev/ttyUSB* из файла заменяются на /dev/rover_can). Остальные
параметры — константы DEFAULTS ниже, значения совпадают с gigarover_v1.yaml.

Запуск на ровере:
    sudo systemctl stop rover-bringup rover-setup-web
    python3 can_teleop.py
Телефон (Wi-Fi точка GIGAROVER): http://10.42.0.1:8765
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import signal
import struct
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional

try:
    import can
except ImportError:                       # позволяет гонять UI/API без железа
    can = None
try:
    import yaml
except ImportError:
    yaml = None
try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None

APP_NAME = 'can_teleop'
APP_VERSION = '1.0'

# VESC CAN: arbitration_id = (packet_type << 8) | controller_id, extended id.
CAN_PACKET_SET_CURRENT = 1
CAN_PACKET_SET_RPM = 3
CAN_PACKET_STATUS = 9         # erpm, ток мотора, duty
CAN_PACKET_STATUS_2 = 14      # ампер-часы
CAN_PACKET_STATUS_4 = 16      # temp_fet, temp_motor, ток входа
CAN_PACKET_STATUS_5 = 27      # тахометр, v_in

CH340_VID_PID = (0x1A86, 0x7523)          # USB-CAN на CH340
ROVER_CAN_SYMLINK = '/dev/rover_can'      # udev 99-gigarover.rules
WHEEL_ORDER = ('front_left', 'front_right', 'rear_left', 'rear_right')
RX_DRAIN_PER_TICK = 32
SPEED_DEADBAND_MPS = 0.005
FREQ_WINDOW_SEC = 5.0                     # окно расчёта частот статусов
CELL_EMPTY_V = 3.3
CELL_FULL_V = 4.15

# Значения согласованы с gigarover_v1.yaml / components/web.yaml —
# менять здесь только вместе с ними (или через motors.yaml, где применимо).
DEFAULTS: dict[str, Any] = {
    'bind_address': '0.0.0.0',
    'port': 8765,
    'can_interface': 'seeedstudio',
    'can_channel': '',                    # '' = motors.yaml / rover_can / автопоиск
    'can_bitrate': 500000,
    'control_rate_hz': 50.0,
    'command_timeout_sec': 0.5,           # deadman: коаст при молчании команд
    'feedback_timeout_sec': 0.35,         # свежесть телеметрии
    'stop_hold_sec': 0.75,                # блокировка движения после /api/stop
    'wheel_can_ids': [57, 25, 92, 71],    # FL FR RL RR; правда — в motors.yaml
    'wheel_inverts': [True, True, False, False],
    'wheel_radius_m': 0.0825,
    'track_width_m': 0.40,
    'gear_ratio': 3.0,
    'motor_pole_pairs': 7,
    'max_erpm': 4000.0,
    'min_erpm': 900.0,                    # подпор: ниже sensorless срывается
    'max_wheel_speed_mps': 1.6,
    'max_wheel_accel_mps2': 1.5,          # торможение — 2x от этого
    'max_linear_speed_mps': 1.5,
    'max_angular_speed_radps': 3.0,
    'default_linear_speed_mps': 0.5,
    'default_angular_speed_radps': 1.0,
    'battery_cells': 6,
}

_T0 = time.time()


def log(message: str) -> None:
    stamp = time.strftime('%H:%M:%S')
    print(f'[{stamp}] {message}', flush=True)


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


# ---------------------------------------------------------------------------
# Конфигурация
# ---------------------------------------------------------------------------
@dataclass
class Config:
    bind_address: str = DEFAULTS['bind_address']
    port: int = DEFAULTS['port']
    can_interface: str = DEFAULTS['can_interface']
    can_channel: str = DEFAULTS['can_channel']
    can_bitrate: int = DEFAULTS['can_bitrate']
    control_rate_hz: float = DEFAULTS['control_rate_hz']
    command_timeout_sec: float = DEFAULTS['command_timeout_sec']
    feedback_timeout_sec: float = DEFAULTS['feedback_timeout_sec']
    stop_hold_sec: float = DEFAULTS['stop_hold_sec']
    wheel_can_ids: list[int] = field(default_factory=lambda: list(DEFAULTS['wheel_can_ids']))
    wheel_inverts: list[bool] = field(default_factory=lambda: list(DEFAULTS['wheel_inverts']))
    wheel_radius_m: float = DEFAULTS['wheel_radius_m']
    track_width_m: float = DEFAULTS['track_width_m']
    gear_ratio: float = DEFAULTS['gear_ratio']
    motor_pole_pairs: int = DEFAULTS['motor_pole_pairs']
    max_erpm: float = DEFAULTS['max_erpm']
    min_erpm: float = DEFAULTS['min_erpm']
    max_wheel_speed_mps: float = DEFAULTS['max_wheel_speed_mps']
    max_wheel_accel_mps2: float = DEFAULTS['max_wheel_accel_mps2']
    max_linear_speed_mps: float = DEFAULTS['max_linear_speed_mps']
    max_angular_speed_radps: float = DEFAULTS['max_angular_speed_radps']
    default_linear_speed_mps: float = DEFAULTS['default_linear_speed_mps']
    default_angular_speed_radps: float = DEFAULTS['default_angular_speed_radps']
    battery_cells: int = DEFAULTS['battery_cells']

    @property
    def erpm_per_mps(self) -> float:
        return (self.gear_ratio * 60.0 * self.motor_pole_pairs
                / (2.0 * math.pi * self.wheel_radius_m))


def motors_config_path() -> Path:
    config_dir = os.environ.get('ROVER_CONFIG_DIR', '').strip() or '~/rover_config'
    return Path(config_dir).expanduser() / 'motors.yaml'


def apply_motors_config(cfg: Config, path: Path) -> None:
    """Слить motors.yaml (экспорт rover-setup-web) — те же правила, что в launch.

    Волатильные /dev/ttyUSB*/ttyACM* из файла не пиним (нумерация плавает при
    других USB-serial устройствах): берём стабильный /dev/rover_can, если он
    есть, иначе оставляем автопоиск CH340.
    """
    if not path.exists():
        log(f'motors.yaml не найден ({path}) — использую встроенные дефолты')
        return
    if yaml is None:
        log(f'ВНИМАНИЕ: PyYAML не установлен — {path} не прочитан, дефолты')
        return
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except Exception as exc:
        log(f'ВНИМАНИЕ: {path} не читается ({exc}) — дефолты')
        return
    if not isinstance(data, dict):
        log(f'ВНИМАНИЕ: {path} не словарь — дефолты')
        return
    log(f'Конфиг моторов: {path}')

    can_section = data.get('can', {})
    can_section = can_section if isinstance(can_section, dict) else {}
    if 'bitrate' in can_section:
        try:
            cfg.can_bitrate = int(can_section['bitrate'])
        except (TypeError, ValueError):
            log('ВНИМАНИЕ: motors.yaml can.bitrate не целое — игнорирую')
    channel = str(can_section.get('channel', '')).strip()
    if channel.startswith('/dev'):
        volatile = channel.startswith(('/dev/ttyUSB', '/dev/ttyACM'))
        if not volatile:
            cfg.can_channel = channel
        elif Path(ROVER_CAN_SYMLINK).exists():
            cfg.can_channel = ROVER_CAN_SYMLINK
            log(f'motors.yaml can.channel {channel} заменён на стабильный '
                f'{ROVER_CAN_SYMLINK}')
        else:
            log(f'motors.yaml can.channel {channel} проигнорирован '
                '(волатильная нумерация USB) — автопоиск CH340')

    wheels = data.get('wheels', {})
    wheels = wheels if isinstance(wheels, dict) else {}
    if wheels:
        complete = all(
            isinstance(wheels.get(name), dict) and 'can_id' in wheels[name]
            for name in WHEEL_ORDER
        )
        ids: list[int] = []
        if complete:
            try:
                ids = [int(wheels[name]['can_id']) for name in WHEEL_ORDER]
            except (TypeError, ValueError):
                complete = False
        if complete:
            cfg.wheel_can_ids = ids
            cfg.wheel_inverts = [
                bool(wheels[name].get('invert', False)) for name in WHEEL_ORDER
            ]
        else:
            log('ВНИМАНИЕ: motors.yaml wheels неполный (нужны can_id для всех '
                'front_left/front_right/rear_left/rear_right) — дефолты')


# ---------------------------------------------------------------------------
# CAN-драйвер (порт боевого rover_vesc_driver без ROS)
# ---------------------------------------------------------------------------
@dataclass
class WheelTelemetry:
    """Последние декодированные значения одного VESC (сырые знаки)."""
    erpm: float = 0.0
    duty: float = 0.0
    motor_current: float = 0.0
    input_current: float = 0.0
    temp_fet: float = 0.0
    temp_motor: float = 0.0
    v_in: float = 0.0
    tacho: int = 0
    amp_hours: float = 0.0
    t_status: float = 0.0
    t_status4: float = 0.0
    t_status5: float = 0.0
    # диагностика линка по STATUS_1 (как в can_health.py)
    max_gap: float = 0.0
    gaps_over_150ms: int = 0


class VescCanDriver:
    """Единственный владелец CAN-адаптера; поток TX 50 Гц + дренаж RX."""

    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.signs = [-1.0 if inv else 1.0 for inv in cfg.wheel_inverts]
        self._lock = threading.Lock()
        # командное состояние (пишет HTTP, читает CAN-поток)
        self._target_mps = [0.0, 0.0, 0.0, 0.0]      # FL FR RL RR
        self._cmd_vx = 0.0
        self._cmd_wz = 0.0
        self._last_cmd: Optional[float] = None       # None = команд ещё не было
        self._stop_until = 0.0
        # состояние CAN-потока
        self._tel: dict[int, WheelTelemetry] = {
            cid: WheelTelemetry() for cid in cfg.wheel_can_ids
        }
        self._speed_mps = [0.0, 0.0, 0.0, 0.0]       # выход слю-лимитера
        self._coasting = True
        self._connected = False
        self._channel_used: Optional[str] = None
        self._last_error: Optional[str] = None
        self._reopens = 0
        self._freq_counts: dict[tuple[int, int], int] = {}
        self._freq_hz: dict[tuple[int, int], float] = {}
        self._freq_t0 = time.monotonic()
        self._log_state: dict[str, Optional[str]] = {}
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._can_loop, name='can_loop', daemon=True
        )

    def start(self) -> None:
        self._thread.start()

    # ---- команды с HTTP-стороны -------------------------------------------
    def set_command(self, vx: float, wz: float) -> dict[str, float]:
        """Принять команду (м/с, рад/с): клампы, микширование, отметка времени."""
        if not (math.isfinite(vx) and math.isfinite(wz)):
            raise ValueError('non-finite command')
        vx = clamp(float(vx), self.cfg.max_linear_speed_mps)
        wz = clamp(float(wz), self.cfg.max_angular_speed_radps)
        half_track = self.cfg.track_width_m / 2.0
        v_left = vx - wz * half_track
        v_right = vx + wz * half_track
        peak = max(abs(v_left), abs(v_right))
        if peak > self.cfg.max_wheel_speed_mps:
            scale = self.cfg.max_wheel_speed_mps / peak
            v_left *= scale
            v_right *= scale
        with self._lock:
            self._target_mps = [v_left, v_right, v_left, v_right]
            self._cmd_vx, self._cmd_wz = vx, wz
            self._last_cmd = time.monotonic()
        return {'linear_x': vx, 'angular_z': wz}

    def stop_soft(self) -> dict[str, float]:
        """Мягкий стоп: цель 0, съезд по слю-лимиту, дальше коаст."""
        return self.set_command(0.0, 0.0)

    def stop_hard(self) -> float:
        """Аварийный стоп: мгновенный коаст (как deadman) + блокировка движения."""
        hold = self.cfg.stop_hold_sec
        with self._lock:
            self._target_mps = [0.0, 0.0, 0.0, 0.0]
            self._cmd_vx = self._cmd_wz = 0.0
            self._last_cmd = None                    # deadman сработает сразу
            self._stop_until = time.monotonic() + hold
        log(f'STOP: коаст всем, блокировка {hold:.2f} с')
        return hold

    # ---- журнал без спама ---------------------------------------------------
    def _log_once(self, key: str, message: Optional[str]) -> None:
        if self._log_state.get(key) == message:
            return
        self._log_state[key] = message
        if message is not None:
            log(message)

    # ---- выбор порта --------------------------------------------------------
    def _detect_channel(self) -> Optional[str]:
        if list_ports is None:
            return None
        ports = list(list_ports.comports())
        matches = [p.device for p in ports
                   if p.vid == CH340_VID_PID[0] and p.pid == CH340_VID_PID[1]]
        if not matches:                   # запасной критерий — по описанию
            matches = [p.device for p in ports
                       if 'CH340' in (p.description or '')]
        if not matches:
            return None
        if len(matches) == 1:
            return matches[0]
        # Несколько одинаковых адаптеров: берём тот, где реально идёт
        # VESC-трафик (extended-id кадры), а не первый попавшийся.
        for device in matches:
            try:
                probe = can.Bus(interface=self.cfg.can_interface, channel=device,
                                bitrate=self.cfg.can_bitrate, timeout=0.1)
            except Exception:
                continue
            alive = False
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                try:
                    frame = probe.recv(timeout=0.2)
                except Exception:
                    break
                if frame is not None and frame.is_extended_id:
                    alive = True
                    break
            try:
                probe.shutdown()
            except Exception:
                pass
            if alive:
                log(f'{len(matches)} адаптеров CH340; VESC-трафик на {device}')
                return device
        log(f'{len(matches)} адаптеров CH340, трафика нет ни на одном; '
            f'беру {matches[0]} (проверьте питание VESC/шину)')
        return matches[0]

    def _resolve_channel(self) -> Optional[str]:
        if self.cfg.can_channel:
            return self.cfg.can_channel
        if Path(ROVER_CAN_SYMLINK).exists():
            return ROVER_CAN_SYMLINK
        return self._detect_channel()

    def _open_bus(self) -> Optional['can.BusABC']:
        if can is None:
            self._log_once('bus', 'python-can не установлен — CAN недоступен '
                                  '(pip install python-can pyserial)')
            self._last_error = 'python-can not installed'
            return None
        channel = self._resolve_channel()
        if not channel:
            self._log_once('bus', 'USB-CAN адаптер (CH340) не найден — жду')
            self._last_error = 'adapter not found'
            return None
        try:
            # Таймаут серийника щедрый: seeedstudio собирает кадр несколькими
            # ser.read() с ЭТИМ таймаутом (аргумент recv() игнорируется);
            # слишком короткий -> рассыпание кадров.
            bus = can.Bus(interface=self.cfg.can_interface, channel=channel,
                          bitrate=self.cfg.can_bitrate, timeout=0.1)
        except Exception as exc:
            self._log_once('bus', f'CAN не открылся на {channel}: {exc!r}')
            self._last_error = f'open failed: {exc}'
            return None
        self._log_once('bus', None)
        self._last_error = None
        self._channel_used = channel
        wheel_map = ' '.join(
            f'{name}=id{cid}{"(inv)" if inv else ""}'
            for name, cid, inv in zip(
                ('FL', 'FR', 'RL', 'RR'), self.cfg.wheel_can_ids,
                self.cfg.wheel_inverts)
        )
        log(f'CAN поднят: {self.cfg.can_interface}:{channel} '
            f'@ {self.cfg.can_bitrate} бит/с; колёса {wheel_map}')
        return bus

    # ---- низкоуровневая отправка -------------------------------------------
    @staticmethod
    def _send_frame(bus: 'can.BusABC', ptype: int, cid: int, payload: bytes) -> None:
        bus.send(can.Message(arbitration_id=(ptype << 8) | cid,
                             is_extended_id=True, data=payload))

    def _send_current(self, bus: 'can.BusABC', cid: int, amps: float) -> None:
        self._send_frame(bus, CAN_PACKET_SET_CURRENT, cid,
                         struct.pack('>i', int(amps * 1000.0)))

    def _send_rpm(self, bus: 'can.BusABC', cid: int, erpm: float) -> None:
        self._send_frame(bus, CAN_PACKET_SET_RPM, cid, struct.pack('>i', int(erpm)))

    # ---- телеметрия ---------------------------------------------------------
    def _decode_frame(self, frame: 'can.Message', now: float) -> None:
        if not frame.is_extended_id:
            return
        ptype = (frame.arbitration_id >> 8) & 0xFF
        cid = frame.arbitration_id & 0xFF
        tel = self._tel.get(cid)
        if tel is None:
            return
        d = frame.data
        with self._lock:
            try:
                if ptype == CAN_PACKET_STATUS and len(d) >= 8:
                    tel.erpm = float(struct.unpack('>i', d[0:4])[0])
                    tel.motor_current = struct.unpack('>h', d[4:6])[0] / 10.0
                    tel.duty = struct.unpack('>h', d[6:8])[0] / 1000.0
                    if tel.t_status > 0.0:
                        gap = now - tel.t_status
                        if gap > tel.max_gap:
                            tel.max_gap = gap
                        if gap > 0.15:
                            tel.gaps_over_150ms += 1
                    tel.t_status = now
                elif ptype == CAN_PACKET_STATUS_2 and len(d) >= 8:
                    tel.amp_hours = struct.unpack('>i', d[0:4])[0] / 1e4
                elif ptype == CAN_PACKET_STATUS_4 and len(d) >= 6:
                    tel.temp_fet = struct.unpack('>h', d[0:2])[0] / 10.0
                    tel.temp_motor = struct.unpack('>h', d[2:4])[0] / 10.0
                    tel.input_current = struct.unpack('>h', d[4:6])[0] / 10.0
                    tel.t_status4 = now
                elif ptype == CAN_PACKET_STATUS_5 and len(d) >= 6:
                    tel.tacho = int(struct.unpack('>i', d[0:4])[0])
                    tel.v_in = struct.unpack('>h', d[4:6])[0] / 10.0
                    tel.t_status5 = now
                else:
                    return
            except struct.error:
                return
            self._freq_counts[(cid, ptype)] = \
                self._freq_counts.get((cid, ptype), 0) + 1

    def _update_freqs(self, now: float) -> None:
        window = now - self._freq_t0
        if window < FREQ_WINDOW_SEC:
            return
        with self._lock:
            self._freq_hz = {k: n / window for k, n in self._freq_counts.items()}
            self._freq_counts = {}
            self._freq_t0 = now

    def _drain_rx(self, bus: 'can.BusABC', now: float) -> None:
        # recv(timeout=0) => драйвер делает ровно одну попытку сборки кадра
        # с таймаутом серийника. На живой шине кадр приходит почти мгновенно;
        # выходим при первом промахе, чтобы не зависать.
        for _ in range(RX_DRAIN_PER_TICK):
            frame = bus.recv(timeout=0)
            if frame is None:
                break
            self._decode_frame(frame, now)

    # ---- командный тракт ----------------------------------------------------
    def _send_commands(self, bus: 'can.BusABC', now: float, dt: float) -> None:
        with self._lock:
            targets = list(self._target_mps)
            last_cmd = self._last_cmd
            stop_hold = now < self._stop_until
        stale = last_cmd is None or (now - last_cmd) > self.cfg.command_timeout_sec
        coast = stale or stop_hold
        if coast != self._coasting:
            self._coasting = coast
            if coast:
                log('deadman: команд нет — коаст всем колёсам')
            else:
                log('поток команд активен')
        if coast:
            # Deadman и STOP всегда коастят (ток 0), без вариантов.
            self._speed_mps = [0.0, 0.0, 0.0, 0.0]
            for cid in self.cfg.wheel_can_ids:
                self._send_current(bus, cid, 0.0)
            return
        for i, cid in enumerate(self.cfg.wheel_can_ids):
            current = self._speed_mps[i]
            target = targets[i]
            same_dir = current == 0.0 or target == 0.0 or current * target > 0.0
            speeding_up = same_dir and abs(target) > abs(current)
            limit = (self.cfg.max_wheel_accel_mps2 if speeding_up
                     else 2.0 * self.cfg.max_wheel_accel_mps2)
            step = limit * dt
            if abs(target - current) <= step:
                current = target
            else:
                current += math.copysign(step, target - current)
            self._speed_mps[i] = current
            if abs(current) < SPEED_DEADBAND_MPS:
                self._send_current(bus, cid, 0.0)
                continue
            erpm = current * self.cfg.erpm_per_mps
            erpm = clamp(erpm, self.cfg.max_erpm)
            # Ненастроенный sensorless FOC ниже min_erpm не держит обороты и
            # уходит в срыв/долбёжку — поднимаем команду до порога с тем же
            # знаком. min_erpm=0 отключает подпор.
            if self.cfg.min_erpm > 0.0 and 0.0 < abs(erpm) < self.cfg.min_erpm:
                erpm = math.copysign(self.cfg.min_erpm, erpm)
            self._send_rpm(bus, cid, self.signs[i] * erpm)

    # ---- основной цикл ------------------------------------------------------
    def _can_loop(self) -> None:
        bus: Optional['can.BusABC'] = None
        backoff = 1.0
        period = 1.0 / self.cfg.control_rate_hz
        last_tick = time.monotonic()
        while not self._stop_event.is_set():
            if bus is None:
                self._connected = False
                bus = self._open_bus()
                if bus is None:
                    if self._stop_event.wait(backoff):
                        break
                    backoff = min(backoff * 2.0, 10.0)
                    continue
                backoff = 1.0
                self._connected = True
                self._reopens += 1
                last_tick = time.monotonic()
            now = time.monotonic()
            dt = max(0.001, min(0.1, now - last_tick))
            last_tick = now
            try:
                self._drain_rx(bus, now)
                self._send_commands(bus, now, dt)
            except Exception as exc:
                self._log_once('bus', f'CAN I/O ошибка: {exc!r}; переоткрываю')
                self._last_error = f'io error: {exc}'
                self._connected = False
                try:
                    bus.shutdown()
                except Exception:
                    pass
                bus = None
                continue
            self._update_freqs(now)
            elapsed = time.monotonic() - now
            if elapsed < period:
                self._stop_event.wait(period - elapsed)
        if bus is not None:
            for _ in range(3):            # страховка: коаст перед закрытием
                for cid in self.cfg.wheel_can_ids:
                    try:
                        self._send_current(bus, cid, 0.0)
                    except Exception:
                        pass
                time.sleep(0.02)
            try:
                bus.shutdown()
            except Exception:
                pass
        self._connected = False

    def close(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=3.0)

    # ---- снимок состояния для API ------------------------------------------
    def snapshot(self) -> dict[str, Any]:
        cfg = self.cfg
        now = time.monotonic()
        with self._lock:
            last_cmd = self._last_cmd
            stop_hold = now < self._stop_until
            cmd_vx, cmd_wz = self._cmd_vx, self._cmd_wz
            targets = list(self._target_mps)
            output = list(self._speed_mps)
            freq_hz = dict(self._freq_hz)
            tel_copy = {
                cid: WheelTelemetry(**vars(t)) for cid, t in self._tel.items()
            }
        deadman = last_cmd is None or (now - last_cmd) > cfg.command_timeout_sec
        wheels: dict[str, Any] = {}
        volts: list[float] = []
        amps_in: list[float] = []
        for i, (name, cid) in enumerate(zip(WHEEL_ORDER, cfg.wheel_can_ids)):
            tel = tel_copy[cid]
            sign = self.signs[i]
            t_last = max(tel.t_status, tel.t_status4, tel.t_status5)
            fresh = t_last > 0.0 and (now - t_last) <= cfg.feedback_timeout_sec
            if tel.t_status5 > 0.0 and (now - tel.t_status5) <= cfg.feedback_timeout_sec:
                volts.append(tel.v_in)
            if tel.t_status4 > 0.0 and (now - tel.t_status4) <= cfg.feedback_timeout_sec:
                amps_in.append(tel.input_current)
            wheels[name] = {
                'can_id': cid,
                'inverted': bool(cfg.wheel_inverts[i]),
                'fresh': fresh,
                'age_ms': round((now - t_last) * 1000.0) if t_last > 0.0 else None,
                'erpm': sign * tel.erpm + 0.0,      # +0.0 гасит «-0.0» в JSON
                'measured_mps': round(sign * tel.erpm / cfg.erpm_per_mps, 3) + 0.0,
                'duty': tel.duty,
                'motor_current': tel.motor_current,
                'input_current': tel.input_current,
                'temp_fet': tel.temp_fet,
                'temp_motor': tel.temp_motor,
                'v_in': tel.v_in,
                'tacho': int(sign) * tel.tacho,
                'amp_hours': tel.amp_hours,
                'status1_hz': round(freq_hz.get((cid, CAN_PACKET_STATUS), 0.0), 1),
                'status5_hz': round(freq_hz.get((cid, CAN_PACKET_STATUS_5), 0.0), 1),
                'max_gap_ms': round(tel.max_gap * 1000.0),
                'gaps_over_150ms': tel.gaps_over_150ms,
            }
        voltage = sum(volts) / len(volts) if volts else 0.0
        percentage = None
        if volts and cfg.battery_cells > 0:
            fraction = ((voltage / cfg.battery_cells - CELL_EMPTY_V)
                        / (CELL_FULL_V - CELL_EMPTY_V))
            percentage = round(max(0.0, min(1.0, fraction)), 3)
        input_current = sum(amps_in) if amps_in else 0.0
        return {
            'ok': True,
            'app': APP_NAME,
            'version': APP_VERSION,
            'robot': 'gigarover',
            'uptime_sec': round(time.time() - _T0, 1),
            'can': {
                'available': can is not None,
                'connected': self._connected,
                'interface': cfg.can_interface,
                'channel': self._channel_used,
                'bitrate': cfg.can_bitrate,
                'reopens': max(0, self._reopens - 1),
                'last_error': self._last_error,
            },
            'link_ok': self._connected and all(w['fresh'] for w in wheels.values()),
            'drive': {
                'deadman': deadman,
                'stop_hold': stop_hold,
                'age_sec': round(now - last_cmd, 3) if last_cmd is not None else None,
                'command': {'linear_x': cmd_vx, 'angular_z': cmd_wz},
                'wheels_target_mps': [round(v, 3) for v in targets],
                'wheels_output_mps': [round(v, 3) for v in output],
            },
            'battery': {
                'present': bool(volts),
                'voltage': round(voltage, 2),
                'percentage': percentage,
                'cells': cfg.battery_cells,
                'input_current': round(input_current, 1),
                'power_w': round(voltage * input_current, 1),
            },
            'wheels': wheels,
            'limits': {
                'linear_x': cfg.max_linear_speed_mps,
                'linear_y': 0.0,
                'angular_z': cfg.max_angular_speed_radps,
                'wheel_mps': cfg.max_wheel_speed_mps,
                'max_erpm': cfg.max_erpm,
                'min_erpm': cfg.min_erpm,
            },
            'defaults': {
                'linear_x': cfg.default_linear_speed_mps,
                'linear_y': 0.0,
                'angular_z': cfg.default_angular_speed_radps,
            },
            'timeout_sec': cfg.command_timeout_sec,
        }

    def drive_payload(self) -> dict[str, Any]:
        """Форма ответа /api/drive — как у веб-морды ROS-стека."""
        snap = self.snapshot()
        drive = snap['drive']
        return {
            'ok': True,
            'command_topic': 'can://vesc',
            'timeout_sec': snap['timeout_sec'],
            'defaults': snap['defaults'],
            'limits': {k: snap['limits'][k] for k in ('linear_x', 'linear_y', 'angular_z')},
            'active': not drive['deadman'] and not drive['stop_hold'],
            'age_sec': drive['age_sec'],
            'last_command': {
                'linear_x': drive['command']['linear_x'],
                'linear_y': 0.0,
                'angular_z': drive['command']['angular_z'],
            },
        }


# ---------------------------------------------------------------------------
# HTTP-сервер
# ---------------------------------------------------------------------------
WEB_ROOT = Path(__file__).resolve().parent / 'web'
MAX_BODY_BYTES = 64 * 1024


class TeleopServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    driver: VescCanDriver


class Handler(BaseHTTPRequestHandler):
    server: TeleopServer
    protocol_version = 'HTTP/1.1'

    # ---- служебное ----------------------------------------------------------
    def log_message(self, fmt: str, *args: Any) -> None:
        pass                              # не спамим journal на каждый запрос

    def _cors(self) -> None:
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

    def _send_json(self, payload: dict[str, Any],
                   status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def _send_error_json(self, status: HTTPStatus, message: str) -> None:
        self._send_json({'ok': False, 'error': message}, status)

    def _read_json_body(self) -> dict[str, Any]:
        length = int(self.headers.get('Content-Length') or 0)
        if length > MAX_BODY_BYTES:
            raise ValueError('body too large')
        raw = self.rfile.read(length) if length > 0 else b''
        if not raw:
            return {}
        data = json.loads(raw.decode('utf-8'))
        if not isinstance(data, dict):
            raise ValueError('JSON body must be an object')
        return data

    # ---- маршруты -----------------------------------------------------------
    def do_OPTIONS(self) -> None:  # noqa: N802
        self.send_response(HTTPStatus.NO_CONTENT)
        self._cors()
        self.send_header('Content-Length', '0')
        self.end_headers()

    def do_GET(self) -> None:  # noqa: N802
        driver = self.server.driver
        path = self.path.split('?', 1)[0]
        try:
            if path in ('/', '/index.html'):
                self._send_file(WEB_ROOT / 'index.html', 'text/html; charset=utf-8')
                return
            if path == '/favicon.ico':
                self.send_response(HTTPStatus.NO_CONTENT)
                self.send_header('Content-Length', '0')
                self.end_headers()
                return
            if path == '/api/health':
                snap = driver.snapshot()
                self._send_json({
                    'ok': True,
                    'app': APP_NAME,
                    'version': APP_VERSION,
                    'uptime_sec': snap['uptime_sec'],
                    'can_connected': snap['can']['connected'],
                    'link_ok': snap['link_ok'],
                })
                return
            if path == '/api/status':
                self._send_json(driver.snapshot())
                return
            if path == '/api/drive':
                self._send_json(driver.drive_payload())
                return
            self._send_error_json(HTTPStatus.NOT_FOUND, 'not found')
        except BrokenPipeError:
            pass
        except Exception as exc:
            log(f'GET {self.path} упал: {exc!r}')
            try:
                self._send_error_json(HTTPStatus.INTERNAL_SERVER_ERROR, str(exc))
            except Exception:
                pass

    def do_POST(self) -> None:  # noqa: N802
        driver = self.server.driver
        path = self.path.split('?', 1)[0]
        try:
            try:
                payload = self._read_json_body()
            except (ValueError, json.JSONDecodeError) as exc:
                self._send_error_json(HTTPStatus.BAD_REQUEST, f'bad JSON: {exc}')
                return
            if path == '/api/drive/command':
                try:
                    command = driver.set_command(
                        float(payload.get('linear_x', 0.0)),
                        float(payload.get('angular_z', 0.0)),
                    )
                except (TypeError, ValueError) as exc:
                    self._send_error_json(HTTPStatus.BAD_REQUEST, str(exc))
                    return
                self._send_json({'ok': True, 'command': {
                    'linear_x': command['linear_x'],
                    'linear_y': 0.0,
                    'angular_z': command['angular_z'],
                }})
                return
            if path == '/api/drive/stop':
                driver.stop_soft()
                self._send_json({'ok': True, 'command': {
                    'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0,
                }})
                return
            if path == '/api/stop':
                hold = driver.stop_hard()
                self._send_json({'ok': True, 'stopped': True, 'hold_sec': hold})
                return
            if path == '/api/heartbeat':      # совместимость со старым UI
                self._send_json({'ok': True})
                return
            self._send_error_json(HTTPStatus.NOT_FOUND, 'not found')
        except BrokenPipeError:
            pass
        except Exception as exc:
            log(f'POST {self.path} упал: {exc!r}')
            try:
                self._send_error_json(HTTPStatus.INTERNAL_SERVER_ERROR, str(exc))
            except Exception:
                pass

    def _send_file(self, path: Path, content_type: str) -> None:
        body = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)


# ---------------------------------------------------------------------------
# Запуск
# ---------------------------------------------------------------------------
def conflicting_services_active() -> list[str]:
    """Активные systemd-службы, владеющие тем же CAN-адаптером."""
    if shutil.which('systemctl') is None:
        return []
    active = []
    for unit in ('rover-bringup.service', 'rover-setup-web.service'):
        try:
            result = subprocess.run(
                ['systemctl', 'is-active', '--quiet', unit],
                check=False, timeout=5.0,
            )
        except Exception:
            return []
        if result.returncode == 0:
            active.append(unit)
    return active


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='GIGAROVER: запасной телеоп по CAN без ROS '
                    '(веб-морда + HTTP API)')
    parser.add_argument('--host', default=DEFAULTS['bind_address'],
                        help='адрес HTTP-сервера (по умолчанию 0.0.0.0)')
    parser.add_argument('--port', type=int, default=DEFAULTS['port'],
                        help='порт HTTP-сервера (по умолчанию 8765)')
    parser.add_argument('--channel', default='',
                        help='серийный порт CAN-адаптера; пусто = motors.yaml '
                             f'/ {ROVER_CAN_SYMLINK} / автопоиск CH340')
    parser.add_argument('--motors-config', default='',
                        help='путь к motors.yaml (по умолчанию '
                             '$ROVER_CONFIG_DIR/motors.yaml или '
                             '~/rover_config/motors.yaml)')
    parser.add_argument('--force', action='store_true',
                        help='стартовать даже при активных rover-bringup/'
                             'rover-setup-web (ОПАСНО: два писателя на шине)')
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    log(f'{APP_NAME} v{APP_VERSION} — запасной телеоп GIGAROVER (без ROS)')

    active = conflicting_services_active()
    if active and not args.force:
        log(f'ОТКАЗ: активны службы {", ".join(active)} — они владеют тем же '
            'CAN-адаптером (два писателя дадут кашу на шине).')
        log('Остановите их: sudo systemctl stop rover-bringup rover-setup-web')
        log('(или запустите через systemd: sudo systemctl start '
            'rover-can-teleop — он остановит их сам через Conflicts=)')
        return 1

    cfg = Config()
    motors_path = (Path(args.motors_config).expanduser()
                   if args.motors_config else motors_config_path())
    apply_motors_config(cfg, motors_path)
    if args.channel:
        cfg.can_channel = args.channel
    cfg.bind_address = args.host
    cfg.port = args.port

    if can is None:
        log('ВНИМАНИЕ: python-can не установлен — поднимаю только веб/API, '
            'шина будет недоступна')

    driver = VescCanDriver(cfg)
    driver.start()

    try:
        httpd = TeleopServer((cfg.bind_address, cfg.port), Handler)
    except OSError as exc:
        log(f'HTTP-сервер не поднялся на {cfg.bind_address}:{cfg.port}: {exc}')
        driver.close()
        return 1
    httpd.driver = driver

    def _sig_handler(signum: int, _frame: Any) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _sig_handler)
    signal.signal(signal.SIGINT, _sig_handler)

    log(f'Веб-телеоп: http://{cfg.bind_address}:{cfg.port} '
        '(с телефона в сети GIGAROVER: http://10.42.0.1:8765)')
    log('API: GET /api/status /api/drive /api/health; '
        'POST /api/drive/command /api/drive/stop /api/stop')
    try:
        httpd.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        log('Остановка…')
    finally:
        driver.close()                    # коаст всем колёсам внутри
        httpd.server_close()
    log('Готово: моторы в коасте, сервер закрыт.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
