#!/usr/bin/env python3
"""Простейшая покатушка GIGAROVER: WASD в терминале -> SET_RPM по CAN.

Никакого ROS, демонов и веб-морд: один файл в стиле остальных скриптов
этого каталога (тот же bus-scan, тот же коаст при любом выходе). Запуск
по ssh на ровере:

    sudo systemctl stop rover-motord          # шиной владеет кто-то один!
    python3 drive.py
    sudo systemctl start rover-motord         # после покатушки

Управление (клавиши работают через автоповтор — держи клавишу):
    w/s     вперёд / назад
    a/d     разворот на месте влево / вправо
    +/-     скорость больше / меньше (шаг 300 eRPM)
    пробел  немедленный стоп (коаст)
    q       выход (коаст)

Отпустил клавишу — через 0.6 с команда обнуляется и моторы в коасте
(плюс у VESC свой CAN-timeout как вторая страховка).
"""
import glob
import select
import signal
import struct
import subprocess
import sys
import termios
import time
import tty

import can

P_SET_CURRENT, P_SET_RPM = 1, 3
P_STATUS = 9

# Раскладка ровера (как в ~/rover_config/motors.yaml): FL FR RL RR.
WHEELS = [  # (имя, can_id, знак: -1 = invert)
    ('FL', 32, -1),
    ('FR', 34, +1),
    ('RL', 104, -1),
    ('RR', 114, +1),
]
LEFT = ('FL', 'RL')

ERPM_START = 1500       # ~0.6 м/с
ERPM_STEP = 300
ERPM_MIN = 900          # ниже — зона срыва sensorless
ERPM_MAX = 3000
TURN_SCALE = 0.8        # разворот чуть медленнее прямой
HOLD_SEC = 0.6          # сколько живёт команда после последнего нажатия
TICK = 0.02             # 50 Гц


def send(bus, ptype, cid, payload):
    bus.send(can.Message(arbitration_id=(ptype << 8) | cid,
                         is_extended_id=True, data=payload))


def send_rpm(bus, cid, erpm):
    send(bus, P_SET_RPM, cid, struct.pack(">i", int(erpm)))


def coast_all(bus):
    for _, cid, _ in WHEELS:
        try:
            send(bus, P_SET_CURRENT, cid, struct.pack(">i", 0))
        except Exception:
            pass


def find_bus():
    """Живой порт перебором /dev/ttyUSB* — как в can_health.py."""
    for port in sorted(glob.glob("/dev/ttyUSB*")):
        try:
            b = can.Bus(interface="seeedstudio", channel=port,
                        bitrate=500000, timeout=0.1)
        except Exception as e:
            print(f"[scan] {port}: не открылся: {e}")
            continue
        deadline = time.time() + 3.0
        while time.time() < deadline:
            m = b.recv(timeout=0.2)
            if m is not None and m.is_extended_id:
                print(f"[scan] шина на {port}")
                return b
        b.shutdown()
        print(f"[scan] {port}: тишина")
    return None


def services_active():
    try:
        # rover-bringup здесь НЕ проверяем: ROS-стек ходит к шине только
        # через демона rover-motord (UDP), сам адаптер он не открывает.
        r = subprocess.run(
            ["systemctl", "is-active", "--quiet",
             "rover-motord", "rover-setup-web", "rover-can-teleop"],
            check=False, timeout=5)
        return r.returncode == 0
    except Exception:
        return False


def main():
    if services_active() and "--force" not in sys.argv:
        print("ОТКАЗ: активна служба, владеющая CAN (rover-motord/...).")
        print("Сначала: sudo systemctl stop rover-motord")
        return 1

    bus = find_bus()
    if bus is None:
        print("Кадров VESC нет ни на одном порту — питание/подключение?")
        return 1

    interactive = sys.stdin.isatty()
    fd = sys.stdin.fileno() if interactive else None
    old_tty = termios.tcgetattr(fd) if interactive else None
    if interactive:
        tty.setcbreak(fd)

    speed = ERPM_START
    mode = ' '            # w/s/a/d или ' ' (стоп)
    hold_until = 0.0
    tel = {cid: 0.0 for _, cid, _ in WHEELS}   # erpm из Status1 (сырое)
    was_moving = False

    print("w/s ехать, a/d разворот, +/- скорость, пробел стоп, q выход")
    try:
        next_tx = time.monotonic()
        while True:
            now = time.monotonic()

            # --- клавиатура (без блокировки) ---
            if interactive:
                while select.select([sys.stdin], [], [], 0)[0]:
                    ch = sys.stdin.read(1).lower()
                    if ch == 'q':
                        return 0
                    if ch == ' ':
                        mode, hold_until = ' ', 0.0
                        coast_all(bus)
                    elif ch in 'wsad':
                        mode, hold_until = ch, now + HOLD_SEC
                    elif ch in '+=':
                        speed = min(ERPM_MAX, speed + ERPM_STEP)
                    elif ch == '-':
                        speed = max(ERPM_MIN, speed - ERPM_STEP)

            if mode != ' ' and now >= hold_until:
                mode = ' '                      # клавишу отпустили

            # --- целевые eRPM по бортам ---
            if mode == 'w':
                left, right = speed, speed
            elif mode == 's':
                left, right = -speed, -speed
            elif mode == 'a':                   # влево: левый борт назад
                left, right = -speed * TURN_SCALE, speed * TURN_SCALE
            elif mode == 'd':
                left, right = speed * TURN_SCALE, -speed * TURN_SCALE
            else:
                left = right = 0.0

            # --- TX по расписанию 50 Гц ---
            if now >= next_tx:
                next_tx += TICK
                moving = left != 0.0 or right != 0.0
                if moving:
                    for name, cid, sign in WHEELS:
                        erpm = left if name in LEFT else right
                        send_rpm(bus, cid, sign * erpm)
                elif was_moving:
                    coast_all(bus)              # переход в стоп -> коаст
                was_moving = moving

            # --- дренаж RX (ограниченно, как везде) ---
            for _ in range(32):
                m = bus.recv(timeout=0)
                if m is None:
                    break
                if m.is_extended_id and (m.arbitration_id >> 8) & 0xFF == P_STATUS:
                    cid = m.arbitration_id & 0xFF
                    if cid in tel and len(m.data) >= 4:
                        tel[cid] = struct.unpack(">i", m.data[0:4])[0]

            # --- статусная строка ---
            if interactive:
                cells = ' '.join(
                    f"{name}:{sign * tel[cid]:+6.0f}"
                    for name, cid, sign in WHEELS)
                label = {'w': 'ВПЕРЁД', 's': 'НАЗАД ', 'a': 'ВЛЕВО ',
                         'd': 'ВПРАВО'}.get(mode, 'стоп  ')
                print(f"\r[{label}] {speed:4.0f} eRPM | {cells}   ",
                      end='', flush=True)

            time.sleep(0.002)
    except KeyboardInterrupt:
        return 0
    finally:
        for _ in range(5):
            coast_all(bus)
            time.sleep(0.02)
        bus.shutdown()
        if interactive:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_tty)
        print("\nкоаст всем, выход")


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(1))
    sys.exit(main())
