#!/usr/bin/env python3
"""Чтение телеметрии GIGAROVER по HTTP — батарея, колёса, одометрия.

Понимает оба API: аварийный телеоп rover-motord (:8767, есть телеметрия
колёс и батареи даже без ROS) и веб-шлюз ROS-стека (:8765, есть
одометрия и диагностика). Тип сервера определяется по ответу.

    python3 http_status.py                     # motord, разовый снимок
    python3 http_status.py --watch             # обновление раз в секунду
    python3 http_status.py --port 8765         # веб-шлюз ROS
    python3 http_status.py --host 192.168.2.72 # по Ethernet
"""
import argparse
import json
import time
import urllib.request


def fetch(base: str, path: str) -> dict:
    with urllib.request.urlopen(base + path, timeout=2.0) as response:
        return json.loads(response.read().decode("utf-8"))


def show_motord(status: dict) -> None:
    can = status.get("can", {})
    link = status.get("link", {})
    drive = status.get("drive", {})
    battery = status.get("battery", {})
    print(f"CAN: connected={can.get('connected')} канал={can.get('channel')} "
          f"линк={link.get('state')} rx={link.get('rx_hz')} Гц")
    command = drive.get("command", {})
    print(f"Управление: источник={drive.get('src')} "
          f"deadman={drive.get('deadman')} "
          f"цель {command.get('linear_x')} м/с, {command.get('angular_z')} рад/с")
    if battery.get("present"):
        percent = battery.get("percentage")
        percent_text = f"{percent * 100:.0f}%" if percent is not None else "?"
        print(f"Батарея: {battery.get('voltage')} В ({percent_text}), "
              f"ток {battery.get('input_current')} А, "
              f"мощность {battery.get('power_w')} Вт")
    for name, wheel in (status.get("wheels") or {}).items():
        print(f"  {name:12s} id={wheel.get('can_id'):>3} "
              f"{wheel.get('measured_mps', 0.0):+.2f} м/с "
              f"{wheel.get('erpm', 0.0):+7.0f} eRPM "
              f"FET {wheel.get('temp_fet')}°C "
              f"{'свежо' if wheel.get('fresh') else 'ПРОТУХЛО'}")


def show_gateway(status: dict) -> None:
    identity = status.get("identity", {})
    system = status.get("system", {})
    odom = status.get("odom")
    diagnostics = status.get("diagnostics", {})
    print(f"Ровер: {identity.get('id')} @ {identity.get('ip_addresses')}")
    cpu = system.get("cpu_percent")
    temperature = system.get("temperature_c")
    print(f"Система: CPU {cpu if cpu is not None else '?'}% "
          f"t={temperature if temperature is not None else '?'}°C")
    if odom:
        print(f"Одометрия: x={odom['x']:+.2f} y={odom['y']:+.2f} "
              f"yaw={odom['yaw']:+.2f} рад, v={odom['vx']:+.2f} м/с")
    level_names = {-1: "нет данных", 0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}
    level = diagnostics.get("highest_level", -1)
    print(f"Диагностика: {level_names.get(level, level)} "
          f"({len(diagnostics.get('items', []))} записей)")
    for item in diagnostics.get("items", []):
        if item.get("level", 0) != 0:
            print(f"  !! {item.get('name')}: {item.get('message')}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="10.42.0.1")
    parser.add_argument("--port", type=int, default=8767,
                        help="8767 — motord (по умолчанию), 8765 — шлюз ROS")
    parser.add_argument("--watch", action="store_true",
                        help="обновлять раз в секунду до Ctrl+C")
    args = parser.parse_args()
    base = f"http://{args.host}:{args.port}"

    try:
        while True:
            status = fetch(base, "/api/status")
            print("\033[2J\033[H" if args.watch else "", end="")
            if status.get("app") == "rover-motord" or "wheels" in status:
                show_motord(status)
            else:
                show_gateway(status)
            if not args.watch:
                break
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
