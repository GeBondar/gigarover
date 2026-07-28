#!/usr/bin/env python3
"""Управление GIGAROVER по HTTP API — с любого устройства в сети ровера.

Работает и с веб-мордой ROS-стека (:8765), и с аварийным телеопом
rover-motord (:8767) — формы API совместимы. Только стандартная
библиотека Python, никаких зависимостей.

ВАЖНО: у ровера дедмен — команда живёт 0.25 с (шлюз :8765) или 0.5 с
(motord :8767). Одиночный POST не поедет: команды надо слать потоком,
этот скрипт шлёт их 20 раз в секунду.

Примеры (телефон/ноутбук в Wi-Fi сети GIGAROVER):

    python3 http_drive.py forward  --speed 0.4 --duration 2.0
    python3 http_drive.py backward --speed 0.3 --duration 1.5
    python3 http_drive.py turn     --omega 1.0 --duration 1.6
    python3 http_drive.py square   --side 1.0 --speed 0.4
    python3 http_drive.py stop

По Ethernet-сети укажите адрес платы:

    python3 http_drive.py --host 192.168.2.72 forward
"""
import argparse
import json
import math
import sys
import time
import urllib.request

SEND_HZ = 20.0  # частота потока команд; дедмен ровера — 0.25/0.5 с


def post(base: str, path: str, payload: dict | None = None) -> dict:
    body = json.dumps(payload or {}).encode("utf-8")
    request = urllib.request.Request(
        base + path,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=2.0) as response:
        return json.loads(response.read().decode("utf-8"))


def drive_for(base: str, linear_x: float, angular_z: float,
              duration: float) -> None:
    """Поток команд linear_x/angular_z в течение duration секунд."""
    period = 1.0 / SEND_HZ
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        post(base, "/api/drive/command",
             {"linear_x": linear_x, "angular_z": angular_z})
        time.sleep(period)
    post(base, "/api/drive/stop")


def cmd_forward(base, args):
    drive_for(base, abs(args.speed), 0.0, args.duration)


def cmd_backward(base, args):
    drive_for(base, -abs(args.speed), 0.0, args.duration)


def cmd_turn(base, args):
    # omega > 0 — против часовой (влево), omega < 0 — по часовой
    drive_for(base, 0.0, args.omega, args.duration)


def cmd_square(base, args):
    """Квадрат по разомкнутому контуру: время = путь / скорость.

    Скид-стир проскальзывает на поворотах, поэтому углы будут неточными —
    это демонстрация, а не навигация. Точные фигуры — через одометрию
    (examples/ros2_cmd_vel.py + /odom) или Nav2.
    """
    side_time = args.side / args.speed
    quarter_turn_time = (math.pi / 2.0) / args.omega
    for _ in range(4):
        drive_for(base, args.speed, 0.0, side_time)
        time.sleep(0.3)
        drive_for(base, 0.0, args.omega, quarter_turn_time)
        time.sleep(0.3)


def cmd_stop(base, _args):
    result = post(base, "/api/stop")
    print(json.dumps(result, ensure_ascii=False))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="10.42.0.1",
                        help="адрес ровера (10.42.0.1 в сети GIGAROVER)")
    parser.add_argument("--port", type=int, default=8765,
                        help="8765 — веб-шлюз ROS, 8767 — аварийный motord")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("forward", help="ехать вперёд")
    p.add_argument("--speed", type=float, default=0.4, help="м/с (лимит 1.5)")
    p.add_argument("--duration", type=float, default=2.0, help="секунды")
    p.set_defaults(func=cmd_forward)

    p = sub.add_parser("backward", help="ехать назад")
    p.add_argument("--speed", type=float, default=0.3)
    p.add_argument("--duration", type=float, default=1.5)
    p.set_defaults(func=cmd_backward)

    p = sub.add_parser("turn", help="разворот на месте")
    p.add_argument("--omega", type=float, default=1.0,
                   help="рад/с, >0 — влево (лимит 3.0)")
    p.add_argument("--duration", type=float, default=1.6)
    p.set_defaults(func=cmd_turn)

    p = sub.add_parser("square", help="квадрат (разомкнутый контур)")
    p.add_argument("--side", type=float, default=1.0, help="сторона, м")
    p.add_argument("--speed", type=float, default=0.4)
    p.add_argument("--omega", type=float, default=1.0)
    p.set_defaults(func=cmd_square)

    p = sub.add_parser("stop", help="аварийный стоп (блок движения 0.75 с)")
    p.set_defaults(func=cmd_stop)

    args = parser.parse_args()
    base = f"http://{args.host}:{args.port}"
    try:
        args.func(base, args)
    except KeyboardInterrupt:
        post(base, "/api/drive/stop")
        print("\nОстановлено (Ctrl+C).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
