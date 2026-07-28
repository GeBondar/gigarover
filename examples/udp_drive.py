#!/usr/bin/env python3
"""Управление через UDP API демона rover-motord — без ROS и без HTTP.

Демон слушает ТОЛЬКО 127.0.0.1:8460, поэтому скрипт запускается на самом
ровере (по SSH). Это самый низкоуровневый программный доступ к ходовой:
JSON-датаграммы прямо в контур 50 Гц.

    python3 udp_drive.py state                      # поток состояния
    python3 udp_drive.py forward --speed 0.4 --duration 2
    python3 udp_drive.py turn --omega 1.0 --duration 1.6
    python3 udp_drive.py stop                       # мягкий стоп
    python3 udp_drive.py estop                      # аварийный стоп

Протокол (детали — tools/motord/README.md):
    {"v":1,"src":"ros","cmd":"drive","vx":0.5,"wz":0.0}   команда движения
    {"v":1,"src":"ros","cmd":"stop"}                      мягкий стоп
    {"v":1,"cmd":"estop"}                                 аварийный стоп
    {"v":1,"cmd":"sub"}          подписка на поток state (50 Гц, TTL 3 с)
    {"v":1,"cmd":"get_state"}    разовый state

Источник "ros" перехватывается телефоном (источник "web" приоритетнее) —
телеоп с :8767 всегда может отобрать управление у скрипта.
"""
import argparse
import json
import socket
import time

MOTORD = ("127.0.0.1", 8460)
SEND_HZ = 20.0


def make_socket() -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    return sock


def send(sock: socket.socket, payload: dict) -> None:
    sock.sendto(json.dumps(payload).encode("utf-8"), MOTORD)


def cmd_state(args) -> None:
    sock = make_socket()
    send(sock, {"v": 1, "cmd": "sub"})
    deadline = time.monotonic() + args.duration
    last_print = 0.0
    while time.monotonic() < deadline:
        try:
            raw, _ = sock.recvfrom(65535)
        except (socket.timeout, ConnectionResetError):
            send(sock, {"v": 1, "cmd": "sub"})  # продлить подписку
            continue
        state = json.loads(raw.decode("utf-8"))
        if state.get("type") != "state":
            continue
        now = time.monotonic()
        if now - last_print < args.period:
            continue  # состояние летит 50 раз/с — печатаем реже
        last_print = now
        link = state.get("link", {})
        drive = state.get("drive", {})
        enc = state.get("enc", {})
        battery = state.get("battery", {})
        print(f"link={link.get('state'):8s} src={str(drive.get('src')):5s} "
              f"enc_seq={enc.get('seq')} valid={enc.get('valid')} "
              f"V={battery.get('voltage')} "
              f"колёса={[round(v, 2) for v in enc.get('mps', [])]}")


def drive_for(vx: float, wz: float, duration: float) -> None:
    sock = make_socket()
    period = 1.0 / SEND_HZ
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        send(sock, {"v": 1, "src": "ros", "cmd": "drive", "vx": vx, "wz": wz})
        time.sleep(period)
    send(sock, {"v": 1, "src": "ros", "cmd": "stop"})


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("state", help="печатать поток состояния демона")
    p.add_argument("--duration", type=float, default=10.0)
    p.add_argument("--period", type=float, default=0.5,
                   help="период печати, с (сам поток — 50 Гц)")
    p.set_defaults(func=cmd_state)

    p = sub.add_parser("forward", help="ехать вперёд")
    p.add_argument("--speed", type=float, default=0.4)
    p.add_argument("--duration", type=float, default=2.0)
    p.set_defaults(func=lambda a: drive_for(abs(a.speed), 0.0, a.duration))

    p = sub.add_parser("turn", help="разворот на месте")
    p.add_argument("--omega", type=float, default=1.0)
    p.add_argument("--duration", type=float, default=1.6)
    p.set_defaults(func=lambda a: drive_for(0.0, a.omega, a.duration))

    p = sub.add_parser("stop", help="мягкий стоп")
    p.set_defaults(func=lambda a: send(make_socket(),
                                       {"v": 1, "src": "ros", "cmd": "stop"}))

    p = sub.add_parser("estop", help="аварийный стоп + блок 0.75 с")
    p.set_defaults(func=lambda a: send(make_socket(), {"v": 1, "cmd": "estop"}))

    args = parser.parse_args()
    try:
        args.func(args)
    except KeyboardInterrupt:
        send(make_socket(), {"v": 1, "src": "ros", "cmd": "stop"})
        print("\nОстановлено (Ctrl+C).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
