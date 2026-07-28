#!/usr/bin/env python3
"""Управление 4 моторами GIGAROVER через один VESC по USB (UART-протокол + CAN-forward).

Локальный VESC получает команды напрямую, остальные — через COMM_FORWARD_CAN.
Использование:
    python vesc_usb.py probe                # версия прошивки + скан CAN-шины
    python vesc_usb.py spin [eRPM] [сек]    # крутнуть все 4 мотора (по умолчанию 1500 eRPM, 3 c)
"""
import struct
import sys
import time

import serial

PORT = "COM18"
BAUD = 115200

COMM_FW_VERSION = 0
COMM_GET_VALUES = 4
COMM_SET_RPM = 8
COMM_SET_CURRENT = 6
COMM_FORWARD_CAN = 34
COMM_PING_CAN = 62

CAN_IDS = [32, 34, 104, 114]  # FL(inv), FR, RL(inv), RR


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def frame(payload: bytes) -> bytes:
    if len(payload) <= 255:
        head = bytes([2, len(payload)])
    else:
        head = bytes([3]) + struct.pack(">H", len(payload))
    return head + payload + struct.pack(">H", crc16(payload)) + b"\x03"


def read_packet(ser: serial.Serial, timeout: float = 1.0):
    ser.timeout = timeout
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b[0] == 2:
            ln = ser.read(1)
            if not ln:
                continue
            n = ln[0]
            rest = ser.read(n + 3)
            if len(rest) == n + 3 and rest[-1] == 3:
                payload = rest[:n]
                if crc16(payload) == struct.unpack(">H", rest[n:n + 2])[0]:
                    return payload
        elif b[0] == 3:
            ln = ser.read(2)
            if len(ln) < 2:
                continue
            n = struct.unpack(">H", ln)[0]
            rest = ser.read(n + 3)
            if len(rest) == n + 3 and rest[-1] == 3:
                payload = rest[:n]
                if crc16(payload) == struct.unpack(">H", rest[n:n + 2])[0]:
                    return payload
    return None


def set_rpm_local(ser, rpm: int):
    ser.write(frame(bytes([COMM_SET_RPM]) + struct.pack(">i", int(rpm))))


def set_rpm_can(ser, can_id: int, rpm: int):
    ser.write(frame(bytes([COMM_FORWARD_CAN, can_id, COMM_SET_RPM]) + struct.pack(">i", int(rpm))))


def release_all(ser, remote_ids):
    zero = struct.pack(">i", 0)
    ser.write(frame(bytes([COMM_SET_CURRENT]) + zero))
    for cid in remote_ids:
        ser.write(frame(bytes([COMM_FORWARD_CAN, cid, COMM_SET_CURRENT]) + zero))


def probe(ser):
    ser.write(frame(bytes([COMM_FW_VERSION])))
    p = read_packet(ser)
    if p and p[0] == COMM_FW_VERSION:
        name = p[3:].split(b"\x00")[0].decode(errors="replace")
        print(f"Локальный VESC: FW {p[1]}.{p[2]}  hw={name}")
    else:
        print("Нет ответа на FW_VERSION — связи с VESC нет")
        return None
    ser.write(frame(bytes([COMM_PING_CAN])))
    p = read_packet(ser, timeout=12.0)  # ping перебирает все 253 ID, это долго
    if p and p[0] == COMM_PING_CAN:
        ids = sorted(p[1:])
        print(f"На CAN-шине видны ID: {ids}")
        return ids
    print("PING_CAN без ответа")
    return None


def spin(ser, erpm: int, dur: float):
    remote = probe(ser)
    if remote is None:
        return 1
    if len(remote) < 3:
        print(f"ВНИМАНИЕ: на шине видно {len(remote)} из 3 удалённых VESC — кручу тех, кто есть")
    print(f"Кручу локальный + CAN {remote} на {erpm} eRPM, {dur} c ...")
    t_end = time.time() + dur
    try:
        while time.time() < t_end:
            set_rpm_local(ser, erpm)
            for cid in remote:
                set_rpm_can(ser, cid, erpm)
            time.sleep(0.02)  # 50 Гц
    finally:
        release_all(ser, remote)
        print("Отпустил моторы (ток 0).")
    return 0


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "probe"
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        if cmd == "probe":
            return 0 if probe(ser) is not None else 1
        if cmd == "spin":
            erpm = int(sys.argv[2]) if len(sys.argv) > 2 else 1500
            dur = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
            return spin(ser, erpm, dur)
        print(__doc__)
        return 2


if __name__ == "__main__":
    sys.exit(main())
