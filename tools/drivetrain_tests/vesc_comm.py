#!/usr/bin/env python3
"""COMM-протокол VESC поверх CAN — конфигурация без VESC Tool.

Умеет то, что нам нужно от VESC Tool, через штатный CAN-адаптер:
    fw                  версия прошивки/железа каждого VESC
    getapp <id>         hex-дамп app-конфига (отладка)
    status5 [--write]   включить CAN Status 1..5 @ 50 Гц на всех VESC
                        (без --write — только читает и показывает план)
    detect-foc --write  мастер мотора (FOC detection) как в VESC Tool,
                        КРУТИТ МОТОРЫ — колёса вывесить!

Механика: COMM-пакеты заворачиваются в CAN-кадры FILL_RX_BUFFER(5/6) +
PROCESS_RX_BUFFER(7) / PROCESS_SHORT_BUFFER(8) — ровно так VESC Tool
работает через CAN-forward. Наш «адрес» на шине — id 254.

App-конфиг пишется по принципу read-modify-write СЫРОГО буфера: читаем,
меняем 3-4 байта по известным смещениям, пишем назад. Смещения зависят от
мажора прошивки (5.x / 6.x) и ПРОВЕРЯЮТСЯ якорями (controller_id и
timeout_msec в начале буфера); якоря не сошлись — никакой записи.

Запуск на ровере (шиной владеет кто-то один):
    sudo systemctl stop rover-motord
    python3 vesc_comm.py fw
    python3 vesc_comm.py status5 --write
    sudo systemctl start rover-motord
"""
import argparse
import glob
import struct
import subprocess
import sys
import time

import can

MY_ID = 254                      # наш id на шине (VESC Tool использует 254)
VESC_IDS = [32, 34, 104, 114]    # FL FR RL RR

# CAN-пакеты (comm_can.c)
PKT_FILL_RX = 5
PKT_FILL_RX_LONG = 6
PKT_PROCESS_RX = 7
PKT_PROCESS_SHORT = 8
PKT_STATUS = 9
PKT_STATUS_5 = 27

# COMM-команды (datatypes.h)
COMM_FW_VERSION = 0
COMM_SET_APPCONF = 16
COMM_GET_APPCONF = 17
COMM_DETECT_APPLY_ALL_FOC = 58


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
        crc &= 0xFFFF
    return crc


def find_bus():
    for port in sorted(glob.glob('/dev/ttyUSB*')):
        try:
            b = can.Bus(interface='seeedstudio', channel=port,
                        bitrate=500000, timeout=0.1)
        except Exception as e:
            print(f'[scan] {port}: не открылся: {e}')
            continue
        deadline = time.time() + 3.0
        while time.time() < deadline:
            m = b.recv(timeout=0.2)
            if m is not None and m.is_extended_id:
                print(f'[scan] шина на {port}')
                return b
        b.shutdown()
        print(f'[scan] {port}: тишина')
    return None


def services_active():
    try:
        r = subprocess.run(
            ['systemctl', 'is-active', '--quiet',
             'rover-motord', 'rover-setup-web', 'rover-can-teleop'],
            check=False, timeout=5)
        return r.returncode == 0
    except Exception:
        return False


class VescComm:
    """COMM-запросы к одному VESC через CAN-буферный протокол."""

    def __init__(self, bus):
        self.bus = bus

    def _send_frame(self, ptype, cid, payload):
        self.bus.send(can.Message(arbitration_id=(ptype << 8) | cid,
                                  is_extended_id=True, data=payload))

    def send_command(self, target: int, payload: bytes) -> None:
        """COMM-пакет -> VESC target; ответ придёт на MY_ID."""
        if len(payload) <= 6:
            self._send_frame(PKT_PROCESS_SHORT, target,
                             bytes([MY_ID, 0]) + payload)
            return
        offset = 0
        while offset < len(payload) and offset < 255:
            chunk = payload[offset:offset + 7]
            self._send_frame(PKT_FILL_RX, target, bytes([offset]) + chunk)
            offset += len(chunk)
        while offset < len(payload):
            chunk = payload[offset:offset + 6]
            self._send_frame(PKT_FILL_RX_LONG, target,
                             bytes([offset >> 8, offset & 0xFF]) + chunk)
            offset += len(chunk)
        self._send_frame(
            PKT_PROCESS_RX, target,
            bytes([MY_ID, 0]) + struct.pack('>HH', len(payload), crc16(payload)))

    def recv_reply(self, timeout=2.0):
        """Собрать ответный COMM-пакет, адресованный MY_ID."""
        chunks = {}
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            m = self.bus.recv(timeout=0.05)
            if m is None or not m.is_extended_id:
                continue
            ptype = (m.arbitration_id >> 8) & 0xFF
            cid = m.arbitration_id & 0xFF
            if cid != MY_ID:
                continue
            d = bytes(m.data)
            if ptype == PKT_PROCESS_SHORT and len(d) >= 3:
                return d[2:]                       # sender, flag, payload...
            if ptype == PKT_FILL_RX and len(d) >= 2:
                chunks[d[0]] = d[1:]
            elif ptype == PKT_FILL_RX_LONG and len(d) >= 3:
                chunks[(d[0] << 8) | d[1]] = d[2:]
            elif ptype == PKT_PROCESS_RX and len(d) >= 6:
                length = (d[2] << 8) | d[3]
                want_crc = (d[4] << 8) | d[5]
                buf = bytearray(length)
                got = bytearray(length)          # маска заполнения
                for off, chunk in chunks.items():
                    for i, byte in enumerate(chunk):
                        if off + i < length:
                            buf[off + i] = byte
                            got[off + i] = 1
                if all(got) and crc16(bytes(buf)) == want_crc:
                    return bytes(buf)
                return None                       # дыры/битый crc
        return None

    def request(self, target: int, payload: bytes, timeout=2.0, retries=3):
        for attempt in range(retries):
            self.send_command(target, payload)
            reply = self.recv_reply(timeout)
            if reply is not None and reply[:1] == payload[:1]:
                return reply
        return None

    # ---- конкретные команды -------------------------------------------------
    def fw_version(self, target: int):
        reply = self.request(target, bytes([COMM_FW_VERSION]))
        if reply is None or len(reply) < 3:
            return None
        major, minor = reply[1], reply[2]
        hw = reply[3:].split(b'\x00', 1)[0].decode('ascii', 'replace')
        return major, minor, hw

    def get_appconf(self, target: int):
        reply = self.request(target, bytes([COMM_GET_APPCONF]), timeout=3.0)
        if reply is None or len(reply) < 20:
            return None
        return reply[1:]                          # конфиг с сигнатурой

    def set_appconf(self, target: int, conf: bytes) -> bool:
        reply = self.request(target, bytes([COMM_SET_APPCONF]) + conf,
                             timeout=5.0)
        return reply is not None and reply[0] == COMM_SET_APPCONF


# ---------------------------------------------------------------------------
# Правка app-конфига: смещения полей после [сигнатура u32][controller_id u8]
# [timeout_msec u32][timeout_brake_current f32] = байт 13.
#   fw 5.x: [send_can_status u8][send_can_status_rate_hz u16]
#   fw 6.x: [can_status_rate_1 u16][can_status_msgs_r1 u8]
#           [can_status_rate_2 u16][can_status_msgs_r2 u8]
# ---------------------------------------------------------------------------
def check_anchors(conf: bytes, target: int) -> bool:
    controller_id = conf[4]
    timeout_msec = struct.unpack('>I', conf[5:9])[0]
    ok = controller_id == target and 50 <= timeout_msec <= 60000
    print(f'    якоря: controller_id={controller_id} (ждём {target}), '
          f'timeout_msec={timeout_msec} -> {"ok" if ok else "НЕ СОШЛИСЬ"}')
    return ok


def plan_status5(conf: bytes, major: int):
    """(описание, патченый буфер) или (причина, None)."""
    buf = bytearray(conf)
    if major == 5:
        cur_mode, cur_rate = buf[13], struct.unpack('>H', bytes(buf[14:16]))[0]
        desc = (f'send_can_status {cur_mode}->5 (1..5), '
                f'rate {cur_rate}->50 Гц')
        buf[13] = 5                               # SEND_CAN_STATUS_1_2_3_4_5
        buf[14:16] = struct.pack('>H', 50)
    elif major == 6:
        cur_r1 = struct.unpack('>H', bytes(buf[13:15]))[0]
        cur_m1 = buf[15]
        desc = (f'rate_1 {cur_r1}->50 Гц, msgs_r1 mask '
                f'0b{cur_m1:08b}->0b00011111 (Status 1..5)')
        buf[13:15] = struct.pack('>H', 50)
        buf[15] = 0x1F
    else:
        return f'прошивка {major}.x не поддержана этим скриптом', None
    return desc, bytes(buf)


def measure_status_rates(bus, seconds=3.0):
    counts = {}
    t0 = time.time()
    while time.time() - t0 < seconds:
        m = bus.recv(timeout=0.05)
        if m is None or not m.is_extended_id:
            continue
        ptype = (m.arbitration_id >> 8) & 0xFF
        cid = m.arbitration_id & 0xFF
        if ptype in (PKT_STATUS, PKT_STATUS_5) and cid in VESC_IDS:
            counts[(cid, ptype)] = counts.get((cid, ptype), 0) + 1
    span = time.time() - t0
    for cid in VESC_IDS:
        s1 = counts.get((cid, PKT_STATUS), 0) / span
        s5 = counts.get((cid, PKT_STATUS_5), 0) / span
        print(f'    ID {cid:3d}: Status1 {s1:5.1f} Гц   Status5 {s5:5.1f} Гц')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('cmd', choices=['fw', 'getapp', 'status5', 'detect-foc'])
    parser.add_argument('id', nargs='?', type=int, help='CAN id (для getapp)')
    parser.add_argument('--write', action='store_true',
                        help='реально писать конфиг (иначе только план)')
    parser.add_argument('--ids', type=lambda s: [int(x) for x in s.split(',')],
                        default=VESC_IDS)
    parser.add_argument('--force', action='store_true')
    parser.add_argument('--max-power-loss', type=float, default=120.0,
                        help='detect-foc: допустимые потери, Вт (масштаб токов)')
    args = parser.parse_args()

    if services_active() and not args.force:
        print('ОТКАЗ: активна служба, владеющая CAN. '
              'Сначала: sudo systemctl stop rover-motord')
        return 1
    bus = find_bus()
    if bus is None:
        print('Шина не найдена')
        return 1
    vc = VescComm(bus)
    try:
        if args.cmd == 'fw':
            for cid in args.ids:
                info = vc.fw_version(cid)
                if info is None:
                    print(f'ID {cid:3d}: НЕТ ОТВЕТА на COMM_FW_VERSION')
                else:
                    print(f'ID {cid:3d}: fw {info[0]}.{info[1]:02d}  hw "{info[2]}"')
            return 0

        if args.cmd == 'getapp':
            if args.id is None:
                print('нужен id'); return 1
            conf = vc.get_appconf(args.id)
            if conf is None:
                print('нет ответа'); return 1
            print(f'app-конфиг {len(conf)} байт, '
                  f'сигнатура {struct.unpack(">I", conf[:4])[0]}')
            for off in range(0, min(len(conf), 64), 16):
                print(f'  {off:3d}: {conf[off:off+16].hex(" ")}')
            check_anchors(conf, args.id)
            return 0

        if args.cmd == 'status5':
            print('Частоты ДО:')
            measure_status_rates(bus)
            failures = 0
            for cid in args.ids:
                print(f'-- VESC id {cid}')
                info = vc.fw_version(cid)
                if info is None:
                    print('    нет ответа, пропуск'); failures += 1; continue
                major = info[0]
                print(f'    fw {info[0]}.{info[1]:02d} "{info[2]}"')
                conf = vc.get_appconf(cid)
                if conf is None:
                    print('    app-конфиг не прочитался, пропуск')
                    failures += 1
                    continue
                print(f'    конфиг {len(conf)} байт; '
                      f'head: {conf[:20].hex(" ")}')
                if not check_anchors(conf, cid):
                    failures += 1
                    continue
                desc, patched = plan_status5(conf, major)
                if patched is None:
                    print(f'    {desc}'); failures += 1; continue
                print(f'    план: {desc}')
                if not args.write:
                    continue
                if not vc.set_appconf(cid, patched):
                    print('    ЗАПИСЬ НЕ ПОДТВЕРЖДЕНА'); failures += 1
                    continue
                back = vc.get_appconf(cid)
                if back == patched:
                    print('    записано и перечитано ✓')
                else:
                    print('    ВНИМАНИЕ: перечитанный конфиг отличается')
                    failures += 1
            if args.write:
                print('Частоты ПОСЛЕ (2 с на применение):')
                time.sleep(2.0)
                measure_status_rates(bus)
            else:
                print('(dry-run: ничего не записано; повторить с --write)')
            return 1 if failures else 0

        if args.cmd == 'detect-foc':
            if not args.write:
                print('detect-foc КРУТИТ МОТОРЫ. Колёса вывесить и запустить '
                      'с --write.')
                return 1
            # Как мастер VESC Tool: детекция и применение на каждом отдельно.
            for cid in args.ids:
                print(f'-- VESC id {cid}: FOC detection '
                      f'(max_power_loss={args.max_power_loss} Вт)…')
                payload = bytes([COMM_DETECT_APPLY_ALL_FOC, 0]) + struct.pack(
                    '>iiiii',
                    int(args.max_power_loss * 1000.0),
                    int(2.0 * 1000.0),        # min_current_in (не исп. в fw)
                    int(30.0 * 1000.0),       # max_current_in (не исп. в fw)
                    int(700.0 * 1000.0),      # openloop_rpm
                    int(1400.0 * 1000.0))     # sl_erpm
                vc.send_command(cid, payload)
                reply = None
                deadline = time.monotonic() + 90.0   # детекция небыстрая
                while time.monotonic() < deadline:
                    reply = vc.recv_reply(timeout=5.0)
                    if reply and reply[0] == COMM_DETECT_APPLY_ALL_FOC:
                        break
                    reply = None
                if reply is None:
                    print('    НЕТ ОТВЕТА (детекция могла не завершиться!)')
                    continue
                res = struct.unpack('>h', reply[1:3])[0]
                verdict = {0: 'OK', -1: 'детекция не удалась',
                           -10: 'отменена', -50: 'CAN-детекция: нет ответа',
                           -51: 'CAN-детекция: ошибка'}.get(res, f'код {res}')
                print(f'    результат: {verdict}')
            return 0
    finally:
        bus.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
