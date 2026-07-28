#!/usr/bin/env python3
"""Конфигурация VESC по USB-серийнику (COMM-протокол, без VESC Tool).

Подключение: USB к ОДНОМУ контроллеру (Windows: COM18, Linux: /dev/ttyACM0);
остальные на шине достаются через COMM_FORWARD_CAN — как CAN-forward в
VESC Tool.

    python3 vesc_usb.py scan                    # кто на проводе и на шине
    python3 vesc_usb.py getapp [--can ID]       # hex app-конфига
    python3 vesc_usb.py status5 [--write]       # CAN Status 1..5 @ 50 Гц всем
    python3 vesc_usb.py detect-foc --write      # мастер мотора (КРУТИТ КОЛЁСА)

App-конфиг правится как read-modify-write сырого буфера: меняем только байты
режима/частоты CAN-статусов по смещениям для major-версии прошивки, якоря
(controller_id, timeout_msec) обязаны сойтись — иначе записи не будет.
"""
import argparse
import struct
import sys
import time

import serial

DEFAULT_PORT = 'COM18' if sys.platform == 'win32' else '/dev/ttyACM0'
VESC_IDS = [32, 34, 104, 114]                 # FL FR RL RR

COMM_FW_VERSION = 0
COMM_SET_MCCONF = 13
COMM_GET_MCCONF = 14
COMM_GET_MCCONF_DEFAULT = 15
COMM_SET_APPCONF = 16
COMM_GET_APPCONF = 17
COMM_REBOOT = 29
COMM_FORWARD_CAN = 34
COMM_DETECT_APPLY_ALL_FOC = 58


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
        head = bytes([3, len(payload) >> 8, len(payload) & 0xFF])
    return head + payload + struct.pack('>H', crc16(payload)) + b'\x03'


def read_packet(ser: serial.Serial, timeout: float):
    """Одна COMM-посылка из порта (стартовый байт 2/3, crc, стоп 3)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        start = ser.read(1)
        if not start:
            continue
        if start[0] == 2:
            ln = ser.read(1)
            if not ln:
                continue
            length = ln[0]
        elif start[0] == 3:
            ln = ser.read(2)
            if len(ln) < 2:
                continue
            length = (ln[0] << 8) | ln[1]
        else:
            continue                          # мусор между пакетами
        rest = ser.read(length + 3)
        if len(rest) < length + 3 or rest[-1] != 3:
            continue
        payload = rest[:length]
        want = (rest[length] << 8) | rest[length + 1]
        if crc16(payload) == want:
            return payload
    return None


class VescUsb:
    def __init__(self, port: str):
        self.ser = serial.Serial(port, 115200, timeout=0.05)

    def request(self, payload: bytes, can_id=None, timeout=2.0, retries=3):
        """COMM-запрос; can_id != None -> через COMM_FORWARD_CAN."""
        wire = payload if can_id is None else (
            bytes([COMM_FORWARD_CAN, can_id]) + payload)
        for _ in range(retries):
            self.ser.reset_input_buffer()
            self.ser.write(frame(wire))
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                reply = read_packet(self.ser, deadline - time.monotonic())
                if reply and reply[:1] == payload[:1]:
                    return reply
        return None

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    # ---- команды ------------------------------------------------------------
    def fw_version(self, can_id=None):
        reply = self.request(bytes([COMM_FW_VERSION]), can_id)
        if reply is None or len(reply) < 3:
            return None
        hw = reply[3:].split(b'\x00', 1)[0].decode('ascii', 'replace')
        return reply[1], reply[2], hw

    def get_appconf(self, can_id=None):
        reply = self.request(bytes([COMM_GET_APPCONF]), can_id, timeout=3.0)
        return None if reply is None or len(reply) < 20 else reply[1:]

    def set_appconf(self, conf: bytes, can_id=None) -> bool:
        reply = self.request(bytes([COMM_SET_APPCONF]) + conf, can_id,
                             timeout=6.0)
        return reply is not None and reply[0] == COMM_SET_APPCONF


# ---- правка app-конфига (смещения после сигнатуры(4)+id(1)+timeout(4)+
# timeout_brake(4) = байт 13; см. confgenerator прошивки) --------------------
def check_anchors(conf: bytes, expect_id=None) -> bool:
    controller_id = conf[4]
    timeout_msec = struct.unpack('>I', conf[5:9])[0]
    ok = 50 <= timeout_msec <= 60000 and (
        expect_id is None or controller_id == expect_id)
    print(f'    якоря: controller_id={controller_id}'
          + (f' (ждём {expect_id})' if expect_id is not None else '')
          + f', timeout_msec={timeout_msec} -> {"ok" if ok else "НЕ СОШЛИСЬ"}')
    return ok


def plan_status5(conf: bytes, major: int):
    buf = bytearray(conf)
    if major == 5:
        cur_mode, cur_rate = buf[13], struct.unpack('>H', bytes(buf[14:16]))[0]
        desc = f'send_can_status {cur_mode}->5 (1..5), rate {cur_rate}->50 Гц'
        buf[13] = 5
        buf[14:16] = struct.pack('>H', 50)
    elif major == 6:
        # fw 6.02: [rate_1 u16 @13][rate_2 u16 @15][msgs_r1 u8 @17]
        #          [msgs_r2 u8 @18][can_baud u8 @19] — сверено с дампом
        #          (rate_1=50, msgs_r1=0x0F, baud=2=500k на этих VESC).
        cur_r1 = struct.unpack('>H', bytes(buf[13:15]))[0]
        cur_m1, can_baud = buf[17], buf[19]
        if can_baud != 2:                     # CAN_BAUD_500K — контроль раскладки
            return (f'байт can_baud@19 = {can_baud}, ждали 2 (500k) — '
                    'раскладка не совпала, не пишу', None)
        desc = (f'rate_1 {cur_r1}->50 Гц, msgs_r1 '
                f'0b{cur_m1:08b}->0b{(cur_m1 | 0x1F):08b} (Status 1..5)')
        buf[13:15] = struct.pack('>H', 50)
        buf[17] = cur_m1 | 0x1F
    else:
        return f'прошивка {major}.x не поддержана', None
    return desc, bytes(buf)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('cmd', choices=['scan', 'getapp', 'status5',
                                        'detect-foc', 'reboot', 'mcdiff',
                                        'fix-inmin', 'fix-openloop',
                                        'fix-absmax'])
    parser.add_argument('--port', default=DEFAULT_PORT)
    parser.add_argument('--can', type=int, default=None,
                        help='CAN id цели (для getapp); без него — локальный')
    parser.add_argument('--write', action='store_true')
    parser.add_argument('--max-power-loss', type=float, default=120.0)
    args = parser.parse_args()

    try:
        vc = VescUsb(args.port)
    except Exception as exc:
        print(f'{args.port} не открылся: {exc}\n'
              f'(закройте VESC Tool, если он держит порт)')
        return 1
    try:
        # локальный контроллер: кто это?
        local_fw = vc.fw_version()
        if local_fw is None:
            print(f'На {args.port} никто не ответил на COMM_FW_VERSION — '
                  'это точно VESC? (закройте VESC Tool)')
            return 1
        local_conf = vc.get_appconf()
        local_id = local_conf[4] if local_conf else None
        print(f'USB {args.port}: fw {local_fw[0]}.{local_fw[1]:02d} '
              f'"{local_fw[2]}", CAN id {local_id}')

        # цели: локальный + остальные через FORWARD_CAN
        targets = []                       # (метка, can_id | None)
        for cid in VESC_IDS:
            targets.append((cid, None if cid == local_id else cid))
        if local_id not in VESC_IDS:
            print(f'ВНИМАНИЕ: локальный id {local_id} не из списка '
                  f'{VESC_IDS} — конфигурю и его тоже')
            targets.insert(0, (local_id, None))

        if args.cmd == 'scan':
            for label, fwd in targets:
                info = vc.fw_version(fwd)
                via = 'USB' if fwd is None else 'CAN'
                if info is None:
                    print(f'  id {label:3d} [{via}]: НЕТ ОТВЕТА')
                else:
                    print(f'  id {label:3d} [{via}]: fw {info[0]}.{info[1]:02d} '
                          f'"{info[2]}"')
            return 0

        if args.cmd == 'getapp':
            fwd = None if (args.can is None or args.can == local_id) else args.can
            conf = vc.get_appconf(fwd)
            if conf is None:
                print('нет ответа')
                return 1
            print(f'app-конфиг {len(conf)} байт, '
                  f'сигнатура {struct.unpack(">I", conf[:4])[0]}')
            for off in range(0, min(len(conf), 64), 16):
                print(f'  {off:3d}: {conf[off:off+16].hex(" ")}')
            check_anchors(conf, args.can)
            return 0

        if args.cmd == 'status5':
            failures = 0
            for label, fwd in targets:
                via = 'USB' if fwd is None else 'CAN'
                print(f'-- VESC id {label} [{via}]')
                info = vc.fw_version(fwd)
                if info is None:
                    print('    нет ответа, пропуск'); failures += 1; continue
                print(f'    fw {info[0]}.{info[1]:02d} "{info[2]}"')
                conf = vc.get_appconf(fwd)
                if conf is None:
                    print('    конфиг не прочитался'); failures += 1; continue
                print(f'    {len(conf)} байт; head: {conf[:20].hex(" ")}')
                if not check_anchors(conf, label):
                    failures += 1
                    continue
                desc, patched = plan_status5(conf, info[0])
                if patched is None:
                    print(f'    {desc}'); failures += 1; continue
                print(f'    план: {desc}')
                if not args.write:
                    continue
                if not vc.set_appconf(patched, fwd):
                    print('    ЗАПИСЬ НЕ ПОДТВЕРЖДЕНА'); failures += 1
                    continue
                back = vc.get_appconf(fwd)
                print('    записано и перечитано ✓' if back == patched
                      else '    ВНИМАНИЕ: перечитанное отличается')
            if not args.write:
                print('(dry-run: ничего не записано; повторить с --write)')
            return 1 if failures else 0

        if args.cmd == 'mcdiff':
            # mcconf vs заводской дефолт той же прошивки: одинаковый
            # сериализатор -> дифф по байтам выравнен. Показывает, что
            # реально записано (детекцией и прежними настройками).
            def f32auto(b):
                v = struct.unpack('>I', b)[0]
                e = (v >> 23) & 0xFF
                sig_i = v & 0x7FFFFF
                if e == 0 and sig_i == 0:
                    return 0.0
                sig = sig_i / 8388608.0 / 2.0 + 0.5
                res = __import__('math').ldexp(sig, e - 126)
                return -res if v & 0x80000000 else res

            fwd = None if (args.can is None or args.can == local_id) else args.can
            cur = vc.request(bytes([COMM_GET_MCCONF]), fwd, timeout=3.0)
            dflt = vc.request(bytes([COMM_GET_MCCONF_DEFAULT]), fwd, timeout=3.0)
            if not cur or not dflt:
                print('mcconf не прочитался')
                return 1
            cur, dflt = cur[1:], dflt[1:]
            print(f'mcconf {len(cur)} байт, дефолт {len(dflt)} байт')
            i, n = 0, min(len(cur), len(dflt))
            while i < n:
                if cur[i] == dflt[i]:
                    i += 1
                    continue
                j = i
                while j < n and cur[j] != dflt[j]:
                    j += 1
                lo = max(0, i - 3)
                hi = min(n, j + 3)
                print(f'@{lo:4d}: dflt {dflt[lo:hi].hex(" ")}')
                print(f'       cur  {cur[lo:hi].hex(" ")}')
                for off in range(max(0, i - 3), min(n - 4, j + 1)):
                    a, b = f32auto(cur[off:off + 4]), f32auto(dflt[off:off + 4])
                    if a != b and (1e-9 < abs(a) < 1e9 or a == 0.0) \
                            and (1e-9 < abs(b) < 1e9 or b == 0.0):
                        print(f'       f32auto@{off}: {b:.6g} -> {a:.6g}')
                i = j
            return 0

        if args.cmd == 'fix-inmin':
            # Лечение после detect-foc с кривым min_current_in=+2:
            # l_in_current_min (f32auto @20 в mcconf 6.02) -> -20 А.
            # Якорь: l_in_current_max @16 должен быть +30 (то, что писала
            # детекция); кодировки сверены с дампом mcdiff.
            BAD = bytes.fromhex('40000000')      # +2.0 (float32_auto)
            GOOD = bytes.fromhex('c1a00000')     # -20.0
            ANCHOR30 = bytes.fromhex('41f00000') # +30.0 @16
            for label, fwd in targets:
                print(f'-- id {label}')
                conf = vc.request(bytes([COMM_GET_MCCONF]), fwd, timeout=3.0)
                if not conf:
                    print('    mcconf не прочитался'); continue
                conf = bytearray(conf[1:])
                cur = bytes(conf[20:24])
                if bytes(conf[16:20]) != ANCHOR30:
                    print(f'    якорь @16 {bytes(conf[16:20]).hex()} != '
                          f'{ANCHOR30.hex()} (30 А) — пропускаю')
                    continue
                if cur == GOOD:
                    print('    уже -20 А ✓'); continue
                if cur != BAD:
                    print(f'    @20 = {cur.hex()} (не +2 А) — пропускаю')
                    continue
                conf[20:24] = GOOD
                if not args.write:
                    print('    план: l_in_current_min +2 -> -20 А (dry-run)')
                    continue
                ack = vc.request(bytes([COMM_SET_MCCONF]) + bytes(conf), fwd,
                                 timeout=6.0)
                if not ack or ack[0] != COMM_SET_MCCONF:
                    print('    ЗАПИСЬ НЕ ПОДТВЕРЖДЕНА'); continue
                back = vc.request(bytes([COMM_GET_MCCONF]), fwd, timeout=3.0)
                ok = back and bytes(back[1:][20:24]) == GOOD
                print('    записано ✓' if ok else '    ПЕРЕЧИТАЛОСЬ НЕ ТО')
            return 0

        if args.cmd == 'fix-openloop':
            # foc_openloop_rpm 700 -> 1400 (f32auto @201 в mcconf 6.02):
            # старт всегда через openloop до скорости уверенного захвата
            # наблюдателем (лечит зеркальный захват FR на малых оборотах).
            OLD = bytes.fromhex('442f0000')      # 700
            NEW = bytes.fromhex('44af0000')      # 1400
            for label, fwd in targets:
                print(f'-- id {label}')
                conf = vc.request(bytes([COMM_GET_MCCONF]), fwd, timeout=3.0)
                if not conf:
                    print('    mcconf не прочитался'); continue
                conf = bytearray(conf[1:])
                cur = bytes(conf[201:205])
                if cur == NEW:
                    print('    уже 1400 ✓'); continue
                if cur != OLD:
                    print(f'    @201 = {cur.hex()} (не 700) — пропускаю')
                    continue
                conf[201:205] = NEW
                if not args.write:
                    print('    план: foc_openloop_rpm 700 -> 1400 (dry-run)')
                    continue
                ack = vc.request(bytes([COMM_SET_MCCONF]) + bytes(conf), fwd,
                                 timeout=6.0)
                if not ack or ack[0] != COMM_SET_MCCONF:
                    print('    ЗАПИСЬ НЕ ПОДТВЕРЖДЕНА'); continue
                back = vc.request(bytes([COMM_GET_MCCONF]), fwd, timeout=3.0)
                ok = back and bytes(back[1:][201:205]) == NEW
                print('    записано ✓' if ok else '    ПЕРЕЧИТАЛОСЬ НЕ ТО')
            return 0

        if args.cmd == 'fix-absmax':
            # Детекция опустила l_abs_current_max до ~98 А, а мгновенные пики
            # тока на openloop-старте бьют 98-100 А -> ABS_OVER_CURRENT и
            # «долбёжка». Возвращаем заводские 150 А (f32auto @24).
            import math as _m

            def _f32auto(b):
                v = struct.unpack('>I', b)[0]
                e, sig_i = (v >> 23) & 0xFF, v & 0x7FFFFF
                if e == 0 and sig_i == 0:
                    return 0.0
                res = _m.ldexp(sig_i / 8388608.0 / 2.0 + 0.5, e - 126)
                return -res if v & 0x80000000 else res

            NEW = bytes.fromhex('43160000')      # 150.0 (заводское)
            for label, fwd in targets:
                print(f'-- id {label}')
                conf = vc.request(bytes([COMM_GET_MCCONF]), fwd, timeout=3.0)
                if not conf:
                    print('    mcconf не прочитался'); continue
                conf = bytearray(conf[1:])
                cur = bytes(conf[24:28])
                if cur == NEW:
                    print('    уже 150 ✓'); continue
                val = _f32auto(cur)
                if not 90.0 <= val <= 110.0:     # ждём ~98 от детекции
                    print(f'    @24 = {val:.4g} А (вне 90..110) — пропускаю')
                    continue
                conf[24:28] = NEW
                if not args.write:
                    print('    план: l_abs_current_max 98 -> 150 А (dry-run)')
                    continue
                ack = vc.request(bytes([COMM_SET_MCCONF]) + bytes(conf), fwd,
                                 timeout=6.0)
                if not ack or ack[0] != COMM_SET_MCCONF:
                    print('    ЗАПИСЬ НЕ ПОДТВЕРЖДЕНА'); continue
                back = vc.request(bytes([COMM_GET_MCCONF]), fwd, timeout=3.0)
                ok = back and bytes(back[1:][24:28]) == NEW
                print('    записано ✓' if ok else '    ПЕРЕЧИТАЛОСЬ НЕ ТО')
            return 0

        if args.cmd == 'reboot':
            # Сначала удалённые (после ребута локального форвард пропадёт).
            for label, fwd in sorted(targets, key=lambda t: t[1] is None):
                print(f'-- id {label}: reboot')
                wire = bytes([COMM_REBOOT]) if fwd is None else (
                    bytes([COMM_FORWARD_CAN, fwd, COMM_REBOOT]))
                vc.ser.write(frame(wire))
                time.sleep(0.3)
            print('ребут отправлен всем; секунд через 5 проверяйте scan')
            return 0

        if args.cmd == 'detect-foc':
            if not args.write:
                print('detect-foc КРУТИТ МОТОРЫ; колёса вывесить, затем --write')
                return 1
            for label, fwd in targets:
                print(f'-- id {label}: FOC detection '
                      f'(max_power_loss={args.max_power_loss} Вт)… ~30 c')
                # min_current_in ОБЯЗАН быть отрицательным (предел регена):
                # +2 А здесь однажды дало принудительный момент и разгон
                # моторов до упора (см. fix-inmin).
                payload = bytes([COMM_DETECT_APPLY_ALL_FOC, 0]) + struct.pack(
                    '>iiiii',
                    int(args.max_power_loss * 1000.0),
                    int(-20.0 * 1000.0), int(30.0 * 1000.0),
                    int(700.0 * 1000.0), int(1400.0 * 1000.0))
                wire = payload if fwd is None else (
                    bytes([COMM_FORWARD_CAN, fwd]) + payload)
                vc.ser.reset_input_buffer()
                vc.ser.write(frame(wire))
                reply = None
                deadline = time.monotonic() + 120.0
                while time.monotonic() < deadline:
                    p = read_packet(vc.ser, 5.0)
                    if p and p[0] == COMM_DETECT_APPLY_ALL_FOC:
                        reply = p
                        break
                if reply is None:
                    print('    НЕТ ОТВЕТА — проверить контроллер!')
                    continue
                res = struct.unpack('>h', reply[1:3])[0]
                verdict = {0: 'OK ✓', -1: 'детекция не удалась',
                           -10: 'отменена'}.get(res, f'код {res}')
                print(f'    результат: {verdict}')
            return 0
    finally:
        vc.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
