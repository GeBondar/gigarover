#!/usr/bin/env python3
"""Тест выбега: различает шум-под-током и проблемы приёма при вращении.

Фазы:
  A  3 с   только приём, моторы стоят (базовая линия)
  B  3 с   TX SET_RPM 2500 всем моторам + приём (ожидаемо худший случай)
  C  ~6 с  ПОЛНАЯ тишина передатчика, моторы свободно докручиваются
           (ток ~0, обороты ненулевые) — только приём
  D  3 с   только приём, моторы уже остановились

Если C чистая, а B грязная -> проблема появляется под током моторов
(шум по питанию/земле в сторону Pi). Если C тоже грязная, пока обороты
высоки -> дело в байтах телеметрии/парсере.
"""
import glob
import struct
import sys
import time

import can

P_SET_CURRENT, P_SET_RPM = 1, 3
P_STATUS = 9
IDS = [32, 34, 104, 114]


def rx_phase(bus, name, seconds):
    stamps, erpms = [], []
    t0 = time.time()
    while time.time() - t0 < seconds:
        m = bus.recv(timeout=0.05)
        while m is not None:
            if m.is_extended_id and ((m.arbitration_id >> 8) & 0xFF) == P_STATUS:
                stamps.append(time.time())
                try:
                    erpms.append(abs(struct.unpack(">i", m.data[0:4])[0]))
                except Exception:
                    pass
            m = bus.recv(timeout=0)
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    n_bad = sum(1 for g in gaps if g > 0.15)
    mx = max(gaps) * 1000 if gaps else 0
    top = max(erpms) if erpms else 0
    print(f"  {name}: кадров={len(stamps)} ({len(stamps)/seconds:.0f}/с)  "
          f"max_gap={mx:.0f} мс  провалов>150мс={n_bad}  max|eRPM|={top}")
    return n_bad, mx


def main():
    for port in sorted(glob.glob("/dev/ttyUSB*")):
        try:
            bus = can.Bus(interface="seeedstudio", channel=port,
                          bitrate=500000, timeout=0.1)
        except Exception as e:
            print(f"[scan] {port}: {e}")
            continue
        t0 = time.time()
        ok = False
        while time.time() - t0 < 2.0:
            m = bus.recv(timeout=0.1)
            if m is not None and m.is_extended_id:
                ok = True
                break
        if ok:
            print(f"[scan] шина на {port}")
            break
        bus.shutdown()
    else:
        print("Шина не найдена")
        return 1

    try:
        print("Фаза A — приём, моторы стоят:")
        rx_phase(bus, "A", 3.0)

        print("Фаза B — TX 2500 eRPM всем + приём (раскрутка):")
        stamps = []
        t0 = time.time()
        next_tx = t0
        while time.time() - t0 < 3.0:
            now = time.time()
            if now >= next_tx:
                for cid in IDS:
                    bus.send(can.Message(
                        arbitration_id=(P_SET_RPM << 8) | cid,
                        is_extended_id=True,
                        data=struct.pack(">i", 2500)))
                next_tx += 0.02
            m = bus.recv(timeout=0)
            while m is not None:
                if m.is_extended_id and ((m.arbitration_id >> 8) & 0xFF) == P_STATUS:
                    stamps.append(time.time())
                m = bus.recv(timeout=0)
            time.sleep(0.001)
        gaps = [b - a for a, b in zip(stamps, stamps[1:])]
        print(f"  B: кадров={len(stamps)}  "
              f"max_gap={(max(gaps)*1000 if gaps else 0):.0f} мс  "
              f"провалов>150мс={sum(1 for g in gaps if g > 0.15)}")
    finally:
        # один кадр "ток 0" каждому — и полная тишина: мотор свободно катится
        for cid in IDS:
            try:
                bus.send(can.Message(
                    arbitration_id=(P_SET_CURRENT << 8) | cid,
                    is_extended_id=True, data=struct.pack(">i", 0)))
            except Exception:
                pass

    print("Фаза C — ВЫБЕГ: передатчик молчит, колёса докручиваются:")
    rx_phase(bus, "C1 (0-2с)", 2.0)
    rx_phase(bus, "C2 (2-4с)", 2.0)
    rx_phase(bus, "C3 (4-6с)", 2.0)

    print("Фаза D — приём, моторы стоят:")
    rx_phase(bus, "D", 3.0)
    bus.shutdown()
    print("\nСравни: если C1/C2 чистые при ненулевых eRPM, а B грязная — "
          "виноват ток моторов (питание/земля Pi). Если C1 грязная — "
          "дело в самих байтах телеметрии при вращении.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
