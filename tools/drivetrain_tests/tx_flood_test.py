#!/usr/bin/env python3
"""Решающий тест: ломает ли ПЕРЕДАЧА приём на этом хосте — без вращения моторов.

Фаза A: 5 с только приём (базовая линия).
Фаза B: 8 с приём + передача SET_RPM на НЕСУЩЕСТВУЮЩИЙ CAN ID 99
        тем же паттерном, что боевой драйвер (пачка 4 кадра каждые 20 мс).
Фаза C: 3 с только приём (восстановление).

Моторы не получают ни одной команды. Если в фазе B появляются провалы
приёма, которых нет в A/C — проблема в хосте (драйвер ch341 / USB),
а не в помехах от моторов.
"""
import glob
import struct
import sys
import time

import can

P_SET_RPM = 3
P_STATUS = 9
GHOST_ID = 99          # на шине таких нет: реальные 32, 34, 104, 114


def phase(bus, name, seconds, tx, paced=False):
    """tx=False: только приём. tx=True: burst 4 кадра/20 мс,
    paced=True: 1 кадр каждые 5 мс (та же средняя скорость 200 кадров/с)."""
    stamps = []
    sent = 0
    t0 = time.time()
    step = 0.005 if paced else 0.02
    per_tick = 1 if paced else 4
    next_tx = t0
    while True:
        now = time.time()
        if now - t0 >= seconds:
            break
        if tx and now >= next_tx:
            for _ in range(per_tick):
                bus.send(can.Message(
                    arbitration_id=(P_SET_RPM << 8) | GHOST_ID,
                    is_extended_id=True, data=struct.pack(">i", 0)))
                sent += 1
            next_tx += step
        m = bus.recv(timeout=0 if tx else 0.05)
        while m is not None:
            if m.is_extended_id and ((m.arbitration_id >> 8) & 0xFF) == P_STATUS:
                stamps.append(time.time())
            m = bus.recv(timeout=0)
        if tx:
            time.sleep(0.001)
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    n_bad = sum(1 for g in gaps if g > 0.15)
    mx = max(gaps) * 1000 if gaps else 0
    rate = len(stamps) / seconds
    print(f"  {name}: Status1 кадров={len(stamps)} ({rate:.0f}/с)  "
          f"передано={sent}  max_gap={mx:.0f} мс  провалов>150мс={n_bad}")
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
        got = False
        while time.time() - t0 < 2.0:
            m = bus.recv(timeout=0.1)
            if m is not None and m.is_extended_id:
                got = True
                break
        if got:
            print(f"[scan] шина на {port}")
            break
        bus.shutdown()
    else:
        print("Адаптер/шина не найдены — воткнут ли CH340 в Orange Pi?")
        return 1

    print("Фаза A — только приём (базовая линия):")
    a_bad, a_mx = phase(bus, "A", 5.0, tx=False)
    print("Фаза B — приём + BURST TX (4 кадра/20 мс, как в драйвере):")
    b_bad, b_mx = phase(bus, "B", 15.0, tx=True)
    print("Фаза C — только приём (пауза/восстановление):")
    c_bad, c_mx = phase(bus, "C", 3.0, tx=False)
    print("Фаза D — приём + PACED TX (1 кадр/5 мс, та же средняя скорость):")
    d_bad, d_mx = phase(bus, "D", 15.0, tx=True, paced=True)
    print("Фаза E — только приём (восстановление):")
    e_bad, e_mx = phase(bus, "E", 3.0, tx=False)
    bus.shutdown()

    print()
    print(f"Итого: baseline={a_bad}  burst={b_bad} (max {b_mx:.0f} мс)  "
          f"paced={d_bad} (max {d_mx:.0f} мс)")
    if b_bad > 0 and d_bad == 0:
        print("ВЕРДИКТ: burst-передача ломает приём, paced — нет. Лечится")
        print("разнесением кадров в драйвере.")
    elif b_bad > 0 and d_bad > 0:
        print("ВЕРДИКТ: хост теряет приём при любой передаче — pacing не")
        print("спасает; смотреть в сторону gs_usb-адаптера/драйвера ch341.")
    elif b_bad == 0 and d_bad == 0:
        print("ВЕРДИКТ: в этом прогоне TX приём не ломал — повторить или")
        print("искать фактор реального вращения (байты телеметрии).")
    else:
        print("ВЕРДИКТ: смотри цифры выше.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
