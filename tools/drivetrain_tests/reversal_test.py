#!/usr/bin/env python3
"""Ловим хаотичные развороты: постоянная команда +1500 eRPM всем моторам,
логируем отрицательные выбросы eRPM (это развороты), кадры с несуществующих
CAN ID (порча приёма) и провалы линка — и смотрим, совпадают ли развороты
по времени с провалами.
"""
import glob
import struct
import sys
import time

import can

P_SET_CURRENT, P_SET_RPM = 1, 3
P_STATUS = 9
IDS = [32, 34, 104, 114]
TARGET = 1500
RUN_S = 12.0
NEG_THRESH = -300      # eRPM ниже этого при команде +1500 = разворот


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

    known = set(IDS)
    ghost = {}                    # ghost_id -> count
    neg_events = {cid: [] for cid in IDS}   # (t, erpm)
    last_rx = {cid: None for cid in IDS}
    gaps = []                     # (t_start, dur) провалы по всем кадрам
    last_any = None
    print(f"[run] +{TARGET} eRPM всем, {RUN_S:.0f} с...")
    t0 = time.time()
    next_tx = t0
    try:
        while time.time() - t0 < RUN_S:
            now = time.time()
            if now >= next_tx:
                for cid in IDS:
                    bus.send(can.Message(
                        arbitration_id=(P_SET_RPM << 8) | cid,
                        is_extended_id=True,
                        data=struct.pack(">i", TARGET)))
                next_tx += 0.02
            m = bus.recv(timeout=0)
            while m is not None:
                t = time.time() - t0
                if m.is_extended_id:
                    ptype = (m.arbitration_id >> 8) & 0xFF
                    cid = m.arbitration_id & 0xFF
                    if cid not in known:
                        ghost[cid] = ghost.get(cid, 0) + 1
                    elif ptype == P_STATUS and len(m.data) >= 4:
                        if last_any is not None and t - last_any > 0.15:
                            gaps.append((last_any, t - last_any))
                        last_any = t
                        erpm = struct.unpack(">i", m.data[0:4])[0]
                        last_rx[cid] = t
                        if erpm < NEG_THRESH:
                            neg_events[cid].append((t, erpm))
                m = bus.recv(timeout=0)
            time.sleep(0.001)
    finally:
        for _ in range(5):
            for cid in IDS:
                try:
                    bus.send(can.Message(
                        arbitration_id=(P_SET_CURRENT << 8) | cid,
                        is_extended_id=True, data=struct.pack(">i", 0)))
                except Exception:
                    pass
            time.sleep(0.02)
    bus.shutdown()

    print("\n===== РАЗБОР =====")
    print(f"Провалов линка >150 мс: {len(gaps)}" +
          (f", max {max(d for _, d in gaps)*1000:.0f} мс" if gaps else ""))
    print(f"Кадры с НЕСУЩЕСТВУЮЩИХ ID (порча приёма): "
          f"{sum(ghost.values())} шт, ids={sorted(ghost)[:12]}")
    total_rev = 0
    for cid in IDS:
        ev = neg_events[cid]
        total_rev += len(ev)
        if ev:
            worst = min(e for _, e in ev)
            near_gap = sum(1 for t, _ in ev
                           if any(gs - 0.1 <= t <= gs + dur + 1.2
                                  for gs, dur in gaps))
            print(f"  ID {cid}: разворотов (eRPM<{NEG_THRESH}): {len(ev)}, "
                  f"худший {worst}, рядом с провалом: {near_gap}/{len(ev)}")
        else:
            print(f"  ID {cid}: разворотов нет")
    print()
    if total_rev and gaps:
        print("Если развороты кучкуются возле провалов — это порченые")
        print("команды + замирание линка: лечится чистым питанием Pi или")
        print("gs_usb-адаптером; паллиатив — Timeout 200-300 мс в VESC Tool.")
    elif not total_rev:
        print("Разворотов в этом прогоне не поймано — повторить/дольше.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
