#!/usr/bin/env python3
"""Тест здоровья VESC/CAN гигаровера: профиль скоростей + сбор телеметрии.

Крутит все найденные моторы по ступеням eRPM (низкие обороты, колёса
вывешены), пишет каждый принятый кадр телеметрии в CSV и в конце печатает
сводку: частоты статусов, напряжение, токи, температуры, слежение за целью,
приращение тахометров — с вердиктом по каждому пункту.

Безопасность: ток 0 всем при любом выходе; цель ограничена 2000 eRPM.
"""
import csv
import glob
import signal
import struct
import sys
import time

import can

P_SET_CURRENT, P_SET_RPM = 1, 3
P_STATUS, P_STATUS_2, P_STATUS_4, P_STATUS_5 = 9, 14, 16, 27

STEPS = [1500, 2500, 1200, 800, 1500]
STEP_S = 3.0
CSV_PATH = "/home/ubuntu/can_telemetry.csv"


def send(bus, ptype, cid, payload):
    bus.send(can.Message(arbitration_id=(ptype << 8) | cid,
                         is_extended_id=True, data=payload))


def send_rpm(bus, cid, erpm):
    send(bus, P_SET_RPM, cid, struct.pack(">i", int(erpm)))


def send_current(bus, cid, amps):
    send(bus, P_SET_CURRENT, cid, struct.pack(">i", int(amps * 1000.0)))


class Collector:
    """Копит последние значения по каждому VESC и пишет каждый кадр в лог."""

    def __init__(self):
        self.last = {}          # cid -> dict полей
        self.rows = []          # (t, cid, ptype, поля...)
        self.counts = {}        # (cid, ptype) -> число кадров
        self.gaps = {}          # cid -> [t_last, max_gap, n_gaps_over_150ms]
        self.t0 = time.time()

    def feed(self, msg, target):
        if not msg.is_extended_id:
            return None
        ptype = (msg.arbitration_id >> 8) & 0xFF
        cid = msg.arbitration_id & 0xFF
        d = msg.data
        rec = self.last.setdefault(cid, {})
        upd = {}
        try:
            if ptype == P_STATUS and len(d) >= 8:
                upd = {"erpm": struct.unpack(">i", d[0:4])[0],
                       "current": struct.unpack(">h", d[4:6])[0] / 10.0,
                       "duty": struct.unpack(">h", d[6:8])[0] / 1000.0}
            elif ptype == P_STATUS_2 and len(d) >= 4:
                upd = {"amp_hours": struct.unpack(">i", d[0:4])[0] / 1e4}
            elif ptype == P_STATUS_4 and len(d) >= 6:
                upd = {"temp_fet": struct.unpack(">h", d[0:2])[0] / 10.0,
                       "temp_motor": struct.unpack(">h", d[2:4])[0] / 10.0,
                       "current_in": struct.unpack(">h", d[4:6])[0] / 10.0}
            elif ptype == P_STATUS_5 and len(d) >= 6:
                upd = {"tacho": struct.unpack(">i", d[0:4])[0],
                       "v_in": struct.unpack(">h", d[4:6])[0] / 10.0}
            else:
                return None
        except struct.error:
            return None
        rec.update(upd)
        t = time.time() - self.t0
        self.counts[(cid, ptype)] = self.counts.get((cid, ptype), 0) + 1
        if ptype == P_STATUS:
            g = self.gaps.setdefault(cid, [t, 0.0, 0])
            dt_g = t - g[0]
            if dt_g > g[1]:
                g[1] = dt_g
            if dt_g > 0.15:
                g[2] += 1
            g[0] = t
        self.rows.append([round(t, 3), cid, ptype, target] + [
            rec.get(k, "") for k in
            ("erpm", "duty", "current", "current_in",
             "temp_fet", "temp_motor", "v_in", "tacho", "amp_hours")])
        return cid


def drain(bus, col, seconds, target=0):
    seen = set()
    t0 = time.time()
    while time.time() - t0 < seconds:
        m = bus.recv(timeout=0.05)
        if m is not None:
            cid = col.feed(m, target)
            if cid is not None:
                seen.add(cid)
    return seen


def fmt(v, spec=".1f"):
    return format(v, spec) if isinstance(v, (int, float)) else "—"


def main():
    ports = sorted(glob.glob("/dev/ttyUSB*"))
    col = Collector()
    bus, ids = None, set()
    for port in ports:
        try:
            b = can.Bus(interface="seeedstudio", channel=port,
                        bitrate=500000, timeout=0.1)
        except Exception as e:
            print(f"[scan] {port}: не открылся: {e}")
            continue
        found = drain(b, col, 3.0)
        if found:
            bus, ids = b, found
            print(f"[scan] шина на {port}, VESC: {sorted(ids)}")
            break
        b.shutdown()
        print(f"[scan] {port}: тишина")
    if bus is None:
        print("Кадров VESC нет ни на одном порту — питание/подключение?")
        return 1

    # профиль скоростей: следим за eRPM в устоявшейся части каждой ступени
    steady = {}   # (step_idx, cid) -> list установившихся eRPM
    run_t0 = time.time()
    try:
        for si, target in enumerate(STEPS):
            target = max(-2000, min(2000, target))
            print(f"[run] ступень {si + 1}/{len(STEPS)}: {target} eRPM, "
                  f"{STEP_S:.0f} с")
            # Цикл — как в боевом драйвере: TX по расписанию 50 Гц,
            # ограниченный дренаж, короткий сон. Иначе метки времени
            # обработки кадров врут и рисуют ложные "провалы линка".
            t0 = time.time()
            next_tx = t0
            while time.time() - t0 < STEP_S:
                now = time.time()
                if now >= next_tx:
                    for cid in ids:
                        send_rpm(bus, cid, target)
                    next_tx += 0.02
                for _ in range(32):
                    m = bus.recv(timeout=0)
                    if m is None:
                        break
                    cid = col.feed(m, target)
                    if cid is not None and time.time() - t0 > STEP_S * 0.5:
                        e = col.last[cid].get("erpm")
                        if e is not None:
                            steady.setdefault((si, cid), []).append(e)
                time.sleep(0.002)
    finally:
        for _ in range(5):
            for cid in ids:
                try:
                    send_current(bus, cid, 0.0)
                except Exception:
                    pass
            time.sleep(0.02)
    run_s = time.time() - run_t0
    # дослушиваем хвост телеметрии после остановки
    drain(bus, col, 1.5)
    bus.shutdown()

    with open(CSV_PATH, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "can_id", "ptype", "target_erpm", "erpm", "duty",
                    "current", "current_in", "temp_fet", "temp_motor",
                    "v_in", "tacho", "amp_hours"])
        w.writerows(col.rows)

    total_s = time.time() - col.t0
    print(f"\n===== СВОДКА (кадров: {len(col.rows)}, CSV: {CSV_PATH}) =====")
    problems = []
    print("\nЧастоты статусов, Гц (за весь тест):")
    print("  ID   St1   St2   St4   St5")
    for cid in sorted(ids):
        r = {p: col.counts.get((cid, p), 0) / total_s
             for p in (P_STATUS, P_STATUS_2, P_STATUS_4, P_STATUS_5)}
        print(f"  {cid:3d}  {r[P_STATUS]:5.1f} {r[P_STATUS_2]:5.1f} "
              f"{r[P_STATUS_4]:5.1f} {r[P_STATUS_5]:5.1f}")
        if r[P_STATUS] < 10:
            problems.append(f"ID {cid}: Status1 {r[P_STATUS]:.1f} Гц — "
                            "мало для управления (нужно 50)")
        if r[P_STATUS_5] < 10:
            problems.append(f"ID {cid}: Status5 {r[P_STATUS_5]:.1f} Гц — "
                            "одометрия работать не будет (нужно 50)")

    print("\nСтабильность линка (по Status1): макс. пауза между кадрами / "
          "число пауз >150 мс:")
    for cid in sorted(ids):
        g = col.gaps.get(cid)
        if g:
            print(f"  ID {cid:3d}: max_gap={g[1] * 1000:6.0f} мс  "
                  f"провалов>150мс: {g[2]}")
            if g[1] > 0.5:
                problems.append(f"ID {cid}: провал телеметрии {g[1]:.2f} с — "
                                "линк рвётся (контакт/помехи)")

    print("\nЭлектрика и температура:")
    for cid in sorted(ids):
        rec = col.last.get(cid, {})
        v = rec.get("v_in")
        print(f"  ID {cid:3d}: v_in={fmt(v)}В  i_mot={fmt(rec.get('current'))}А"
              f"  i_in={fmt(rec.get('current_in'))}А"
              f"  t_fet={fmt(rec.get('temp_fet'))}°C"
              f"  t_mot={fmt(rec.get('temp_motor'))}°C"
              f"  tacho={rec.get('tacho', '—')}")
        if isinstance(v, (int, float)) and not (19.0 <= v <= 26.0):
            problems.append(f"ID {cid}: v_in={v}В вне нормы 6S (19–25.2)")
        tf = rec.get("temp_fet")
        if isinstance(tf, (int, float)) and tf > 60:
            problems.append(f"ID {cid}: t_fet={tf}°C — горячо для холостого")

    print("\nСлежение за целью (средний eRPM во 2-й половине ступени):")
    hdr = "  цель " + "".join(f"  ID{cid:<4d}" for cid in sorted(ids))
    print(hdr)
    for si, target in enumerate(STEPS):
        cells = []
        for cid in sorted(ids):
            vals = steady.get((si, cid), [])
            if vals:
                mean = sum(vals) / len(vals)
                cells.append(f"{mean:7.0f}")
                if abs(mean - target) > max(150, 0.25 * target):
                    problems.append(
                        f"ступень {target}: ID {cid} держит {mean:.0f}")
            else:
                cells.append("      —")
        print(f"  {target:5d}" + "".join(cells))

    print()
    if problems:
        print("ПРОБЛЕМЫ:")
        for p in problems:
            print("  ✗ " + p)
        print("\nИТОГ: шина живая, но есть замечания выше.")
        return 1
    print("ИТОГ: ✓ всё в норме — телеметрия полная, моторы держат цель.")
    return 0


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(1))
    sys.exit(main())
