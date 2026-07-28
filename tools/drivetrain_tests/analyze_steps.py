import csv
import statistics as st
from collections import defaultdict

rows = list(csv.DictReader(open('can_telemetry.csv')))
print(f"frames: {len(rows)}")

# провалы по Status1 (ptype 9) на весь бас: по каждому cid отдельно
stamps = defaultdict(list)
for r in rows:
    if r['ptype'] == '9':
        stamps[r['can_id']].append((float(r['t']), r['target_erpm']))

# шкала ступеней: target по времени (берём любой cid)
print("\nПровалы >150 мс по времени (какая ступень шла):")
for cid in sorted(stamps, key=int):
    s = stamps[cid]
    for (t0, tgt0), (t1, tgt1) in zip(s, s[1:]):
        gap = t1 - t0
        if gap > 0.15:
            print(f"  ID {cid}: t={t0:6.2f}..{t1:6.2f}  gap={gap*1000:4.0f} мс  "
                  f"target={tgt0}->{tgt1}")

# удержание цели по ступеням
print("\nУдержание (средний eRPM, 2-я половина ступени):")
series = defaultdict(list)
for r in rows:
    if r['ptype'] == '9' and r['erpm'] != '':
        series[r['can_id']].append((float(r['t']), int(r['erpm']),
                                    int(r['target_erpm'])))
for cid in sorted(series, key=int):
    blocks, cur, blk = [], None, []
    for t, e, tgt in series[cid]:
        if tgt != cur:
            if blk:
                blocks.append((cur, blk))
            cur, blk = tgt, []
        blk.append((t, e))
    if blk:
        blocks.append((cur, blk))
    out = []
    for tgt, b in blocks:
        if tgt == 0:
            continue
        t_start = b[0][0]
        half = [e for t, e in b if t >= t_start + 1.5]
        mean = int(st.mean(half)) if half else -9999
        out.append(f"{tgt}:{mean}")
    print(f"  ID {cid}: " + "  ".join(out))
