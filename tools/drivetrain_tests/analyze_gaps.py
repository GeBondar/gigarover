import csv
from collections import defaultdict

rows = list(csv.DictReader(open('can_telemetry.csv')))
print(f"frames: {len(rows)}")
# был ли вообще ненулевой eRPM (транзиенты)
peak = defaultdict(lambda: [0, 0])   # cid -> [max_abs_erpm, n_nonzero]
stamps = defaultdict(list)           # cid -> все t кадров Status1
for r in rows:
    if r['ptype'] == '9' and r['erpm'] != '':
        cid = r['can_id']
        e = abs(int(r['erpm']))
        stamps[cid].append(float(r['t']))
        if e > peak[cid][0]:
            peak[cid][0] = e
        if e > 50:
            peak[cid][1] += 1
for cid in sorted(peak, key=int):
    p = peak[cid]
    print(f"ID {cid}: max|eRPM|={p[0]:5d}, кадров с вращением: {p[1]}")

# кластеры провалов: где по времени дыры >150 мс (по ID 32)
print("\nПровалы >150 мс по ID 32 (t начала, длительность мс):")
ts = stamps['32']
shown = 0
for a, b in zip(ts, ts[1:]):
    if b - a > 0.15:
        print(f"  t={a:6.2f}  gap={(b - a) * 1000:5.0f}")
        shown += 1
        if shown >= 25:
            break
