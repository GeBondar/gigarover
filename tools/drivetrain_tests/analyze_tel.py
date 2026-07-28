import csv
import statistics as st
from collections import defaultdict

rows = list(csv.DictReader(open('can_telemetry.csv')))
print(f"frames: {len(rows)}")
series = defaultdict(list)
for r in rows:
    if r['ptype'] == '9' and r['erpm'] != '':
        series[r['can_id']].append((float(r['t']), int(r['erpm']),
                                    float(r['duty'] or 0),
                                    int(r['target_erpm'])))
for cid in sorted(series, key=int):
    s = series[cid]
    print(f"\n=== VESC ID {cid} ===")
    blocks, cur_t, block = [], None, []
    for t, e, d, tgt in s:
        if tgt != cur_t:
            if block:
                blocks.append((cur_t, block))
            cur_t, block = tgt, []
        block.append((t, e, d))
    if block:
        blocks.append((cur_t, block))
    for tgt, b in blocks:
        if tgt == 0:
            continue
        es = [e for _, e, _ in b]
        ds = [d for _, _, d in b]
        zeros = sum(1 for e in es if abs(e) < 50)
        print(f"  target {tgt:5d}: n={len(es):3d}  erpm min/avg/max = "
              f"{min(es):5d}/{int(st.mean(es)):5d}/{max(es):5d}  "
              f"std={int(st.pstdev(es)):4d}  zeros={zeros:3d}  "
              f"duty avg/max={st.mean(ds):+.3f}/{max(ds):+.3f}")
