#!/usr/bin/env python3
"""分析 capture_trace.py 抓到的 CSV: 分段(按状态) + 统计抖动幅度。
用法: python3 scripts/analyze_trace.py <csv> [段号]
"""
import sys

path = sys.argv[1]
rows = []
with open(path) as f:
    head = f.readline().strip().split(",")
    for line in f:
        p = line.strip().split(",")
        if len(p) == len(head):
            rows.append(p)

# 列: t_ms,pitch,gyro,out,tgt,spd_tgt,spd,yaw,pos,pos_out,dt,st
def col(name):
    i = head.index(name)
    return [float(r[i]) for r in rows]

states = col("st")
# 按状态切段
segs = []
cur = [0]
for i in range(1, len(rows)):
    if states[i] == states[i - 1]:
        cur.append(i)
    else:
        segs.append((states[cur[0]], cur[0], cur[-1]))
        cur = [i]
segs.append((states[cur[0]], cur[0], cur[-1]))

print(f"总行数 {len(rows)}")
for st, a, b in segs:
    if b - a < 2:
        continue
    name = {0: "IDLE", 1: "STANDBY", 2: "BALANCING", 3: "FALLEN"}.get(int(st), str(st))
    print(f"  st={int(st)}({name}) 行 {a}..{b} 共 {b-a+1} 帧 "
          f"({(b-a+1)*0.1:.1f}s)  t={rows[a][0]}..{rows[b][0]}ms")

print("\n=== BALANCING 段详细 ===")
for st, a, b in segs:
    if int(st) != 2 or b - a < 5:
        continue
    print(f"\n--- 段 行{a}..{b}  {b-a+1} 帧 ---")
    p = col("pitch")[a:b + 1]
    g = col("gyro")[a:b + 1]
    o = col("out")[a:b + 1]
    tg = col("tgt")[a:b + 1]
    y = col("yaw")[a:b + 1]
    dt = col("dt")[a:b + 1]
    sp = col("spd")[a:b + 1]
    print(f"pitch  min={min(p):7.2f} max={max(p):7.2f} 峰峰={max(p)-min(p):6.2f} 均值={sum(p)/len(p):7.2f}")
    print(f"gyro   min={min(g):7.2f} max={max(g):7.2f} 峰峰={max(g)-min(g):6.2f}")
    print(f"out    min={min(o):7.2f} max={max(o):7.2f} 峰峰={max(o)-min(o):6.2f}")
    print(f"tgt    min={min(tg):7.2f} max={max(tg):7.2f} 峰峰={max(tg)-min(tg):6.2f}")
    print(f"yaw    min={min(y):7.2f} max={max(y):7.2f} 峰峰={max(y)-min(y):6.2f}")
    print(f"spd    min={min(sp):7.2f} max={max(sp):7.2f}")
    print(f"dt     min={min(dt):7.2f} max={max(dt):7.2f}")
    # 零点穿越次数 → 粗略频率
    cross = sum(1 for i in range(1, len(p)) if (p[i - 1] - 0) * (p[i] - 0) < 0)
    dur = (b - a + 1) * 0.1
    print(f"零穿越 {cross} 次 / {dur:.1f}s → 约 {cross/max(dur,0.1)/2:.2f} Hz")
    print("波形(每 5 帧取 1): t_ms  pitch  gyro   out    tgt    yaw")
    for i in range(a, b + 1, 5):
        print(f"  {rows[i][0]:>8} {rows[i][1]:>7} {rows[i][2]:>6} {rows[i][3]:>6} "
              f"{rows[i][4]:>6} {rows[i][8]:>6}")
