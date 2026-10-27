#!/usr/bin/env python3
"""分析 capture_trace.py 抓到的 CSV: 分段(按状态) + 统计抖动幅度。

用法: python3 scripts/analyze_trace.py <csv> [段号]

CSV 表头 (固件 / 采集脚本 / 分析脚本 三方统一):
  t_ms,pitch,gyro,out,tgt,spd,yaw,pos,pos_out,kd,dt,st
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


def col(name):
    """按**列名**取列 (不写死下标, 固件加字段不会把这里带偏)"""
    i = head.index(name)
    return [float(r[i]) for r in rows]


states = col("st")
t_ms = col("t_ms")

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
    # 时长从时间戳算 (不假设采样率, 固件改采样周期这里不用改)
    dur = (t_ms[b] - t_ms[a]) / 1000.0
    print(f"  st={int(st)}({name}) 行 {a}..{b} 共 {b-a+1} 帧 "
          f"({dur:.1f}s)  t={t_ms[a]:.0f}..{t_ms[b]:.0f}ms")

print("\n=== BALANCING 段详细 ===")
for st, a, b in segs:
    if int(st) != 2 or b - a < 5:
        continue
    p = col("pitch")[a:b + 1]
    g = col("gyro")[a:b + 1]
    o = col("out")[a:b + 1]
    tg = col("tgt")[a:b + 1]
    y = col("yaw")[a:b + 1]
    dt = col("dt")[a:b + 1]
    sp = col("spd")[a:b + 1]
    kd = col("kd")[a:b + 1]
    po = col("pos_out")[a:b + 1]
    dur = (t_ms[b] - t_ms[a]) / 1000.0
    print(f"\n--- 段 行{a}..{b}  {b-a+1} 帧  {dur:.1f}s ---")
    print(f"pitch  min={min(p):7.2f} max={max(p):7.2f} 峰峰={max(p)-min(p):6.2f} 均值={sum(p)/len(p):7.2f}")
    print(f"gyro   min={min(g):7.2f} max={max(g):7.2f} 峰峰={max(g)-min(g):6.2f}")
    print(f"out    min={min(o):7.2f} max={max(o):7.2f} 峰峰={max(o)-min(o):6.2f}")
    print(f"tgt    min={min(tg):7.2f} max={max(tg):7.2f} 峰峰={max(tg)-min(tg):6.2f}")
    print(f"yaw    min={min(y):7.2f} max={max(y):7.2f} 峰峰={max(y)-min(y):6.2f}")
    print(f"spd    min={min(sp):7.2f} max={max(sp):7.2f}")
    print(f"posOut min={min(po):7.2f} max={max(po):7.2f}")
    print(f"kd     min={min(kd):7.2f} max={max(kd):7.2f}")
    print(f"dt     min={min(dt):7.2f} max={max(dt):7.2f}")
    # 零点穿越次数 → 粗略频率
    cross = sum(1 for i in range(1, len(p)) if (p[i - 1] - 0) * (p[i] - 0) < 0)
    print(f"零穿越 {cross} 次 / {dur:.1f}s → 约 {cross/max(dur,0.1)/2:.2f} Hz")
    print("波形(每 5 帧取 1): t_ms  pitch  gyro   out    tgt    yaw")
    for i in range(a, b + 1, 5):
        print(f"  {t_ms[i]:>8.0f} {p[i-a]:>7} {g[i-a]:>6} {o[i-a]:>6} "
              f"{tg[i-a]:>6} {y[i-a]:>6}")
