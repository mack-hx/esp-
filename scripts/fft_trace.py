#!/usr/bin/env python3
"""对抓到的 trace 做频谱分析: 找出真实的振荡频率和各分量的贡献。

用法: python3 scripts/fft_trace.py <csv>
"""
import csv
import sys

import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/hi.csv"
rows = [r for r in csv.reader(open(path)) if r and r[0] != "t_ms"]
if not rows:
    print("没数据")
    sys.exit(1)

t = np.array([float(r[0]) for r in rows]) / 1000.0
cols = {name: np.array([float(r[i]) for r in rows])
        for i, name in enumerate(
            ["t", "pitch", "gyro", "out", "tgt", "spd_tgt", "spd",
             "yaw", "pos", "pos_out", "dt", "st"]) if i < len(rows[0])}

# 取 BALANCING 且输出有明显动作的连续段
st = cols["st"]
act = (st == 2) & (np.abs(cols["out"]) > 5)
segs, cur = [], []
for i in range(len(st)):
    if act[i]:
        cur.append(i)
    elif cur:
        segs.append(cur)
        cur = []
if cur:
    segs.append(cur)

print(f"总帧数 {len(rows)}, 找到 {len(segs)} 段抖动")
for si, seg in enumerate(segs):
    if len(seg) < 40:
        continue
    idx = np.array(seg)
    tt = t[idx]
    dur = tt[-1] - tt[0]
    if dur < 0.5:
        continue
    print(f"\n=== 段{si}: {len(seg)} 帧, {dur:.1f}s, "
          f"输出峰峰 {cols['out'][idx].max()-cols['out'][idx].min():.0f}% ===")
    fs = 1.0 / np.mean(np.diff(tt))
    print(f"采样率 {fs:.1f} Hz (名义 50Hz), Nyquist {fs/2:.1f} Hz")
    for name in ["pitch", "gyro", "out"]:
        x = cols[name][idx]
        x = x - x.mean()
        win = np.hanning(len(x))
        X = np.abs(np.fft.rfft(x * win))
        f = np.fft.rfftfreq(len(x), 1 / fs)
        k = X.argmax()
        # 峰值能量占比
        tot = (X ** 2).sum() or 1
        print(f"  {name:6s} 主频 {f[k]:6.2f} Hz  幅度 {x.max()-x.min():7.2f}  "
              f"主峰能量占比 {100*(X[k]**2)/tot:5.1f}%  "
              f"前3峰: " + ", ".join(f"{f[i]:.1f}Hz" for i in np.argsort(X)[-3:][::-1]))
    # 陀螺积分 vs 姿态角变化 (物理一致性)
    g = cols["gyro"][idx]
    p = cols["pitch"][idx]
    integ = np.trapz(g, tt)
    dpx = p[-1] - p[0]
    print(f"  一致性: ∫gyro={integ:+.1f}°  Δpitch={dpx:+.1f}°  "
          f"比值={integ/dpx if abs(dpx)>0.5 else float('nan'):.1f} (物理上应≈1)")
    print(f"  dt: {cols['dt'][idx].min():.1f}~{cols['dt'][idx].max():.1f}ms  "
          f"out 范围 {cols['out'][idx].min():.0f}~{cols['out'][idx].max():.0f}")
