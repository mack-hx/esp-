#!/usr/bin/env python3
"""对抓到的 trace 做频谱分析: 找出真实的振荡频率和各分量的贡献。

用法: python3 scripts/fft_trace.py <csv>
"""
import csv
import sys

import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/hi.csv"
with open(path) as f:
    rdr = csv.reader(f)
    header = next(rdr, None)
    rows = [r for r in rdr if r]
if not rows:
    print("没数据")
    sys.exit(1)

# 表头里的列名 (t_ms 是第一列, 后面是各字段)
# ⚠ 跟固件 user/trace/trace.c 的 trace_print_one() 和 capture_trace.py 一一对应
names = (header[1:] if header and header[0] == "t_ms" else
         ["pitch", "gyro", "out", "tgt", "spd", "yaw", "pos",
          "pos_out", "kd", "dt", "st"])
t = np.array([float(r[0]) for r in rows]) / 1000.0
# ⚠ 列号要跳过第 0 列 (t_ms): names 里的第 0 个是 pitch, 它在 row 里的下标是 1。
#   原来写成 enumerate(names) 直接当下标用 → 整张表**错位一列**,
#   st 读到的是 pos_out, 于是永远找不到 BALANCING 段。
cols = {name: np.array([float(r[i + 1]) for r in rows])
        for i, name in enumerate(names) if i + 1 < len(rows[0])}

# 取 BALANCING 的连续段 (按状态切, 不用"输出幅度"当门槛)
#
# ⚠ 2026-10-27 修: 原来用 "|out| > 5" 当门槛, 但**振荡信号的输出本来
#   就在 0 附近来回穿** (比如 out 在 ±48% 之间摆, 有一半帧 |out| < 5),
#   于是每一段振荡被切成几十个碎片, 每片都不到 40 帧 → 脚本报告"找到 0 段抖动",
#   **恰恰在真正抖的时候什么都不输出**。改成按状态切连续段。
st = cols["st"]
segs, cur = [], []
for i in range(len(st)):
    if st[i] == 2:
        cur.append(i)
    elif cur:
        segs.append(cur)
        cur = []
if cur:
    segs.append(cur)

print(f"总帧数 {len(rows)}, 找到 {len(segs)} 段 BALANCING")
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
    print(f"采样率 {fs:.1f} Hz, Nyquist {fs/2:.1f} Hz")
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
    integ = np.trapezoid(g, tt) if hasattr(np, "trapezoid") else np.trapz(g, tt)
    dpx = p[-1] - p[0]
    print(f"  一致性: ∫gyro={integ:+.1f}°  Δpitch={dpx:+.1f}°  "
          f"比值={integ/dpx if abs(dpx)>0.5 else float('nan'):.1f} (物理上应≈1)")
    print(f"  dt: {cols['dt'][idx].min():.1f}~{cols['dt'][idx].max():.1f}ms  "
          f"out 范围 {cols['out'][idx].min():.0f}~{cols['out'][idx].max():.0f}")
