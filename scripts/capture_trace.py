#!/usr/bin/env python3
"""抓 ESP32 串口的 BALANCE TRACE 行, 存成 CSV 方便分析。

两种数据分开存:
  <输出文件>.csv   所有实时 TRACE 行 (带抓包时刻)
  <输出文件>.dump.csv 固件回灌的那一段 (摔倒/关闭平衡时自动打印的最近 30 秒)

用法: python3 scripts/capture_trace.py [秒数] [输出文件前缀]
"""
import os
import re
import sys
import time

import serial

PORT = "/dev/cu.usbmodem21133401"
SECS = int(sys.argv[1]) if len(sys.argv) > 1 else 120
BASE = sys.argv[2] if len(sys.argv) > 2 else "/tmp/trace"

pat = re.compile(
    r"TRACE t=([\d.]+) pitch=(-?[\d.]+) gyro=(-?[\d.]+) out=(-?[\d.]+) "
    r"spd=(-?[\d.]+) kd=(-?[\d.]+) dt=([\d.]+) st=(\d+)"
)
HEAD = "t_ms,pitch,gyro,out,spd,kd,dt,st\n"

s = serial.Serial(PORT, 230400, timeout=0.5)
t0 = time.time()
n = 0
in_dump = False
dumped = 0
with open(BASE + ".csv", "w") as f, open(BASE + ".dump.csv", "w") as fd:
    f.write(HEAD)
    fd.write(HEAD)
    buf = b""
    while time.time() - t0 < SECS:
        try:
            b = s.read(512)
        except serial.SerialException as e:
            # USB CDC 偶发重枚举 (Errno 6 Device not configured) → 重开串口继续抓
            print("串口掉线, 3 秒后重连: %s" % e, flush=True)
            time.sleep(3)
            try:
                s.close()
            except Exception:
                pass
            try:
                s = serial.Serial(PORT, 230400, timeout=0.5)
            except Exception as e2:
                print("重连失败: %s" % e2, flush=True)
                time.sleep(3)
                continue
            continue
        if not b:
            continue
        # ⚠ 必须把跨块的分行拼起来: 直接 splitlines 会丢掉块边界的半行
        buf += b
        lines = buf.split(b"\n")
        buf = lines.pop()
        for raw in lines:
            line = raw.decode("utf-8", "replace")
            if "TRACE DUMP BEGIN" in line:
                in_dump = True
                continue
            if "TRACE DUMP END" in line:
                in_dump = False
                continue
            m = pat.search(line)
            if not m:
                continue
            n += 1
            g = list(m.groups())
            # 第一列: 固件自己的 esp_timer 时刻 (ms), 没有就退回主机抓包时刻
            if g[0] is not None:
                t_ms = f"{float(g[0]) * 1000:.0f}"
                g = g[1:]
            else:
                t_ms = f"{(time.time()-t0)*1000:.0f}"
            row = t_ms + "," + ",".join(g) + "\n"
            f.write(row)
            f.flush()
            if in_dump:
                fd.write(row)
                fd.flush()
                dumped += 1
s.close()
print(f"抓到 {n} 条 TRACE (其中回灌 {dumped} 条), 写入 {BASE}.csv / {BASE}.dump.csv")
