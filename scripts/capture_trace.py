#!/usr/bin/env python3
"""抓 ESP32 串口的 TRACE 行, 存成 CSV 方便分析。

两种数据分开存:
  <输出文件>.csv       所有实时 TRACE 行
  <输出文件>.dump.csv  固件回灌的那一段 (摔倒/关闭平衡时自动打印的历史)

用法:
  python3 scripts/capture_trace.py [秒数] [输出文件前缀] [串口设备]
例:
  python3 scripts/capture_trace.py 60 /tmp/trace /dev/cu.usbmodem21133401

⚠ 串口波特率必须跟固件一致 = 230400 (100Hz × 约 125 字节 = 12.5KB/s,
  115200 装不下, 会丢行 → 采样率就假了)。
"""
import os
import re
import sys
import time

import serial

# 串口设备: 优先用命令行参数, 其次环境变量 TRACE_PORT, 最后自动挑一个
def pick_port(arg):
    if arg:
        return arg
    env = os.environ.get("TRACE_PORT")
    if env:
        return env
    import glob
    cands = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/tty.usbmodem*"))
    if not cands:
        sys.exit("找不到串口设备 (用参数或 TRACE_PORT 指定, 如 /dev/cu.usbmodem2101)")
    return cands[0]


PORT = pick_port(sys.argv[3] if len(sys.argv) > 3 else None)
SECS = int(sys.argv[1]) if len(sys.argv) > 1 else 120
BASE = sys.argv[2] if len(sys.argv) > 2 else "/tmp/trace"
BAUD = 230400

# CSV 表头 (跟固件 trace.c 的 trace_print_one() 一一对应)
HEAD = "t_ms,pitch,gyro,out,tgt,spd,yaw,pos,pos_err,pos_out,pos_vel,kd,dt,st\n"

# 字段顺序必须跟 HEAD (除 t_ms) 一致
FIELDS = ["pitch", "gyro", "out", "tgt", "spd", "yaw", "pos", "pos_err",
          "pos_out", "pos_vel", "kd", "dt", "st"]
# 用 "key=value" 通用解析, 固件加字段时脚本不用改 (只认 HEAD 里有的)
pat = re.compile(r"(\w+)=(-?[\d.]+)")


def open_port():
    return serial.Serial(PORT, BAUD, timeout=0.5)


s = open_port()
print("串口 %s @ %d baud, 抓 %d 秒" % (PORT, BAUD, SECS), flush=True)
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
                s = open_port()
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
            if "TRACE" not in line:
                continue
            kv = dict(pat.findall(line))
            if "t" not in kv:
                continue
            # 第一列: 固件自己的 esp_timer 时刻 (ms), 没有就退回主机抓包时刻
            t_ms = f"{float(kv['t']) * 1000:.0f}"
            try:
                row = t_ms + "," + ",".join(kv[f] for f in FIELDS) + "\n"
            except KeyError:
                continue      # 字段不全的行 (半行/旧固件格式) → 跳过
            n += 1
            f.write(row)
            f.flush()
            if in_dump:
                fd.write(row)
                fd.flush()
                dumped += 1
s.close()
print(f"抓到 {n} 条 TRACE (其中回灌 {dumped} 条), 写入 {BASE}.csv / {BASE}.dump.csv")
