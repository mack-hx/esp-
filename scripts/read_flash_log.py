#!/usr/bin/env python3
"""读出固件存在 flash "storage" 分区里的脱机跟踪记录, 存成 CSV。

固件在离开平衡态 (摔倒 / 关闭平衡) 时, 把最近 40 秒的采样写进 storage 分区
(见 0.40/user/trace/trace.c)。脱机测试后插上 USB, 运行本脚本即可读出。

用法:
  python3 scripts/read_flash_log.py [输出文件前缀] [串口设备]
例:
  python3 scripts/read_flash_log.py /tmp/flog

输出: <前缀>.csv, 表头与 capture_trace.py 一致, 分析脚本通用。
读完板子会被 esptool 复位一次; flash 里的记录不会被清除, 下次离开平衡态时才被覆盖。

⚠ 记录格式必须跟 trace.c 的 flog_hdr_t / trace_sample_t 一致 (56 字节/条)。
"""
import glob
import os
import struct
import subprocess
import sys
import tempfile

PART_ADDR = 0x720000      # partitions.csv 里 storage 的偏移
SECTOR = 4096
HDR_FMT = "<4I"           # magic, count, rec_size, reserved
REC_FMT = "<13fb3x"       # 13 个 float + int8 状态 + 3 字节对齐
REC_SIZE = struct.calcsize(REC_FMT)
MAGIC = 0x474F4C54
FLOG_MAX = 4000           # 固件 FLOG_N: 最多记录条数
HEAD = "t_ms,pitch,gyro,out,tgt,spd,yaw,pos,pos_err,pos_out,pos_vel,kd,dt,st\n"

ESPTOOL = os.path.expanduser("~/.platformio/packages/tool-esptoolpy/esptool.py")
PYTHON = os.path.expanduser("~/.platformio/penv/bin/python")


def pick_port(arg):
    if arg:
        return arg
    env = os.environ.get("TRACE_PORT")
    if env:
        return env
    cands = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not cands:
        sys.exit("找不到串口设备 (用参数或 TRACE_PORT 指定)")
    return cands[0]


def read_flash(port, offset, size, out, reset):
    cmd = [PYTHON, ESPTOOL, "--chip", "esp32s3", "--port", port, "--baud", "460800",
           "--after", "hard_reset" if reset else "no_reset",
           "read_flash", hex(offset), hex(size), out]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit("esptool 读取失败:\n" + r.stdout[-600:] + r.stderr[-600:])


def main():
    base = sys.argv[1] if len(sys.argv) > 1 else "/tmp/flog"
    port = pick_port(sys.argv[2] if len(sys.argv) > 2 else None)

    # 一次连接读完"头 + 最大记录数", 避免读到一半重新连接
    total = SECTOR + (FLOG_MAX * REC_SIZE + SECTOR - 1) // SECTOR * SECTOR
    with tempfile.TemporaryDirectory() as tmp:
        raw_bin = os.path.join(tmp, "flog.bin")
        read_flash(port, PART_ADDR, total, raw_bin, reset=True)
        raw = open(raw_bin, "rb").read()
    magic, count, rec_size, _ = struct.unpack_from(HDR_FMT, raw, 0)
    if magic != MAGIC:
        sys.exit("flash 里没有有效记录 (头无效: 还没离开过平衡态, 或保存中途被打断)")
    if rec_size != REC_SIZE:
        sys.exit("记录大小不符: flash=%d 脚本=%d, 固件与脚本版本不一致" % (rec_size, REC_SIZE))
    if count > FLOG_MAX:
        sys.exit("记录条数异常: %d" % count)
    data = raw[SECTOR:]

    with open(base + ".csv", "w") as f:
        f.write(HEAD)
        for i in range(count):
            (t, pitch, gyro, out, tgt, spd, yaw, pos, pos_err, pos_out, pos_vel,
             kd, dt, st) = struct.unpack_from(REC_FMT, data, i * REC_SIZE)
            f.write(f"{t * 1000:.0f},{pitch:.2f},{gyro:.1f},{out:.1f},{tgt:.2f},"
                    f"{spd:.1f},{yaw:.1f},{pos:.1f},{pos_err:.2f},{pos_out:.1f},"
                    f"{pos_vel:.2f},{kd:.2f},{dt:.1f},{st}\n")
    print("读出 %d 条, 写入 %s.csv" % (count, base))


if __name__ == "__main__":
    main()
