#!/usr/bin/env python3
"""
bal_sim.py — 平衡环参数离线仿真 (倒立摆 + 轮子)

为什么需要它
    真机调 PID 的成本很高 (每次都要扶车、每次都可能摔)。
    这里用一个简化但物理方向正确的模型, 先把"哪些参数组合是稳定的、
    哪些是明显发散的"扫一遍, 真机只需要在稳定区里细调。

模型 (轮 + 倒立摆, 单自由度)
    θ'' = (g/L)·sin θ − k·u
    u   = motor_sign · 控制器输出 / 100      (u > 0 = 轮子往前追)
    k   = 电机把输出变成轮加速度的系数

    ⚠ motor_sign = 固件里 balance.h 的 **BAL_MOTOR_SIGN**:
      固件的 out = bal_out · BAL_MOTOR_SIGN, 而"轮子往倒的方向追"要求
      bal_out 与 θ 同号 (前倾 θ>0 → bal_out>0), 所以只有 BAL_MOTOR_SIGN=+1
      时 u 才与 θ 同号。2026-10-25 真机实测: 之前固件取的是 -1, 车必然往前扑倒。

控制器 (与固件 user/control/pid.c 的 pid_update_rate() 完全一致)
    out = Kp·θ_deg + Ki·i + Kd·θ̇_deg      ← D 项**正号**
    i   = clamp(i + θ_deg·dt, i_max)

    ⚠ D 项必须是正号: 车往前倒 (θ>0) 时轮子要往倒的方向追才接得住。
      取负号 → 负阻尼 → 越晃越大, 必然摔倒 (见 logs/2026-10-25.md)。

用法
    python3 test/bal_sim.py            # 跑默认参数 + 参数扫描
    python3 test/bal_sim.py --kp 10 --kd 1.2
    python3 test/bal_sim.py --motor-sign -1   # 复现"电机方向装反"→ 必摔
"""

import argparse
import math

G = 9.81


class Sim:
    """倒立摆 + PID 闭环仿真"""

    def __init__(self, kp, ki, kd, out_max=30.0, L=0.12, k_motor=91.8,
                 i_max=None, d_sign=+1, dt=0.01, motor_sign=+1):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_max = out_max
        self.L, self.k_motor = L, k_motor
        self.d_sign = d_sign
        self.dt = dt
        # 固件: out(电机) = bal_out * BAL_MOTOR_SIGN, 再换算成轮加速度
        # (u > 0 = 轮子往前追 = 往倒的方向追)
        self.motor_sign = motor_sign
        # 固件里夹的是**累加值 i**, 积分项上限 = out_max/2 → i_max = (out_max/2)/Ki
        self.i_max = (out_max * 0.5 / ki) if (ki > 0 and i_max is None) else (i_max or 0.0)

    def run(self, theta0_deg=6.0, seconds=10.0):
        theta = math.radians(theta0_deg)
        rate = 0.0
        i = 0.0
        w0sq = G / self.L
        n = int(seconds / self.dt)
        outs, thetas = [], []

        for _ in range(n):
            err_deg = math.degrees(theta)
            rate_deg = math.degrees(rate)
            i = max(-self.i_max, min(self.i_max, i + err_deg * self.dt)) if self.i_max else i + err_deg * self.dt
            out = self.kp * err_deg + self.ki * i + self.d_sign * self.kd * rate_deg
            out = max(-self.out_max, min(self.out_max, out))
            u = self.motor_sign * out / 100.0

            thdd = w0sq * math.sin(theta) - self.k_motor * u
            rate += thdd * self.dt
            theta += rate * self.dt
            outs.append(out)
            thetas.append(math.degrees(theta))

            if abs(math.degrees(theta)) > 60:      # 摔了
                break

        # 指标: 绝对输出的均方根 (越小越安静), 末尾振幅, 是否摔
        rms = math.sqrt(sum(o * o for o in outs) / max(1, len(outs)))
        tail = thetas[-int(0.5 / self.dt):] or thetas[-1:]
        tail_max = max(abs(t) for t in tail)
        return {
            "rms": rms,
            "tail_max": tail_max,
            "fell": abs(thetas[-1]) > 60,
            "out_max_used": max(abs(o) for o in outs) if outs else 0.0,
        }


def fmt(r):
    if r["fell"]:
        return "摔了!"
    return f"RMS {r['rms']:5.1f}%  末段振幅 {r['tail_max']:5.2f}°  峰值输出 {r['out_max_used']:4.1f}%"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kp", type=float, default=12.0)
    ap.add_argument("--ki", type=float, default=0.5)
    ap.add_argument("--kd", type=float, default=1.5)
    ap.add_argument("--out-max", type=float, default=30.0)
    ap.add_argument("--motor-sign", type=float, default=+1.0,
                    help="固件 balance.h 的 BAL_MOTOR_SIGN (真机实测为 +1; 取 -1 会立刻扑倒)")
    ap.add_argument("--scan", action="store_true", help="跑参数扫描")
    args = ap.parse_args()

    print("=" * 66)
    print(f"当前参数: Kp={args.kp}  Ki={args.ki}  Kd={args.kd}  上限={args.out_max}%")
    print(f"电机方向 BAL_MOTOR_SIGN={args.motor_sign:+.0f}")
    print("=" * 66)

    sim = Sim(args.kp, args.ki, args.kd, args.out_max, motor_sign=args.motor_sign)
    print(f"  D 符号=正 (正确): {fmt(sim.run())}")

    bad = Sim(args.kp, args.ki, args.kd, args.out_max, d_sign=-1,
              motor_sign=args.motor_sign)
    print(f"  D 符号=负 (对照): {fmt(bad.run())}")

    flipped = Sim(args.kp, args.ki, args.kd, args.out_max,
                  motor_sign=-args.motor_sign)
    print(f"  电机方向取反 (对照): {fmt(flipped.run())}")
    print()

    if args.scan:
        print("参数扫描 (D 符号=正, 只列稳定且安静的组合):")
        print(f"  {'Kp':>5} {'Ki':>5} {'Kd':>5}  {'RMS':>6}  {'末段振幅':>9}  峰值输出")
        results = []
        for kp in (6, 8, 10, 12, 15, 18, 22, 28):
            for ki in (0.0, 0.3, 0.5, 1.0):
                for kd in (0.5, 1.0, 1.5, 2.0, 3.0):
                    r = Sim(kp, ki, kd, args.out_max).run()
                    if not r["fell"] and r["rms"] < 12.0:
                        results.append((kp, ki, kd, r))
        results.sort(key=lambda t: t[3]["rms"])
        for kp, ki, kd, r in results[:20]:
            print(f"  {kp:5} {ki:5} {kd:5}  {r['rms']:6.1f}  {r['tail_max']:9.2f}  {r['out_max_used']:8.1f}")
        print("\n  提示: RMS 越小说明电机越安静; 挑 RMS 小的, 再看峰值输出有没有顶到上限。")
        print("  真机上还要按 logs/2026-10-25.md 的顺序微调 (配平 → Kp → Kd → Ki)。")


if __name__ == "__main__":
    main()
