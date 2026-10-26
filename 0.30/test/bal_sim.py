#!/usr/bin/env python3
"""
bal_sim.py — 平衡环离线仿真 (倒立摆 + 轮子 + 电机 + IMU)

为什么需要它
    真机调 PID 的成本很高 (每次都要扶车、每次都可能摔)。
    这里用一个物理方向正确的模型, 先把"哪些参数组合是稳的、
    哪些是一碰就抖的"扫一遍, 真机只需要在稳定区里细调。

模型 (轮 + 倒立摆, 单自由度)
    θ'' = (g/L)·sin θ − a          a = 轮子的等效加速度 (rad/s²)

控制器 (与固件 user/control/pid.c 的 pid_update_rate() 完全一致)
    out = Kp·θ_deg + Ki·i + Kd·θ̇_deg      ← D 项**正号**
    i   = clamp(i + θ_deg·dt, i_max), 输出顶限幅且误差同向时**冻结**积分

电机链路 (固件 hardware/motor_driver/motor_driver.c 的真实行为)
    1) 输出截断成 int8 (整数 %)
    2) 死区补偿: dz_mode="raise" → 非零输出抬到 dz_fw (固件现状)
                  dz_mode="add"   → 非零输出加上 dz_fw (建议改法)
    3) 硬件死区: |duty| < dz_true 时电机**根本不转** (力矩为 0)
    4) 一阶滞后 τ: PWM → 力矩 → 轮加速度 不是瞬时的

IMU 链路 (固件 hardware/mpu9250/mpu9250.c 的真实行为)
    1) 加速度计测的是**比力**: 静止时读 1g, 车一加速就多出"车体加速度"
       → atan2(az, −ay) 算出来的俯仰角会**偏几十度**
    2) 互补滤波: θ̂ = α·(θ̂ + gyro·dt) + (1−α)·accel_pitch
       → 污染的加速度角会以 (1−α) 的权重持续把姿态角拽偏
    3) 可选的"可信度门控": |a| 偏离 1g 越多, 加速度角的权重越低
       (dev < 0.05 → 权重 1; dev > 0.30 → 权重 0, 只信陀螺)

用法
    python3 test/bal_sim.py                       # 默认参数 + 对照组
    python3 test/bal_sim.py --scan                # 参数扫描
    python3 test/bal_sim.py --no-contam           # 关掉加速度计污染 (对照)
    python3 test/bal_sim.py --no-trust            # 关掉可信度门控 (对照)
    python3 test/bal_sim.py --push 30              # 加一个角速度冲量
"""

import argparse
import math

G = 9.81


class Sim:
    """倒立摆 + PID 闭环 + 电机链路 + IMU 仿真"""

    def __init__(self, kp, ki, kd, out_max=60.0, L=0.12, k_motor=91.8,
                 i_max=None, d_sign=+1, dt=0.01, motor_sign=+1,
                 dz_fw=4.0, dz_true=12.0, dz_mode="raise", tau=0.05,
                 gyro_lpf=0.2, alpha=0.98,
                 contam=+1, trust=True,
                 trust_lo=0.05, trust_hi=0.30):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_max = out_max
        self.L, self.k_motor = L, k_motor
        self.d_sign = d_sign
        self.dt = dt
        self.motor_sign = motor_sign
        self.dz_fw = dz_fw          # 固件里配的死区补偿值 (%)
        self.dz_true = dz_true      # 电机硬件真实死区 (%)
        self.dz_mode = dz_mode      # "raise" | "add"
        self.tau = tau              # 电机一阶滞后 (s)
        self.gyro_lpf = gyro_lpf    # 陀螺一阶低通系数 (每周期)
        self.alpha = alpha          # 互补滤波系数 (陀螺权重)
        self.contam = contam        # 加速度计污染方向: +1 / -1 / 0 (关)
        self.trust = trust          # 可信度门控
        self.trust_lo = trust_lo    # |a| 偏离 1g 多少以内 → 完全信加速度计
        self.trust_hi = trust_hi    # 偏离多少以上 → 完全不信 (只信陀螺)
        # 固件: i_max = (out_max*0.5)/Ki, 积分项最大 = 输出一半
        self.i_max = (out_max * 0.5 / ki) if (ki > 0 and i_max is None) else (i_max or 0.0)

    # ---------- 电机链路 ----------
    def _duty(self, out_pct):
        """控制器输出 (%) → 实际加到 PWM 上的占空比 (%)"""
        v = int(out_pct)                     # 固件: (int8_t)out 截断
        if v == 0:
            return 0.0
        if self.dz_mode == "add":
            d = abs(v) + self.dz_fw
        else:                                # 固件现状: 抬到死区
            d = max(abs(v), self.dz_fw)
        return math.copysign(min(d, 100.0), v)

    def _accel_cmd(self, duty):
        """占空比 (%) → 轮加速度 (rad/s²), 硬件死区在这里生效
        ⚠ 符号: 正方向 = "轮子往 θ 增大的方向跑"
          = motor_sign · (占空比扣掉死区后的余量)
          与 thdd = w0sq·sinθ − wheel_acc 配套 (motor_sign=+1 才稳定)"""
        if abs(duty) < self.dz_true:
            return 0.0
        return (self.motor_sign * self.k_motor *
                math.copysign(abs(duty) - self.dz_true, duty) / 100.0)

    # ---------- IMU 链路 ----------
    def _accel_pitch(self, theta, wheel_acc):
        """加速度计算出来的俯仰角 (°)
        @param theta     真实倾角 (rad)
        @param wheel_acc 轮加速度 (rad/s², 正方向 = 轮子跑的方向 = θ 增大的方向)
        ⚠ 加速度计测的是**比力**: 读数 = g·(车体上方向) − 车体平动加速度
          车往"轮子跑"的方向加速时, 读数里的水平分量被**抵消**一部分
          → 算出来的俯仰角偏小 → 控制器"以为还没倒那么厉害" → 少出力
          → 越晃越大 (正反馈)
          ⚠ contam 用来验证两种安装方向: +1 = 上面这种 (正反馈, 最危险),
            -1 = 装反了 (负反馈, 反而会掩盖问题)"""
        a_lin = self.L * wheel_acc * self.contam      # m/s²
        ay = math.sin(theta) - a_lin / G              # MPU 的 Y 轴 (车体前后)
        az = math.cos(theta)                           # MPU 的 Z 轴 (车体上)
        return math.degrees(math.atan2(az, -ay)) - 90.0

    def _trust(self, theta, wheel_acc):
        """加速度计可信度: |a| 偏离 1g 越多 → 越不可信"""
        if not self.trust:
            return 1.0
        a_lin = self.L * wheel_acc * self.contam
        ay = math.sin(theta) - a_lin / G
        az = math.cos(theta)
        mag = math.sqrt(ay * ay + az * az)
        dev = abs(mag - 1.0)
        if dev <= self.trust_lo:
            return 1.0
        if dev >= self.trust_hi:
            return 0.0
        return (self.trust_hi - dev) / (self.trust_hi - self.trust_lo)

    # ---------- 主循环 ----------
    def run(self, theta0_deg=8.0, seconds=10.0, push_deg_s=0.0, push_at=1.0):
        theta = math.radians(theta0_deg)
        rate = 0.0
        i = 0.0
        rate_f = 0.0
        accel = 0.0
        pitch_est = None                 # 互补滤波输出的姿态角 (控制器只看它)
        w0sq = G / self.L
        n = int(seconds / self.dt)
        outs, thetas, rates, ests = [], [], [], []
        pushed = False

        for t in range(n):
            tsec = t * self.dt
            if push_deg_s and not pushed and tsec >= push_at:
                rate += math.radians(push_deg_s)
                pushed = True

            # ---- IMU: 陀螺积分 + 加速度角 + 互补滤波 (固件同款) ----
            raw_rate = math.degrees(rate)
            if pitch_est is None:
                pitch_est = self._accel_pitch(theta, 0.0)
            else:
                pitch_est += raw_rate * self.dt        # 陀螺积分
                acc_pitch = self._accel_pitch(theta, accel)
                tw = self._trust(theta, accel)
                pitch_est += (1.0 - self.alpha) * tw * (acc_pitch - pitch_est)

            # 固件: 陀螺一阶低通 (balance.c 的 s_gyro_lpf)
            rate_f += self.gyro_lpf * (raw_rate - rate_f)

            # ---- 固件 pid_update_rate(): 误差用**姿态估计**, D 用角速度 ----
            err_deg = pitch_est
            d = self.d_sign * rate_f
            i_new = max(-self.i_max, min(self.i_max, i + err_deg * self.dt)) \
                if self.i_max else i + err_deg * self.dt
            out_now = self.kp * err_deg + self.ki * i + self.kd * d
            if not (out_now >= self.out_max and err_deg > 0) and \
               not (out_now <= -self.out_max and err_deg < 0):
                i = i_new
            out = self.kp * err_deg + self.ki * i + self.kd * d
            out = max(-self.out_max, min(self.out_max, out))

            # ---- 输出 → 电机 → 轮加速度 ----
            duty = self._duty(out)
            a_cmd = self._accel_cmd(duty)
            alpha_m = self.dt / max(self.tau, 1e-6)
            accel += (a_cmd - accel) * min(alpha_m, 1.0)

            thdd = w0sq * math.sin(theta) - accel
            rate += thdd * self.dt
            theta += rate * self.dt
            outs.append(out)
            thetas.append(math.degrees(theta))
            rates.append(raw_rate)
            ests.append(pitch_est)

            if abs(math.degrees(theta)) > 60:      # 摔了
                break

        rms = math.sqrt(sum(o * o for o in outs) / max(1, len(outs)))
        tail = thetas[-int(0.5 / self.dt):] or thetas[-1:]
        tail_max = max(abs(t) for t in tail)
        est_err = max(abs(a - b) for a, b in zip(ests, thetas)) if ests else 0.0
        # 摆动次数: 角速度过零的次数
        signs = [1 if r > 0 else (-1 if r < 0 else 0) for r in rates]
        nz = [s for s in signs if s != 0]
        osc = sum(1 for a, b in zip(nz, nz[1:]) if a != b)
        return {
            "rms": rms,
            "tail_max": tail_max,
            "fell": abs(thetas[-1]) > 60,
            "out_max_used": max(abs(o) for o in outs) if outs else 0.0,
            "peak_angle": max(abs(t) for t in thetas) if thetas else 0.0,
            "est_err": est_err,       # 姿态估计的最大误差 (°)
            "osc": osc,
            "final": thetas[-1] if thetas else 0.0,
        }


def fmt(r):
    if r["fell"]:
        return "摔了!"
    return (f"RMS {r['rms']:5.1f}%  末段振幅 {r['tail_max']:5.2f}°  "
            f"峰值角 {r['peak_angle']:5.2f}°  姿态估计误差 {r['est_err']:5.1f}°  "
            f"摆动 {r['osc']:3d} 次  峰值输出 {r['out_max_used']:4.1f}%")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kp", type=float, default=12.0)
    ap.add_argument("--ki", type=float, default=0.5)
    ap.add_argument("--kd", type=float, default=1.5)
    ap.add_argument("--out-max", type=float, default=60.0)
    ap.add_argument("--motor-sign", type=float, default=+1.0)
    ap.add_argument("--dz-fw", type=float, default=4.0, help="固件里配的死区补偿 (%)")
    ap.add_argument("--dz-true", type=float, default=12.0, help="电机硬件真实死区 (%)")
    ap.add_argument("--dz-mode", choices=["raise", "add"], default="add")
    ap.add_argument("--tau", type=float, default=0.05, help="电机一阶滞后 (s)")
    ap.add_argument("--gyro-lpf", type=float, default=0.8)
    ap.add_argument("--alpha", type=float, default=0.997, help="互补滤波陀螺权重")
    ap.add_argument("--no-contam", action="store_true", help="关掉加速度计污染")
    ap.add_argument("--no-trust", action="store_true", help="关掉可信度门控")
    ap.add_argument("--contam-dir", type=float, default=+1.0)
    ap.add_argument("--push", type=float, default=0.0, help="第 1 秒加的角速度冲量 (°/s)")
    ap.add_argument("--theta0", type=float, default=8.0, help="初始倾角 (°)")
    ap.add_argument("--scan", action="store_true")
    args = ap.parse_args()

    common = dict(out_max=args.out_max, dz_fw=args.dz_fw, dz_true=args.dz_true,
                  tau=args.tau, gyro_lpf=args.gyro_lpf, alpha=args.alpha)

    def mk(**kw):
        d = dict(common)
        d["dz_mode"] = args.dz_mode
        d.update(kw)
        ms = d.pop("motor_sign", args.motor_sign)
        return Sim(args.kp, args.ki, args.kd, motor_sign=ms, **d)

    contam = 0.0 if args.no_contam else args.contam_dir
    trust = not args.no_trust

    print("=" * 92)
    print(f"Kp={args.kp} Ki={args.ki} Kd={args.kd} 上限={args.out_max}% "
          f"死区补偿={args.dz_fw}%({args.dz_mode}) 硬件死区={args.dz_true}% "
          f"电机滞后={args.tau*1000:.0f}ms 陀螺低通={args.gyro_lpf} "
          f"互补α={args.alpha} 污染={'关' if args.no_contam else '开'}"
          f"({args.contam_dir:+.0f}) 门控={'关' if args.no_trust else '开'}")
    print(f"初始倾角 {args.theta0}°, 冲量 {args.push}°/s")
    print("=" * 92)

    def go(sim, label):
        r = sim.run(theta0_deg=args.theta0, push_deg_s=args.push)
        print(f"  {label:22s} {fmt(r)}")
        return r

    go(mk(contam=contam, trust=trust), "当前配置")
    go(mk(contam=contam, trust=trust, alpha=0.98), "旧: 互补α=0.98")
    go(mk(contam=contam, trust=trust, gyro_lpf=0.2), "旧: 陀螺低通=0.2")
    go(mk(contam=contam, trust=trust, dz_mode="raise"), "旧: 死区'抬到'")
    go(mk(contam=contam, trust=trust, alpha=0.98, gyro_lpf=0.2, dz_mode="raise"),
       "旧: 全部旧值")
    go(mk(contam=0.0, trust=trust), "无污染(对照)")
    go(mk(contam=contam, trust=False), "只信陀螺(无门控)")
    go(mk(contam=contam, trust=trust, alpha=0.997), "互补α=0.995")
    go(mk(contam=contam, trust=trust, d_sign=-1), "D 符号取负(对照)")
    go(mk(contam=contam, trust=trust, motor_sign=-args.motor_sign), "电机方向取反(对照)")
    print()

    if args.scan:
        print("参数扫描 (D 符号=正, 污染开, 门控开):")
        print(f"  {'Kp':>5} {'Ki':>5} {'Kd':>5}  {'RMS':>6}  {'末段振幅':>9}  "
              f"{'姿态误差':>8}  峰值输出")
        results = []
        for kp in (6, 8, 10, 12, 15, 18, 22, 28):
            for ki in (0.0, 0.3, 0.5, 1.0):
                for kd in (0.5, 1.0, 1.5, 2.0, 3.0, 4.0):
                    r = mk(kp=kp, ki=ki, kd=kd, contam=contam, trust=trust).run(
                        theta0_deg=args.theta0, push_deg_s=args.push)
                    if not r["fell"] and r["rms"] < 12.0:
                        results.append((kp, ki, kd, r))
        results.sort(key=lambda t: t[3]["rms"])
        for kp, ki, kd, r in results[:25]:
            print(f"  {kp:5} {ki:5} {kd:5}  {r['rms']:6.1f}  {r['tail_max']:9.2f}  "
                  f"{r['est_err']:8.1f}  {r['out_max_used']:8.1f}")
        print("\n  提示: RMS 越小说明电机越安静; 真机上按 配平 → Kp → Kd → Ki 的顺序微调。")


if __name__ == "__main__":
    main()
