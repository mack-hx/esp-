#!/usr/bin/env python3
"""
bal_sim.py — 平衡环离线仿真 (倒立摆 + 轮子 + 电机 + IMU)

为什么需要它
    真机调 PID 的成本很高 (每次都要扶车、每次都可能摔)。
    这里用一个物理方向正确的模型, 先把"哪些参数组合是稳的、
    哪些是一碰就抖的"扫一遍, 真机只需要在稳定区里细调。

模型 (轮 + 倒立摆, 单自由度)
    θ'' = (g/L)·sin θ − a          a = 轮子的等效加速度 (rad/s²)

控制器 (与固件 user/pid/pid.c 的 pid_update_rate() 完全一致)
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


# ================= 行走仿真 (小车 + 倒立摆 + 速度环) =================
#
# 为什么要单独一个模型
#   bal_sim.py 的摆是**定轴**的 (没有轮子的位置/速度), 所以它只能验证
#   "站着稳不稳", 验证不了"走"。
#   行走必须知道车往哪走了: 目标轮速 → 目标倾角 → 车加速 → 速度变了 → 倾角回收。
#   这里把车当成一个真正的小车 (位置 x / 速度 v), 就能把速度环整个跑起来。
#
# 符号约定 (跟固件一致, 见 user/control/balance.c 的注释)
#   θ  = 车体倾角, **前倾 (车头往下) 为负** (固件的 pitch, 单位 °)
#   x  = 车的位置, **+x = 车尾方向** (电机输出为正 → 轮子往车尾 → 车往 +x 加速)
#   → 前进速度 v_fwd = −ẋ
#   → 平衡环: out = Kp·(θ−θ_target) + Ki·i + Kd·θ̇
#   → 速度环: err = 实测前进转速 − 目标转速;  θ_target = PID(err)
#              (走慢了 → err<0 → θ_target<0 → 前倾 → 加速前进 ✓)
#
# 摆的运动方程 (L = 摆长, a = 轮子的等效加速度)
#   θ'' = (g/L)·sin θ − a        a = L·ẍ
#
# 编码器 (固件: 300ms 滑动窗口, 44 脉冲/转, 减速比 gear)
#   实测转速 = 量化(窗口内平均转速)
#   量化台阶 = 60 / (44 × 窗口秒数 × gear)
#   ⚠ gear=30 (本机实测: 编码器在电机轴上, 减速箱 30:1) → 300ms 窗口下 0.15 rpm;
#     gear=1 时是 4.55 rpm (那个数已作废)。改 gear 必须同步改这里的 gear 参数。


class WalkSim(Sim):
    """会真的跑起来的车 (轮式倒立摆) + 串级控制 (速度环 → 目标倾角 → 平衡环)

    与定轴模型 (Sim) 的关系 —— **故意保持一致**, 这样"平衡环能站住"这件事
    在两个模型里是同一套数学, 差别只是: 定轴模型把车的平动丢掉了,
    这里把它补回来。

      定轴:  θ̈ = (g/L)·sinθ − a_w        (a_w = 轮加速度, 正 = 往 +x)
      行走:  θ̈ = (g/L)·sinθ − a_w        ← 一样 (θ 的动力学完全没变)
             ẍ  = L·a_w                  ← 车往轮子跑的方向平动
             轮速 (前进为正) = −ẋ·60/(2πr)

    也就是说: 定轴模型里的 "a_w" 本来就等于 ẍ/L (L = 质心高),
    行走模型只是把被丢掉的平动速度补回来。

    符号 (跟固件一致):
      θ > 0  = 车体往 +x 倾 (x = 车尾方向), pitch 同号
      前进为正 → 轮速 rpm_fwd = −ẋ·60/(2πr)

    ⚠ 植物模型 (摆长 L / k_motor / 电机滞后 / 硬件死区) 是**按 lumped 参数猜的**,
      没把减速箱的力矩放大和电机转速上限建进去 —— 齿轮比只影响**测速量化**
      (enc_res), 不影响动力学。结论用来做"相对比较"是可靠的,
      绝对数值要真机复核。

    编码器 (跟固件一致): **每 300ms 才更新一次**, 用这 300ms 里的累计脉冲
    算速度, 量化台阶 = 60/(44 × 0.3 × gear) = **0.15 rpm** (gear=30)。
    测速的滞后是速度环的头号敌人。
    """

    def __init__(self, kp=12.0, ki=0.5, kd=1.5, out_max=60.0,
                 spd_kp=0.06, spd_ki=0.01, spd_kd=0.0,
                 spdout_max=4.0, spd_deadband=1.0,
                 ramp_up=15.0, ramp_down=30.0,
                 gear=30.0, enc_win_s=0.3,
                 wheel_r=0.08, enc_res=None,
                 **kw):
        super().__init__(kp, ki, kd, out_max=out_max, **kw)
        self.spd_kp, self.spd_ki, self.spd_kd = spd_kp, spd_ki, spd_kd
        self.spdout_max = spdout_max      # 目标倾角上限 (°)
        self.spd_deadband = spd_deadband  # 速度误差死区 (rpm)
        self.ramp_up, self.ramp_down = ramp_up, ramp_down
        self.wheel_r = wheel_r            # 轮半径 (m)
        # 编码器量化台阶: rpm = 脉冲数 × 60 / (窗口秒数 × 44 × gear)
        # ⚠ gear 是固件参数表里的 gear (本机实测 30:1), 改了 gear 就要重算。
        #   gear=30 → 300ms 窗口 0.15 rpm; gear=1 → 4.55 rpm (已作废)。
        self.enc_res = enc_res if enc_res is not None else \
            60.0 / (44.0 * enc_win_s * gear)
        self.enc_win_s = enc_win_s        # 测速窗口 (s)
        # 速度环的积分上限: 跟固件一样 = (输出上限/2)/Ki
        self.spd_i_max = (spdout_max * 0.5 / spd_ki) if spd_ki > 0 else 0.0
        self._spd_prev = None

    def run_walk(self, target_rpm=24.0, seconds=12.0, t_start=1.0,
                 push_deg_s=0.0, push_at=3.0):
        """走一段路: t_start 时目标转速跳到 target_rpm"""
        theta = 0.0            # 倾角 (rad)
        rate = 0.0            # 角速度 (rad/s)
        i = 0.0               # 平衡环积分
        spd_i = 0.0           # 速度环积分
        rate_f = 0.0          # 陀螺低通后的角速度 (°/s)
        a_w = 0.0             # 轮加速度 (rad/s²), 正 = 往 +x
        v = 0.0               # 车速度 (m/s), 正 = 车尾方向
        pitch_est = None
        tgt = 0.0             # 目标前进转速 (rpm)
        theta_target = 0.0    # 速度环给出的目标倾角 (°)
        meas_rpm = 0.0        # 编码器测出来的前进转速 (rpm)
        enc_sum = 0.0         # 当前测速窗口里累计的 rpm·秒
        enc_t = 0.0           # 测速窗口已过时间 (s)
        pushed = False
        outs, thetas, spds, tgts = [], [], [], []
        tgt_leans, ests = [], []
        lean_steady = []
        lean_ripple = []
        fell = False

        for t in range(int(seconds / self.dt)):
            tsec = t * self.dt

            # ---- 指令: t_start 之后目标转速生效, 目标按斜坡逼近 ----
            cmd_rpm = target_rpm if tsec >= t_start else 0.0
            step = (self.ramp_up if cmd_rpm > tgt else self.ramp_down) * self.dt
            d = cmd_rpm - tgt
            tgt += step if abs(d) > step else d

            # ---- IMU: 陀螺积分 + 加速度角 + 互补滤波 (跟固件同款) ----
            raw_rate = math.degrees(rate)
            if pitch_est is None:
                pitch_est = self._accel_pitch(theta, 0.0)
            else:
                pitch_est += raw_rate * self.dt
                acc_pitch = self._accel_pitch(theta, a_w)
                tw = self._trust(theta, a_w)
                pitch_est += (1.0 - self.alpha) * tw * (acc_pitch - pitch_est)
            rate_f += self.gyro_lpf * (raw_rate - rate_f)

            # ---- 编码器: 300ms 窗口, 窗口结束才更新读数 (阶梯状) ----
            rpm_fwd_true = -v * 60.0 / (2.0 * math.pi * self.wheel_r)
            enc_sum += rpm_fwd_true * self.dt
            enc_t += self.dt
            if enc_t >= self.enc_win_s:
                avg = enc_sum / enc_t
                meas_rpm = round(avg / self.enc_res) * self.enc_res
                enc_sum = 0.0
                enc_t = 0.0

            # ---- 速度环: 误差 = 实测 − 目标 (前进为正) ----
            spd_err = meas_rpm - tgt
            if abs(spd_err) < self.spd_deadband:
                spd_err = 0.0
            if self._spd_prev is None:
                d_spd = 0.0
            else:
                d_spd = (spd_err - self._spd_prev) / self.dt
            self._spd_prev = spd_err
            spd_i_new = max(-self.spd_i_max,
                            min(self.spd_i_max, spd_i + spd_err * self.dt)) \
                if self.spd_i_max else spd_i + spd_err * self.dt
            out_now = self.spd_kp * spd_err + self.spd_ki * spd_i \
                + self.spd_kd * d_spd
            if not (out_now >= self.spdout_max and spd_err > 0) and \
               not (out_now <= -self.spdout_max and spd_err < 0):
                spd_i = spd_i_new
            theta_target = self.spd_kp * spd_err + self.spd_ki * spd_i \
                + self.spd_kd * d_spd
            theta_target = max(-self.spdout_max,
                               min(self.spdout_max, theta_target))

            # ---- 平衡环 (固件 pid_update_rate: D 项用角速度, 正号) ----
            err_deg = pitch_est - theta_target
            d_term = self.d_sign * rate_f
            i_new = max(-self.i_max, min(self.i_max, i + err_deg * self.dt)) \
                if self.i_max else i + err_deg * self.dt
            out_now = self.kp * err_deg + self.ki * i + self.kd * d_term
            if not (out_now >= self.out_max and err_deg > 0) and \
               not (out_now <= -self.out_max and err_deg < 0):
                i = i_new
            out = self.kp * err_deg + self.ki * i + self.kd * d_term
            out = max(-self.out_max, min(self.out_max, out))

            # ---- 输出 → 电机 → 轮加速度 → 车平动 + 车体摆动 ----
            duty = self._duty(out)
            a_cmd = self._accel_cmd(duty)
            alpha_m = self.dt / max(self.tau, 1e-6)
            a_w += (a_cmd - a_w) * min(alpha_m, 1.0)

            thdd = (G / self.L) * math.sin(theta) - a_w
            rate += thdd * self.dt
            theta += rate * self.dt
            v += self.L * a_w * self.dt          # ẍ = L·a_w

            if push_deg_s and not pushed and tsec >= push_at:
                rate += math.radians(push_deg_s)
                pushed = True

            outs.append(out)
            thetas.append(math.degrees(theta))
            spds.append(meas_rpm)
            tgts.append(tgt)
            tgt_leans.append(theta_target)
            ests.append(pitch_est)
            if tsec > seconds - 1.0:
                lean_steady.append(math.degrees(theta))
            if tsec > seconds - 3.0:
                lean_ripple.append(math.degrees(theta))

            if abs(math.degrees(theta)) > 45:      # 摔了
                fell = True
                break

        tail = thetas[-int(0.5 / self.dt):] or thetas[-1:]
        return {
            "fell": fell,
            "final_speed": spds[-1] if spds else 0.0,
            "target": tgt,
            "peak_angle": max(abs(t) for t in thetas) if thetas else 0.0,
            "tail_max": max(abs(t) for t in tail),
            "lean_steady": sum(lean_steady) / len(lean_steady) if lean_steady else 0.0,
            "lean_ripple": (max(lean_ripple) - min(lean_ripple)) if lean_ripple else 0.0,
            "out_max_used": max(abs(o) for o in outs) if outs else 0.0,
            "rms": math.sqrt(sum(o * o for o in outs) / max(1, len(outs))),
            "final": thetas[-1] if thetas else 0.0,
            "lean_cmd": tgt_leans[-1] if tgt_leans else 0.0,
            "trace": {"t": list(range(len(thetas))), "theta": thetas,
                      "tgt_lean": tgt_leans, "est": ests,
                      "out": outs, "meas": spds, "tgt": tgts},
        }


def fmt_walk(r):
    if r["fell"]:
        return "摔了!"
    return (f"末速 {r['final_speed']:6.1f} rpm (目标 {r['target']:5.1f})  "
            f"稳态倾角 {r['lean_steady']:+5.2f}°  倾角波动 {r['lean_ripple']:5.2f}°  "
            f"峰值角 {r['peak_angle']:5.2f}°  输出峰值 {r['out_max_used']:5.1f}%")


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
    ap.add_argument("--dz-fw", type=float, default=4.0, help="固件里配的死区补偿 (百分比)")
    ap.add_argument("--dz-true", type=float, default=12.0, help="电机硬件真实死区 (百分比)")
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
    ap.add_argument("--walk", action="store_true", help="行走仿真 (小车+速度环)")
    ap.add_argument("--walk-rpm", type=float, default=40.0, help="行走目标轮速 (rpm)")
    ap.add_argument("--spd-kp", type=float, default=0.06, help="速度环 Kp (°/rpm)")
    ap.add_argument("--spd-ki", type=float, default=0.01, help="速度环 Ki")
    ap.add_argument("--spd-kd", type=float, default=0.0, help="速度环 Kd")
    ap.add_argument("--walk-scan", action="store_true", help="扫描速度环增益")
    ap.add_argument("--walk-lean-max", type=float, default=4.0,
                    help="走路时允许的最大目标倾角 (°)")
    ap.add_argument("--walk-deadband", type=float, default=1.0,
                    help="速度误差死区 (rpm, 固件 BAL_SPD_ERR_DEADBAND = 1.0)")
    ap.add_argument("--walk-ramp", type=float, default=15.0,
                    help="目标轮速斜坡 (rpm/s)")
    ap.add_argument("--gear", type=float, default=30.0,
                    help="减速比 (固件参数表 gear; 编码器在电机轴上, 本机 30:1)")
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

    # ================= 行走仿真 =================
    if args.walk or args.walk_scan:
        wbase = dict(dz_fw=args.dz_fw, dz_true=args.dz_true, tau=args.tau,
                     gyro_lpf=args.gyro_lpf, alpha=args.alpha,
                     contam=contam, trust=trust, motor_sign=args.motor_sign,
                     kp=args.kp, ki=args.ki, kd=args.kd, out_max=args.out_max,
                     dz_mode=args.dz_mode, gear=args.gear)

        def wmk(**kw):
            d = dict(wbase)
            d.update(kw)
            return WalkSim(**d)

        print("=" * 92)
        print(f"行走仿真: 目标轮速 {args.walk_rpm:.0f} rpm, 速度环 "
              f"Kp={args.spd_kp} Ki={args.spd_ki} Kd={args.spd_kd}, "
              f"目标倾角上限 {args.walk_lean_max}°, 死区 {args.walk_deadband} rpm, "
              f"斜坡 {args.walk_ramp} rpm/s")
        print("=" * 92)
        for rpm in (20.0, 40.0, 60.0, 80.0):
            r = wmk(spd_kp=args.spd_kp, spd_ki=args.spd_ki,
                    spd_kd=args.spd_kd, spdout_max=args.walk_lean_max,
                    spd_deadband=args.walk_deadband,
                    ramp_up=args.walk_ramp, ramp_down=args.walk_ramp * 2
                    ).run_walk(target_rpm=rpm)
            print(f"  目标 {rpm:5.1f} rpm  {fmt_walk(r)}")

        # 对照组: 速度环方向取反 (如果实车跑反了, 就是这个符号错了)
        r = wmk(spd_kp=-args.spd_kp, spd_ki=-args.spd_ki,
                spd_kd=args.spd_kd, spdout_max=args.walk_lean_max,
                spd_deadband=args.walk_deadband,
                ramp_up=args.walk_ramp, ramp_down=args.walk_ramp * 2
                ).run_walk(target_rpm=args.walk_rpm)
        print(f"  [对照] 速度环符号取反  {fmt_walk(r)}")

        # 对照组: 走的时候被推一下
        r = wmk(spd_kp=args.spd_kp, spd_ki=args.spd_ki,
                spd_kd=args.spd_kd, spdout_max=args.walk_lean_max,
                spd_deadband=args.walk_deadband,
                ramp_up=args.walk_ramp, ramp_down=args.walk_ramp * 2
                ).run_walk(target_rpm=args.walk_rpm,
                           push_deg_s=25.0, push_at=4.0)
        print(f"  [对照] 走的时候被推 25°/s  {fmt_walk(r)}")
        print()

    if args.walk_scan:
        print("速度环增益扫描 (目标 40 rpm, 看能不能走到 + 会不会摔):")
        print(f"  {'spd_kp':>7} {'spd_ki':>7}  {'末速':>7}  {'稳态倾角':>9}  "
              f"{'倾角波动':>9}  {'峰值角':>7}")
        for skp in (0.02, 0.05, 0.10, 0.15, 0.25):
            for ski in (0.0, 0.01, 0.02, 0.05):
                r = WalkSim(spd_kp=skp, spd_ki=ski, spd_kd=0.0,
                            dz_fw=args.dz_fw, dz_true=args.dz_true,
                            tau=args.tau, gyro_lpf=args.gyro_lpf,
                            alpha=args.alpha, contam=contam, trust=trust,
                            motor_sign=args.motor_sign, dz_mode=args.dz_mode,
                            spdout_max=args.walk_lean_max,
                            spd_deadband=args.walk_deadband,
                            ramp_up=args.walk_ramp, ramp_down=args.walk_ramp * 2,
                            kp=args.kp, ki=args.ki, kd=args.kd,
                            out_max=args.out_max, gear=args.gear
                            ).run_walk(target_rpm=40.0)
                print(f"  {skp:7.2f} {ski:7.2f}  {r['final_speed']:7.1f}  "
                      f"{r['lean_steady']:9.2f}  {r['lean_ripple']:9.2f}  "
                      f"{r['peak_angle']:7.2f}"
                      + ("   摔了!" if r["fell"] else ""))
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
