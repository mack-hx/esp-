#!/usr/bin/env python3
"""
pos_sim.py — 位置保持环离线仿真 (复用 bal_sim.WalkSim 的车体/电机/IMU 模型)

控制结构与固件 balance.c 一致 (指令速度为 0 时位置环独占):
    target_pitch = −position_pid(锚点−当前位置, 里程计速度)      (pitch 前倾为负)
    平衡环: out = Kp·(pitch − target_pitch) + Ki·∫ + Kd·gyro

场景:
    push   在 1 s 时给车体一个角速度冲量 (模拟手推), 看最大位移 / 回位 / 是否摔
    bias   pitch 带固定零偏 (模拟配平误差 / 地面略斜), 看站住后会不会一直漂

用法:
    python3 test/pos_sim.py                 # 当前固件参数跑全部场景
    python3 test/pos_sim.py --kp 0.5 --kd 0.10 --ki 0.1
"""

import argparse
import math

from bal_sim import WalkSim, G

LEAN_MAX = 3.0        # POS_PID_LEAN_MAX (°)
DEADBAND = 0.1        # POS_PID_ERR_DEADBAND (cm)
SOFT_BAND = 0.2       # POS_PID_ERR_SOFT_BAND (cm)
I_MAX = 20.0          # POS_PID_I_MAX (cm·s)
I_TAU = 3.0           # POS_PID_I_DECAY_TAU (s)
WALK_PITCH_MAX = 4.0  # SPD_PID_WALK_PITCH_MAX (°)


def soft_deadband(err, db, sb):
    a = abs(err)
    if a <= db:
        return 0.0
    eff = sb * ((a - db) / sb) ** 2 if (sb > 0 and a < db + sb) else a - db
    return math.copysign(eff, err)


class PosCtl:
    """与 position_pid.c 同算法 (可通过子类替换做对比)"""

    def __init__(self, kp, kd, ki):
        self.kp, self.kd, self.ki = kp, kd, ki
        self.i = 0.0

    def update(self, dt, err, v):
        if abs(err) <= DEADBAND:
            e = 0.0
            self.i *= math.exp(-dt / I_TAU)
        else:
            e = soft_deadband(err, DEADBAND, SOFT_BAND)
        raw = self.kp * e + self.ki * self.i - self.kd * v
        sat = (raw >= 0.9 * LEAN_MAX and err > 0) or \
              (raw <= -0.9 * LEAN_MAX and err < 0)
        if not sat:
            self.i = max(-I_MAX, min(I_MAX, self.i + e * dt))
            raw = self.kp * e + self.ki * self.i - self.kd * v
        return LEAN_MAX * math.tanh(raw / LEAN_MAX)


def run(ctl, push_deg_s=0.0, bias_deg=0.0, seconds=20.0, push_at=1.0,
        sim_kw=None, anchor_cm=0.0, shove_cmps=0.0):
    """返回 dict: 最大位移 / 终点位置 / 稳定时间 / 是否摔 / 轨迹"""
    s = WalkSim(**(sim_kw or {}))
    dt = s.dt
    theta = 0.0
    rate = 0.0
    i_bal = 0.0
    rate_f = 0.0
    a_w = 0.0
    x = 0.0                    # 车位置 (m), +x = 车尾
    v = 0.0
    pitch_est = None
    pos_buf = [0.0] * 6        # 里程计速度: 60 ms 位移窗口
    pushed = False
    fell = False
    tr_x, tr_th, tr_lean = [], [], []

    for k in range(int(seconds / dt)):
        t = k * dt
        if push_deg_s and not pushed and t >= push_at:
            rate += math.radians(push_deg_s)
            pushed = True
        if shove_cmps and not pushed and t >= push_at:
            v += -shove_cmps / 100.0       # 车被推着往前跑 (+x = 车尾, 前进为负)
            pushed = True

        raw_rate = math.degrees(rate)
        if pitch_est is None:
            pitch_est = s._accel_pitch(theta, 0.0)
        else:
            pitch_est += raw_rate * dt
            acc_pitch = s._accel_pitch(theta, a_w)
            tw = s._trust(theta, a_w)
            pitch_est += (1.0 - s.alpha) * tw * (acc_pitch - pitch_est)
        rate_f += s.gyro_lpf * (raw_rate - rate_f)
        pitch_meas = pitch_est + bias_deg

        pos_fwd = -x * 100.0                         # cm, 前进为正
        pos_buf = pos_buf[1:] + [pos_fwd]
        v_fwd = (pos_buf[-1] - pos_buf[0]) / (5 * dt) if k >= 5 else 0.0

        lean = ctl.update(dt, anchor_cm - pos_fwd, v_fwd)
        tgt = max(-WALK_PITCH_MAX, min(WALK_PITCH_MAX, -lean))

        err = pitch_meas - tgt
        i_new = max(-s.i_max, min(s.i_max, i_bal + err * dt))
        out_now = s.kp * err + s.ki * i_bal + s.kd * rate_f
        if not (out_now >= s.out_max and err > 0) and \
           not (out_now <= -s.out_max and err < 0):
            i_bal = i_new
        out = s.kp * err + s.ki * i_bal + s.kd * rate_f
        out = max(-s.out_max, min(s.out_max, out))

        a_cmd = s._accel_cmd(s._duty(out))
        a_w += (a_cmd - a_w) * min(dt / max(s.tau, 1e-6), 1.0)
        thdd = (G / s.L) * math.sin(theta) - a_w
        rate += thdd * dt
        theta += rate * dt
        v += s.L * a_w * dt
        x += v * dt

        tr_x.append(pos_fwd)
        tr_th.append(math.degrees(theta))
        tr_lean.append(lean)
        if abs(math.degrees(theta)) > 45:
            fell = True
            break

    n = len(tr_x)
    peak = max(tr_x, key=abs) if n else 0.0
    # 稳定时间: 最后一次 |位置| > 0.5 cm 的时刻 (相对推的时刻)
    last_out = 0
    for i, p in enumerate(tr_x):
        if abs(p - anchor_cm) > 0.5:
            last_out = i
    settle = max(0.0, last_out * dt - push_at) if not fell else float("nan")
    # 回过头: 反向最大位移
    sgn = 1.0 if peak >= 0 else -1.0
    over = max((-sgn * p for p in tr_x[tr_x.index(peak):]), default=0.0)
    return {"fell": fell, "peak": peak, "over": max(0.0, over),
            "final": tr_x[-1] if n else 0.0, "settle": settle,
            "trace": {"x": tr_x, "th": tr_th, "lean": tr_lean}}


def fmt(r):
    if r["fell"]:
        return "摔了!"
    return (f"最大位移 {r['peak']:+7.1f} cm  回过头 {r['over']:5.1f} cm  "
            f"稳定用时 {r['settle']:5.1f} s  末位置 {r['final']:+6.2f} cm")


SCENARIOS = [
    ("轻推 15°/s", dict(push_deg_s=15.0)),
    ("中推 40°/s", dict(push_deg_s=40.0)),
    ("重推 80°/s", dict(push_deg_s=80.0)),
    ("零偏 +0.5°", dict(bias_deg=0.5, seconds=30.0)),
    ("零偏 +1.0°", dict(bias_deg=1.0, seconds=30.0)),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kp", type=float, default=0.5)
    ap.add_argument("--kd", type=float, default=0.10)
    ap.add_argument("--ki", type=float, default=0.1)
    ap.add_argument("--bal-kp", type=float, default=12.0)
    ap.add_argument("--bal-ki", type=float, default=0.5)
    ap.add_argument("--bal-kd", type=float, default=0.4)
    ap.add_argument("--dz-true", type=float, default=12.0)
    a = ap.parse_args()
    kw = dict(kp=a.bal_kp, ki=a.bal_ki, kd=a.bal_kd, dz_fw=4.0,
              dz_true=a.dz_true, dz_mode="add", gyro_lpf=0.8, alpha=0.997,
              contam=1, tau=0.02)
    print(f"位置环 kp={a.kp} kd={a.kd} ki={a.ki}  |  平衡环 {a.bal_kp}/{a.bal_ki}/{a.bal_kd}")
    for name, sc in SCENARIOS:
        r = run(PosCtl(a.kp, a.kd, a.ki), sim_kw=kw, **sc)
        print(f"  {name:<12s} {fmt(r)}")


if __name__ == "__main__":
    main()
