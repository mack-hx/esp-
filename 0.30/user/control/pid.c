/**
 * @file pid.c
 * @brief 通用 PID 控制器实现 (位置式)
 * @author ESP32 机械狗项目
 * @date 2026-10-04
 */

#include "pid.h"

/**
 * @brief 把值限制到 [-limit, +limit]
 * @param v      原值
 * @param limit 限幅绝对值 (<=0 表示不限幅)
 * @return 限幅后的值
 */
static float clamp_sym(float v, float limit)
{
    if (limit <= 0.0f) return v;
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

/**
 * @brief 判断"输出已饱和且误差还在往同一方向推" → 此时不能再积分
 * @param pid   PID 结构指针
 * @param out   当前完整输出 (含积分项)
 * @param err   当前误差
 * @return true=本周期应冻结积分
 *
 * @note  抗积分饱和 (anti-windup) 的标准做法:
 *   输出顶在限幅上时, 误差继续同向积分, 累积值 i 会越攒越大;
 *   等误差反向, 系统要先把这段"退积分"走完才能响应 → 输出长时间卡在
 *   限幅上 (车表现为一直猛冲 / 回不来)。
 *   实测表现 (2026-10-26): 车先站住, 站一会儿后积分慢慢顶死,
 *   然后开始高频抖动 —— 因为积分项一直在满输出附近来回横跳。
 */
static bool pid_windup_blocked(const bal_pid_t *pid, float out, float err)
{
    if (pid->out_max <= 0.0f) return false;   /* 输出不限幅 → 不需要抗饱和 */
    if (out >=  pid->out_max && err > 0.0f) return true;   /* 顶上限, 误差还在往上推 */
    if (out <= -pid->out_max && err < 0.0f) return true;   /* 顶下限, 误差还在往下推 */
    return false;
}

void pid_set_gains(bal_pid_t *pid, float kp, float ki, float kd,
                   float i_max, float out_max)
{
    pid->kp      = kp;
    pid->ki      = ki;
    pid->kd      = kd;
    pid->i_max   = i_max;
    pid->out_max = out_max;
    /* ⚠ 不碰 i / prev_err / primed: 运行中改参数不能把积分清掉,
     *   否则每次改参数都会让积分项重新从 0 累积, 输出会出现台阶。 */
}

void pid_config(bal_pid_t *pid, float kp, float ki, float kd,
                float i_max, float out_max)
{
    pid_set_gains(pid, kp, ki, kd, i_max, out_max);
    pid_reset(pid);
}

void pid_reset(bal_pid_t *pid)
{
    pid->i        = 0.0f;
    pid->prev_err = 0.0f;
    pid->primed   = false;
}

float pid_update(bal_pid_t *pid, float err, float dt)
{
    if (dt <= 0.0f) return 0.0f;

    /* --- 微分项 ---
     * 首次调用时 prev_err 还是 0, 直接拿它做微分会算出一个假的大跳变,
     * 所以 primed=false 时只记录不参与运算。 */
    float d = 0.0f;
    if (pid->primed) {
        d = (err - pid->prev_err) / dt;
    }
    pid->prev_err = err;
    pid->primed   = true;

    /* --- 积分项 (带累积限幅, 防积分饱和) --- */
    float i_new = clamp_sym(pid->i + err * dt, pid->i_max);

    /* 抗积分饱和: 输出已顶到限幅且误差还在同向 → 冻结积分 */
    float out = pid->kp * err + pid->ki * pid->i + pid->kd * d;
    if (!pid_windup_blocked(pid, out, err)) {
        pid->i = i_new;
    }

    /* --- 输出 --- */
    out = pid->kp * err + pid->ki * pid->i + pid->kd * d;
    return clamp_sym(out, pid->out_max);
}


/**
 * @brief D 项用角速度反馈的 PID (平衡环专用)
 * @note  与 pid_update() 的唯一区别: D 项直接取 rate, 不对误差求导。
 *        详细理由见 pid.h 里 pid_update_rate() 的文档。
 *
 * @note  ⚠ D 项符号 (2026-10-25 修正): 取**正号**, 与 err/rate 同号。
 *        车体前倾时轮子要往倒的方向追才接得住, rate 越大追得越凶 = 阻尼。
 *        取负号 = 负阻尼 (越晃越大)。详见下面实现处的推导。
 */
float pid_update_rate(bal_pid_t *pid, float err, float rate, float dt)
{
    if (dt <= 0.0f) return 0.0f;

    /* D 项 = 角速度本身。首次调用跳过, 防止上电瞬间的假阻尼冲量。 */
    float d = pid->primed ? rate : 0.0f;

    pid->prev_err = err;
    pid->primed   = true;

    /* 抗积分饱和: 输出已顶到限幅且误差还在同向 → 冻结积分
     * (说明见 pid_windup_blocked 的注释) */
    float i_new = clamp_sym(pid->i + err * dt, pid->i_max);
    float out_now = pid->kp * err + pid->ki * pid->i + pid->kd * d;
    if (!pid_windup_blocked(pid, out_now, err)) {
        pid->i = i_new;
    }

    /* ⚠ D 项与 P 项**同号** (2026-10-25 修正, 之前是负号 = 负阻尼, 车必然站不住)
     *
     * 物理关系 (轮 + 倒立摆), 与 "pitch 正负代表哪边" 的约定无关:
     *   设 θ = 车体偏离竖直的角度 (往倒的方向为正), 轮加速度 a。
     *   车体前倾时轮子必须**往倒的方向加速**才能把重心甩回轮轴上方,
     *   即 a = K1·θ + K2·θ̇, 闭环稳定要求 **K2 > 0**。
     *   代入本函数 (out = kp·err + ki·i + kd·rate, err 与 rate 都与 θ 同号):
     *       a = k·(kp·θ + kd·θ̇)   →  K2 = k·kd > 0  ✓ 阻尼正确
     *   若 D 项取负号 → K2 < 0 → 负阻尼: 越晃越大, 必然摔倒。
     *
     * 直觉: 往前倒得越快, 轮子要追得**越凶** (前冲才接得住);
     *       "电机往回收" 是在帮着车继续倒, 不是阻尼。
     *
     * ⚠ out 正的方向 = "轮子往倒的方向追", 这件事由 balance.h 的
     *   BAL_MOTOR_SIGN 决定 (2026-10-25 实测复核为 **+1**)。
     *   本函数只负责: err 与 rate 同号, 且 D 项与 P 项同号。
     *
     * rate 必须与 err **同号** (本工程: 前倾时两者都是负数), 见 pid.h 的说明。 */
    float out = pid->kp * err + pid->ki * pid->i + pid->kd * d;
    return clamp_sym(out, pid->out_max);
}
