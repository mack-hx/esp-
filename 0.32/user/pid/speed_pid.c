/**
 * @file speed_pid.c
 * @brief 速度环 (外环) 实现: 轮速误差 → 目标倾角 (°)
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-27
 */

#include "speed_pid.h"
#include "pid.h"
#include "param.h"
#include <math.h>

/* 模块内部唯一的 PID 实例 (速度环) */
static bal_pid_t s_pid;

void speed_pid_load_params(void)
{
    float spdout_max = param_get_float(PARAM_SPEED_OUT_MAX);

    float kp = param_get_float(PARAM_PID_SPD_KP);
    float ki = param_get_float(PARAM_PID_SPD_KI);
    float kd = param_get_float(PARAM_PID_SPD_KD);

    /* 速度环: 输出 = 目标倾角 (°)。
     * ⚠ 限幅夹到 SPD_PID_WALK_PITCH_MAX: 走路时车最多前倾这么多度,
     *   目标轮速定太高时宁可走慢, 也不能倾到摔。
     *   (夹在 PID 内部而不是外面: 否则积分会继续往 40° 攒,
     *    误差反向时要很久才退得回来 → 车会先猛冲再回不来) */
    float walk_max = spdout_max;
    if (walk_max > SPD_PID_WALK_PITCH_MAX) walk_max = SPD_PID_WALK_PITCH_MAX;
    if (walk_max < 0.0f) walk_max = 0.0f;

    /* 速度环增益为 0 时用内置默认值 (防止旧版 App 把参数写回 0,
     * 结果方向键完全没反应 —— 2026-10-26 那次"恢复默认"事故的教训) */
    float kp_eff = (kp > 0.001f) ? kp : SPD_PID_KP_DEFAULT;
    float ki_eff = (ki > 0.001f) ? ki : SPD_PID_KI_DEFAULT;
    float kd_eff = (kd > 0.001f) ? kd : 0.0f;

    /* ⚠ 只改系数, 不清积分 (每周期都调, 用 pid_config 会让积分永远为 0) */
    pid_set_gains(&s_pid,
                  kp_eff, ki_eff, kd_eff,
                  pid_i_max(ki_eff, walk_max),
                  walk_max);        /* 输出 = 目标倾角 (°) */
}

void speed_pid_reset(void)
{
    pid_reset(&s_pid);
}

void speed_pid_init(void)
{
    speed_pid_load_params();
    speed_pid_reset();
}

float speed_pid_update(float dt, float speed, float target)
{
    float err = speed - target;

    /* ⚠ 误差死区 + 软化过渡带
     *
     * 死区: 编码器量化台阶是 0.15 rpm (gear=30, 300ms 窗口),
     *   不设死区的话, 车会为了追一个量化噪声一直歪着 (慢慢往一边跑)。
     *
     * 软化: 死区不是"开关", 而是一段过渡带 (SPD_PID_ERR_SOFT_BAND)。
     *   |err| <= 死区             → 0
     *   死区 < |err| < 死区+过渡带  → 线性压缩, 0 → 1
     *   |err| >= 死区+过渡带      → 完整保留
     *   原因: 硬归零会让目标倾角在死区边缘跳变 (实测轮速跨一个量化台阶
     *   0.90↔1.05 rpm 时, 目标倾角每 300ms 跳一次) → 平衡环反复纠偏 → 抖动。
     */
    float abs_err = fabsf(err);
    if (abs_err <= SPD_PID_ERR_DEADBAND) {
        err = 0.0f;

        /* ⚠ 死区内让积分**衰减**而不是冻结 (2026-10-27 修"车一直往前跑")
         *
         * 死区把 err 归零后传进 PID, 积分变成 i += 0*dt → 冻结:
         * 既不增长也不减少。走路时攒下的积分残留会**永久冻结**在非零值,
         * 目标倾角恒定非零 → 车持续往前跑。
         * 积分只在换指令/摔倒/上电这些边界才清, 平衡期间永不清理。
         *
         * 这里用指数衰减 (时间常数 SPD_PID_I_DECAY_TAU) 让积分平滑滑向 0:
         *   - 不直接清零: 直接清零会让目标倾角瞬间跳变 → 抖动
         *   - 指数衰减: 输出连续, 不抖
         *
         * 死区外不衰减: 那时积分本来就该正常累积来消除稳态误差。 */
        pid_decay_integral(&s_pid, dt, SPD_PID_I_DECAY_TAU);
    } else if (abs_err < SPD_PID_ERR_DEADBAND + SPD_PID_ERR_SOFT_BAND) {
        /* 过渡带: 按比例线性压缩, 保证误差连续 (消除边界跳变) */
        float t = (abs_err - SPD_PID_ERR_DEADBAND) / SPD_PID_ERR_SOFT_BAND;
        err = (err > 0.0f) ? t : -t;
    }

    return pid_update(&s_pid, err, dt);
}
