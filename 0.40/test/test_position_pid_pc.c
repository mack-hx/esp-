/**
 * @file test_position_pid_pc.c
 * @brief 位置环离线单元测试 (PC 上用 gcc 直接跑, 不烧硬件)
 *
 * @note  覆盖:
 *   1) 死区 + 软化过渡带 —— 死区内不要求移动, 过渡带内误差线性压缩
 *   2) 输出限幅 —— 目标速度不超过 POS_PID_V_MAX (cm/s)
 *   3) 死区内积分衰减 —— 跟速度环同一个坑 (积分冻结 → 车永久偏移)
 *   4) 输出有限性 —— 极端输入不出 NaN/Inf
 *
 * @note  位置环输出的是**目标速度 (cm/s)**, 不是倾角:
 *        位置误差 (cm) → 目标速度 (cm/s) → 锁位速度环 → 目标倾角 (°) → 平衡环。
 *
 * 编译运行:
 *   cd 0.40
 *   gcc -std=c99 -Wall -Wextra -DPARAM_OFFLINE_TEST \
 *       test/test_position_pid_pc.c user/pid/position_pid.c user/pid/pid.c \
 *       -o /tmp/position_pid_test -I test -I user/pid -I user/param -lm
 *   /tmp/position_pid_test
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "position_pid.h"
#include "param.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, fmt, ...)                                          \
    do {                                                                \
        if (cond) {                                                     \
            g_pass++;                                                   \
            printf("  [PASS] " fmt "\n", ##__VA_ARGS__);                \
        } else {                                                        \
            g_fail++;                                                   \
            printf("  [FAIL] " fmt "\n", ##__VA_ARGS__);                \
        }                                                               \
    } while (0)

/* ========== param 桩 ==========
 * 位置环的增益用全局变量模拟参数表, 测试可以随时改 (改完要调
 * position_pid_load_params() 才会生效)。 */
static float g_pos_kp = 1.0f;   /* 测试用比例增益 (默认 1.0, 期望值好算) */
static float g_pos_ki = 0.0f;   /* 默认纯 P (跟真机默认值一致) */

float param_get_float(param_id_t id)
{
    switch (id) {
    case PARAM_POS_KP:        return g_pos_kp;
    case PARAM_POS_KI:        return g_pos_ki;
    case PARAM_ENCODER_GEAR:  return 30.0f;
    case PARAM_WHEEL_DIA:     return 6.5f;
    default:                  return 0.0f;
    }
}

/**
 * @brief 固定误差跑 n 个周期, 返回最后一次的输出
 * @param err 位置误差 (cm)
 * @param n   周期数 (每个周期 10 ms)
 * @return 输出 (cm/s)
 */
static float settle(float err, int n)
{
    float out = 0.0f;
    for (int i = 0; i < n; i++) {
        out = position_pid_update(0.01f, err);
    }
    return out;
}

/**
 * @brief 用例 1: 死区 + 软化过渡带
 */
static void test_deadband_softening(void)
{
    printf("[1] 位置环死区软化: 死区内不动, 过渡带内误差线性压缩\n");

    /* 纯 P (ki=0), 这样期望值就是 kp × 压缩后的误差 */
    g_pos_kp = 1.0f;
    g_pos_ki = 0.0f;

    position_pid_init();
    float out_in = settle(0.2f, 100);
    CHECK(fabsf(out_in) < 1e-3f,
          "死区内 (err=0.2cm) 输出=%.6f cm/s (期望 0)", out_in);

    /* 过渡带: 压缩后误差 = (|err| − 死区) / 过渡带 */
    position_pid_init();
    float out_soft = settle(0.5f, 100);
    float expect = (0.5f - POS_PID_ERR_DEADBAND) / POS_PID_ERR_SOFT_BAND;
    CHECK(fabsf(out_soft - expect) < 1e-3f,
          "过渡带内 (err=0.5cm) 输出=%.5f cm/s (期望 ≈%.5f)", out_soft, expect);

    position_pid_init();
    float out_neg = settle(-0.5f, 100);
    CHECK(fabsf(out_neg + expect) < 1e-3f,
          "负方向 (err=-0.5cm) 输出=%.5f cm/s (期望 ≈-%.5f)", out_neg, expect);

    /* 过渡带之外: 误差原样进 PID */
    position_pid_init();
    float out_full = settle(5.0f, 100);
    CHECK(fabsf(out_full - 5.0f) < 0.05f,
          "过渡带外 (err=5cm) 输出=%.5f cm/s (期望 ≈5.0 = kp×err)", out_full);

    position_pid_init();
    float out_full_neg = settle(-5.0f, 100);
    CHECK(fabsf(out_full_neg + 5.0f) < 0.05f,
          "负方向 (err=-5cm) 输出=%.5f cm/s (期望 ≈-5.0)", out_full_neg);

    /* 死区边界: 误差从小到大扫过去, 输出不能跳一大截 */
    position_pid_init();
    float below = settle(POS_PID_ERR_DEADBAND - 0.01f, 100);
    position_pid_init();
    float above = settle(POS_PID_ERR_DEADBAND + 0.01f, 100);
    CHECK(fabsf(above - below) < 0.05f,
          "死区边界输出应连续 (%.5f → %.5f cm/s)", below, above);
}

/**
 * @brief 用例 2: 输出限幅 (安全阀)
 */
static void test_output_clamp(void)
{
    printf("[2] 位置环输出限幅: 目标速度不超过 ±%.1f cm/s\n", POS_PID_V_MAX);

    g_pos_kp = 1.0f;
    g_pos_ki = 0.0f;

    position_pid_init();
    float out = settle(1000.0f, 100);
    CHECK(fabsf(out) <= POS_PID_V_MAX + 1e-3f,
          "巨大误差下输出=%.3f cm/s (期望 |x|<=%.1f)", out, POS_PID_V_MAX);

    position_pid_init();
    float out_neg = settle(-1000.0f, 100);
    CHECK(out_neg >= -POS_PID_V_MAX - 1e-3f,
          "巨大负误差下输出=%.3f cm/s (期望 |x|<=%.1f)", out_neg, POS_PID_V_MAX);

    /* 限幅不改变符号 */
    CHECK(out > 0.0f, "正向误差输出应为正 (实际 %.3f)", out);
    CHECK(out_neg < 0.0f, "负向误差输出应为负 (实际 %.3f)", out_neg);
}

/**
 * @brief 用例 3: 死区内积分衰减 (防止车一直往一个方向偏)
 */
static void test_integral_decay_in_deadband(void)
{
    printf("[3] 死区内积分衰减: 残留积分退干净, 不会永久偏移\n");

    /* 这里特意用带积分的配置 (真机默认 ki=0, 但积分衰减的坑必须测) */
    g_pos_kp = 1.0f;
    g_pos_ki = 0.05f;

    position_pid_init();

    /* 过渡带内的误差: 输出不撞限幅, 积分能正常累积 */
    float out_big = settle(0.5f, 200);
    CHECK(out_big > 0.5f,
          "偏 0.5cm 时输出=%.3f cm/s (期望 >0.5, 积分在工作)", out_big);

    /* 回正 (进死区) 后, 积分要退干净, 输出不能一直挂着 */
    float out_at_zero = settle(0.1f, 200);
    CHECK(fabsf(out_at_zero) < fabsf(out_big),
          "回正 2 秒后输出=%.4f cm/s (期望明显小于偏的时候 %.3f)",
          out_at_zero, out_big);

    float out_after = settle(0.1f, 1000);
    CHECK(fabsf(out_after) < 0.01f,
          "站 10 秒后输出=%.6f cm/s (期望接近 0)", out_after);
}

/**
 * @brief 用例 4: 输出必须是有限值 (不能出 NaN/Inf)
 */
static void test_finite_output(void)
{
    printf("[4] 输出有限性: 极端输入也不能出 NaN/Inf\n");

    position_pid_init();
    float out = position_pid_update(0.01f, 100.0f);
    CHECK(!isnan(out) && !isinf(out),
          "输出应为有限值 (实际 %.3f)", out);
}

int main(void)
{
    printf("==== 位置环 (position_pid) 离线验证 ====\n\n");

    test_deadband_softening();
    test_output_clamp();
    test_integral_decay_in_deadband();
    test_finite_output();

    printf("\n==== %d/%d 通过, %d 失败 ====\n",
           g_pass + g_fail, g_pass + g_fail, g_fail);
    return g_fail == 0 ? 0 : 1;
}
