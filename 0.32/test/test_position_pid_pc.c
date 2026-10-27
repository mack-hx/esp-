/**
 * @file test_position_pid_pc.c
 * @brief 位置环离线单元测试 (PC 上用 gcc 直接跑, 不烧硬件)
 *
 * @note  覆盖:
 *   1) 死区软化 —— 误差连续, 输出不跳变
 *   2) 输出限幅 —— 纠偏速度不能大过正常行走速度
 *   3) 死区内积分衰减 —— 跟速度环同一个坑 (积分冻结 → 车永久偏移)
 *
 * 编译运行:
 *   cd 0.32
 *   gcc -std=c99 -Wall -Wextra test/test_position_pid_pc.c \
 *       user/pid/position_pid.c user/pid/pid.c \
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

/* ========== param 桩 (与 param.c 出厂默认值一致) ========== */
float param_get_float(param_id_t id)
{
    switch (id) {
    case PARAM_POS_KP:        return 1.0f;
    case PARAM_POS_KI:        return 0.05f;
    case PARAM_ENCODER_GEAR:  return 30.0f;
    case PARAM_WHEEL_DIA:     return 6.5f;
    default:                  return 0.0f;
    }
}

/**
 * @brief 用例 1: 死区软化 —— 误差连续, 输出不跳变
 */
static void test_deadband_softening(void)
{
    printf("[1] 位置环死区软化: 过渡带内误差线性压缩\n");

    position_pid_init();

    float out_in = position_pid_update(0.01f, 0.5f);
    CHECK(fabsf(out_in) < 1e-6f,
          "死区内 (err=0.5cm) 输出=%.6f (期望 0)", out_in);

    position_pid_init();
    float out_soft = position_pid_update(0.01f, 1.5f);
    float expect = 1.0f * 0.5f;   /* kp × 压缩后误差 ((1.5-1.0)/1.0 = 0.5) */
    /* 容差 1e-3: ki=0.05 的积分项会贡献一点点 (约 0.00025) */
    CHECK(fabsf(out_soft - expect) < 1e-3f,
          "过渡带内 (err=1.5cm) 输出=%.5f (期望 ≈%.5f)", out_soft, expect);

    position_pid_init();
    float out_full = position_pid_update(0.01f, 5.0f);
    CHECK(fabsf(out_full - 5.0f) < 0.01f,
          "过渡带外 (err=5cm) 输出=%.5f (期望 ≈5.0 = kp×err)", out_full);

    position_pid_init();
    float out_neg = position_pid_update(0.01f, -1.5f);
    CHECK(fabsf(out_neg + 0.5f) < 1e-3f,
          "负方向 (err=-1.5cm) 输出=%.5f (期望 ≈-0.5)", out_neg);
}

/**
 * @brief 用例 2: 输出限幅 (纠偏速度不能大过正常行走速度)
 */
static void test_output_clamp(void)
{
    printf("[2] 位置环输出限幅: 不超过 ±%.0f rpm\n", POS_PID_OUT_MAX);

    position_pid_init();
    float out = position_pid_update(0.01f, 1000.0f);
    CHECK(fabsf(out) <= POS_PID_OUT_MAX + 1e-4f,
          "巨大误差下输出=%.3f rpm (期望 |x|<=%.0f)", out, POS_PID_OUT_MAX);
}

/**
 * @brief 用例 3: 死区内积分衰减 (防止车一直往一个方向偏)
 */
static void test_integral_decay_in_deadband(void)
{
    printf("[3] 死区内积分衰减: 残留积分退干净, 不会永久偏移\n");

    position_pid_init();

    for (int i = 0; i < 200; i++) {
        position_pid_update(0.01f, 20.0f);
    }
    float out_big = position_pid_update(0.01f, 20.0f);
    CHECK(out_big > 19.0f,
          "偏 20cm 时输出=%.3f rpm (期望 >19, 纠偏力度足够)", out_big);

    float out_at_zero = position_pid_update(0.01f, 0.5f);
    CHECK(fabsf(out_at_zero) < fabsf(out_big),
          "回正瞬间输出=%.4f (小于偏的时候 %.3f), 积分在退",
          out_at_zero, out_big);

    for (int i = 0; i < 1000; i++) {
        position_pid_update(0.01f, 0.5f);
    }
    float out_after = position_pid_update(0.01f, 0.5f);
    CHECK(fabsf(out_after) < 0.5f,
          "站 10 秒后输出=%.4f rpm (期望接近 0)", out_after);
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
