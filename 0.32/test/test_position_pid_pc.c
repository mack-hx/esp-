/**
 * @file test_position_pid_pc.c
 * @brief 位置环离线单元测试 (PC 上用 gcc 直接跑, 不烧硬件)
 *
 * @note  覆盖:
 *   1) 死区软化 —— 误差连续, 输出不跳变
 *   2) 输出限幅 + 输出整形 (限速 / 一阶低通) —— 纠偏速度不能大过正常行走速度,
 *      而且不能一步跳到位 (0.32 起位置环输出加了限速 + 低通, 见 position_pid.h)
 *   3) 死区内积分衰减 —— 跟速度环同一个坑 (积分冻结 → 车永久偏移)
 *
 * @note  ⚠ 因为输出有整形 (限速 12 rpm/s + 低通 τ=0.2s), 单次 update() 的输出
 *        远小于稳态值, 所以测试都用 settle() 跑到稳态再断言。
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
 * @return 稳态输出 (rpm)
 * @note  输出有整形 (限速 + 低通), 必须跑够周期才到稳态
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
 * @brief 用例 1: 死区软化 —— 误差连续, 输出不跳变
 */
static void test_deadband_softening(void)
{
    printf("[1] 位置环死区软化: 过渡带内误差线性压缩\n");

    /* 纯 P (ki=0), 这样期望值就是 kp × 压缩后的误差 */
    g_pos_kp = 1.0f;
    g_pos_ki = 0.0f;

    position_pid_init();
    float out_in = settle(0.5f, 300);
    CHECK(fabsf(out_in) < 1e-3f,
          "死区内 (err=0.5cm) 输出=%.6f (期望 0)", out_in);

    position_pid_init();
    float out_soft = settle(1.5f, 300);
    float expect = 1.0f * 0.5f;   /* kp × 压缩后误差 ((1.5-1.0)/1.0 = 0.5) */
    CHECK(fabsf(out_soft - expect) < 1e-2f,
          "过渡带内 (err=1.5cm) 输出=%.5f (期望 ≈%.5f)", out_soft, expect);

    position_pid_init();
    float out_full = settle(5.0f, 300);
    CHECK(fabsf(out_full - 5.0f) < 0.05f,
          "过渡带外 (err=5cm) 输出=%.5f (期望 ≈5.0 = kp×err)", out_full);

    position_pid_init();
    float out_neg = settle(-1.5f, 300);
    CHECK(fabsf(out_neg + 0.5f) < 1e-2f,
          "负方向 (err=-1.5cm) 输出=%.5f (期望 ≈-0.5)", out_neg);
}

/**
 * @brief 用例 2: 输出限幅 + 输出整形 (限速 / 低通)
 */
static void test_output_clamp(void)
{
    printf("[2] 位置环输出限幅: 不超过 ±%.0f rpm\n", POS_PID_OUT_MAX);

    g_pos_kp = 1.0f;
    g_pos_ki = 0.0f;

    position_pid_init();
    float out = settle(1000.0f, 300);
    CHECK(fabsf(out) <= POS_PID_OUT_MAX + 1e-3f,
          "巨大误差下输出=%.3f rpm (期望 |x|<=%.0f)", out, POS_PID_OUT_MAX);

    /* 整形: 误差一下变大, 输出不能一步跳到 20 rpm,
     * 否则目标轮速跳变 → 目标倾角跳变 → 平衡环猛纠 → 车前后摆 */
    position_pid_init();
    float first = position_pid_update(0.01f, 1000.0f);
    float max_step = POS_PID_OUT_SLEW * 0.01f;   /* 一个周期允许的最大变化 */
    CHECK(fabsf(first) <= max_step + 1e-3f,
          "误差突变时第一拍输出=%.4f rpm (期望 ≤%.4f = 限速 × 周期)",
          first, max_step);
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

    float out_big = settle(20.0f, 200);
    CHECK(out_big > 19.0f,
          "偏 20cm 时输出=%.3f rpm (期望 >19, 纠偏力度足够)", out_big);

    /* 回正 (进死区) 后, 积分要退干净, 输出不能一直挂着 */
    float out_at_zero = settle(0.5f, 200);
    CHECK(fabsf(out_at_zero) < 5.0f,
          "回正 2 秒后输出=%.4f rpm (期望 <5, 明显小于偏的时候 %.3f)",
          out_at_zero, out_big);

    float out_after = settle(0.5f, 1000);
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
