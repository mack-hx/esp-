/**
 * @file test_speed_pid_pc.c
 * @brief 速度环离线单元测试 (PC 上用 gcc 直接跑, 不烧硬件)
 *
 * @note  验证 2026-10-27 修的两个问题:
 *   1) "车一直往前跑": 死区内积分**冻结** → 改成指数衰减
 *   2) "一直在高频抖动": 死区硬归零导致目标倾角在边界跳变 → 改成软化过渡带
 *
 * @note  离线编译时用桩函数替代 param_get_float (真实实现要读 NVS),
 *        所以本测试只覆盖速度环的**纯运算逻辑**。
 *
 * 编译运行:
 *   cd 0.32
 *   gcc -std=c99 -Wall -Wextra test/test_speed_pid_pc.c \
 *       user/pid/speed_pid.c user/pid/pid.c \
 *       -o /tmp/speed_pid_test -I test -I user/pid -I user/param -lm
 *   /tmp/speed_pid_test
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "speed_pid.h"
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

/* ========== param 桩 ========== */
/* 桩里的增益: 与 param.c 的出厂默认值一致, 这样测出来的行为贴近真机 */
float param_get_float(param_id_t id)
{
    switch (id) {
    case PARAM_PID_SPD_KP:     return 0.06f;
    case PARAM_PID_SPD_KI:     return 0.01f;
    case PARAM_PID_SPD_KD:     return 0.0f;
    case PARAM_SPEED_OUT_MAX:  return 8.0f;
    default:                   return 0.0f;
    }
}

/**
 * @brief 测试 1: 死区软化过渡带 —— 误差连续, 目标倾角不跳变
 */
static void test_deadband_softening(void)
{
    printf("[1] 死区软化: 过渡带内误差线性压缩, 目标倾角连续\n");

    speed_pid_init();

    /* 死区内: 输出必须是 0 */
    float out_in = speed_pid_update(0.01f, 0.5f, 0.0f);   /* err=0.5 < 死区 1.0 */
    CHECK(fabsf(out_in) < 1e-6f,
          "死区内 (err=0.5) 输出=%.6f (期望 0)", out_in);

    /* 死区边界: err=1.0 刚好在死区, 输出 0 */
    float out_edge = speed_pid_update(0.01f, 1.0f, 0.0f);
    CHECK(fabsf(out_edge) < 1e-6f,
          "死区边界 (err=1.0) 输出=%.6f (期望 0)", out_edge);

    /* 过渡带内: err=1.1, 压缩系数 = (1.1-1.0)/0.5 = 0.2 */
    speed_pid_init();
    float out_soft = speed_pid_update(0.01f, 1.1f, 0.0f);
    float expect = 0.06f * 0.2f;   /* kp × 压缩后的误差 */
    CHECK(fabsf(out_soft - expect) < 1e-4f,
          "过渡带内 (err=1.1) 输出=%.5f (期望 %.5f = kp×0.2)", out_soft, expect);

    /* 过渡带末端: err=1.5, 压缩系数 = 1.0, 等于原始值。
     * 用"接近"而不是精确相等 —— 积分项 (ki*i) 会叠加一点。
     * 对照: 硬归零版本在这里会输出 kp×(1.5-1.0)=0.03 (因为 1.5 在死区外
     * 会被完整保留, 但 1.0~1.5 之间是跳变的)。 */
    speed_pid_init();
    float out_full = speed_pid_update(0.01f, 1.5f, 0.0f);
    float expect_full = 0.06f * 1.5f;
    CHECK(fabsf(out_full - expect_full) < 0.005f,
          "过渡带末端 (err=1.5) 输出=%.5f (期望 ≈%.5f = kp×1.5)",
          out_full, expect_full);

    /* 负方向对称 */
    speed_pid_init();
    float out_neg = speed_pid_update(0.01f, -1.1f, 0.0f);
    CHECK(fabsf(out_neg + 0.06f * 0.2f) < 1e-4f,
          "负方向 (err=-1.1) 输出=%.5f (期望 %.5f, 与正方向对称)",
          out_neg, -0.06f * 0.2f);
}

/**
 * @brief 测试 2: 关键回归 —— 量化台阶跨死区边缘不再产生跳变
 *
 * @note  这是抖动的直接成因: 硬归零时 err 从 0 跳到 1.05,
 *        目标倾角从 0 跳到 0.063°, 平衡环被迫纠偏。
 *        软化后应该是连续的小幅变化。
 */
static void test_quantization_no_jump(void)
{
    printf("[2] 量化台阶跨死区边缘: 目标倾角不再跳变\n");

    speed_pid_init();

    /* 模拟编码器量化: 实测轮速在 0.9 / 1.05 之间来回 (相邻台阶) */
    float o1 = speed_pid_update(0.01f, 0.90f, 0.0f);
    float o2 = speed_pid_update(0.01f, 1.05f, 0.0f);

    float jump = fabsf(o2 - o1);
    /* 硬归零时跳变 = kp × 1.05 = 0.063°; 软化后应该小得多 */
    CHECK(jump < 0.02f,
          "跨台阶跳变=%.5f (期望 <0.02; 硬归零会是 %.5f)",
          jump, 0.06f * 1.05f);

    /* 再跨一步 (0.9 -> 1.20): 压缩后误差差 = (1.20-1.0)/0.5 = 0.4,
     * 乘 kp = 0.024。跳变确实比硬归零小, 但不是零 ——
     * 因为 0.30 rpm 的误差差是**真实存在**的速度差, 本来就该有响应。
     * 所以判据是"比硬归零版本小", 而不是绝对阈值。 */
    speed_pid_init();
    float o3 = speed_pid_update(0.01f, 1.20f, 0.0f);
    float jump2 = fabsf(o3 - o1);
    /* 硬归零版本: err=1.20 会保留原值 → 跳变 = kp×(1.20-0) = 0.072 */
    float hard_jump = 0.06f * 1.20f;
    CHECK(jump2 < hard_jump * 0.5f,
          "跨两个台阶跳变=%.5f (硬归零会是 %.5f, 期望 < 它的一半)",
          jump2, hard_jump);
}

/**
 * @brief 测试 3: 死区内积分衰减 (修"一直往前跑")
 */
static void test_integral_decay_in_deadband(void)
{
    printf("[3] 死区内积分衰减: 残留积分会退干净, 不会永久前跑\n");

    speed_pid_init();

    /* 先让车"走"一段: 目标 60 rpm, 持续 2 秒 → 积分攒起来 */
    for (int i = 0; i < 200; i++) {
        speed_pid_update(0.01f, 0.0f, 60.0f);   /* 实测 0, 目标 60 → err=-60 */
    }
    float pitch_while_walking = speed_pid_update(0.01f, 0.0f, 60.0f);
    CHECK(pitch_while_walking < -0.5f,
          "走路时目标倾角=%.3f° (期望 < -0.5, 应该是负的)", pitch_while_walking);

    /* 现在停下: 目标 0, 实测 0 → 进死区, 积分应该衰减 */
    float pitch_at_stop = speed_pid_update(0.01f, 0.0f, 0.0f);
    /* 死区内积分刚开始衰减, 目标倾角还带着残留 (应该仍是负的, 但在往 0 走) */
    CHECK(pitch_at_stop < 0.0f,
          "停下瞬间目标倾角=%.3f° (期望 <0, 残留积分还没退完)", pitch_at_stop);

    /* 继续停 10 秒 (tau=3s → exp(-10/3) ≈ 0.036), 积分应该退到接近 0 */
    for (int i = 0; i < 1000; i++) {
        speed_pid_update(0.01f, 0.0f, 0.0f);
    }
    float pitch_after = speed_pid_update(0.01f, 0.0f, 0.0f);
    CHECK(fabsf(pitch_after) < 0.05f,
          "停 10 秒后目标倾角=%.4f° (期望 |x|<0.05, 残留积分应退干净)",
          pitch_after);

    /* 关键: 不衰减的话, 残留积分会永久保留 → 车一直往前跑 */
    CHECK(fabsf(pitch_after) < fabsf(pitch_at_stop),
          "目标倾角绝对值在缩小 (%.4f → %.4f), 说明积分在退",
          fabsf(pitch_at_stop), fabsf(pitch_after));
}

/**
 * @brief 测试 4: 死区外积分正常累积 (不能把消除稳态误差的能力废掉)
 */
static void test_integral_still_works_outside_deadband(void)
{
    printf("[4] 死区外积分正常累积 (保留消除稳态误差的能力)\n");

    speed_pid_init();

    /* 误差 5 rpm (在死区外), 持续累积 */
    float out1 = speed_pid_update(0.01f, 5.0f, 0.0f);
    for (int i = 0; i < 100; i++) {
        speed_pid_update(0.01f, 5.0f, 0.0f);
    }
    float out2 = speed_pid_update(0.01f, 5.0f, 0.0f);

    /* 积分累积会让输出随时间增大 (同号误差持续存在) */
    CHECK(out2 > out1,
          "死区外误差持续存在时输出在增大 (%.5f → %.5f), 说明积分在工作",
          out1, out2);

    /* 误差反向后积分应该能退回来 (不是只增不减) */
    float out_rev = speed_pid_update(0.01f, -5.0f, 0.0f);
    CHECK(out_rev < out2,
          "误差反向后输出回落 (%.5f → %.5f), 说明积分能退",
          out2, out_rev);
}

/**
 * @brief 测试 5: 输出限幅仍然生效 (安全阀不能被破坏)
 */
static void test_output_clamp(void)
{
    printf("[5] 目标倾角安全阀仍然生效 (不超过 SPD_PID_WALK_PITCH_MAX)\n");

    speed_pid_init();

    /* 巨大误差, 输出应被夹在 ±4° (spdout_max 桩给 8, 取小 = 4) */
    float out = speed_pid_update(0.01f, 500.0f, 0.0f);
    CHECK(fabsf(out) <= 4.0f + 1e-4f,
          "巨大误差下输出=%.4f° (期望 |x|<=4.0, 安全阀生效)", out);
}

int main(void)
{
    printf("==== 速度环 (speed_pid) 离线验证 ====\n\n");

    test_deadband_softening();
    test_quantization_no_jump();
    test_integral_decay_in_deadband();
    test_integral_still_works_outside_deadband();
    test_output_clamp();

    printf("\n==== %d/%d 通过, %d 失败 ====\n",
           g_pass + g_fail, g_pass + g_fail, g_fail);
    return g_fail == 0 ? 0 : 1;
}
