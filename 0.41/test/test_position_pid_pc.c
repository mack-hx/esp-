/**
 * @file test_position_pid_pc.c
 * @brief 位置保持环离线单元测试 (PC 上用 gcc 直接跑, 不烧硬件)
 *
 * @note  覆盖:
 *   1) 死区 + 软化过渡带 —— 死区内不要求回位, 过渡带内误差压缩
 *   2) 速度阻尼项 —— 往前动时给的是刹车倾角 (反向)
 *   3) 倾角上限 —— 输出不超过 ±POS_PID_LEAN_MAX
 *   4) 输出连续性 —— 误差从小到大扫过去, 输出不能跳变 (硬夹会跳)
 *   5) 积分抗饱和 + 死区内衰减
 *   6) 输出有限性 —— 极端输入不出 NaN/Inf
 *
 * @note  位置保持环输出的是**目标倾角 (°)**: 位置误差 (cm) + 平移速度 (cm/s)
 *        → 目标倾角 (°) → 平衡环 → 电机输出。
 *
 * 编译运行:
 *   cd 0.41
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
 * 位置保持环的增益用全局变量模拟参数表, 测试可以随时改 (改完要调
 * position_pid_load_params() 才会生效)。 */
static float g_pos_kp = 1.0f;   /* 测试用比例增益 (默认 1.0, 期望值好算) */
static float g_pos_kd = 0.0f;   /* 默认不带阻尼 (单独测阻尼时再打开) */
static float g_pos_ki = 0.0f;   /* 默认纯 P (跟真机默认值一致) */

float param_get_float(param_id_t id)
{
    switch (id) {
    case PARAM_POS_KP:        return g_pos_kp;
    case PARAM_POS_KD:        return g_pos_kd;
    case PARAM_POS_KI:        return g_pos_ki;
    case PARAM_ENCODER_GEAR:  return 30.0f;
    case PARAM_WHEEL_DIA:     return 6.5f;
    default:                  return 0.0f;
    }
}

/**
 * @brief 固定输入跑 n 个周期, 返回最后一次的输出
 * @param err 位置误差 (cm)
 * @param v   平移速度 (cm/s, 前进为正)
 * @param n   周期数 (每个周期 10 ms)
 * @return 输出 (°)
 */
static float settle(float err, float v, int n)
{
    float out = 0.0f;
    for (int i = 0; i < n; i++) {
        out = position_pid_update(0.01f, err, v);
    }
    return out;
}

/**
 * @brief 从软饱和后的输出反推饱和前的原始值
 * @param out 位置保持环输出 (°)
 * @return 原始值 (°)
 * @note  out = L·tanh(raw/L) → raw = L·atanh(out/L)
 */
static float raw_from_out(float out)
{
    float r = out / POS_PID_LEAN_MAX;
    if (r > 0.999f) r = 0.999f;
    if (r < -0.999f) r = -0.999f;
    return POS_PID_LEAN_MAX * atanhf(r);
}

/**
 * @brief 用例 1: 死区 + 软化过渡带
 */
static void test_deadband_softening(void)
{
    printf("[1] 位置保持环死区软化: 死区内不回位, 过渡带内误差压缩\n");

    /* 纯 P (kd=0, ki=0), 这样原始值就是 kp × 压缩后的误差 */
    g_pos_kp = 1.0f;
    g_pos_kd = 0.0f;
    g_pos_ki = 0.0f;

    position_pid_init();
    float out_in = settle(0.05f, 0.0f, 100);
    CHECK(fabsf(out_in) < 1e-3f,
          "死区内 (err=0.05cm) 输出=%.6f° (期望 0)", out_in);

    /* 过渡带: 有效误差 = 过渡带宽度 × ((|err| − 死区) / 过渡带宽度)² */
    position_pid_init();
    float out_soft = settle(0.15f, 0.0f, 100);
    float t = (0.15f - POS_PID_ERR_DEADBAND) / POS_PID_ERR_SOFT_BAND;
    float expect = POS_PID_ERR_SOFT_BAND * t * t;
    float raw_soft = raw_from_out(out_soft);
    CHECK(fabsf(raw_soft - expect) < 1e-3f,
          "过渡带内 (err=0.15cm) 原始值=%.5f° (期望 ≈%.5f)", raw_soft, expect);

    position_pid_init();
    float out_neg = settle(-0.15f, 0.0f, 100);
    float raw_neg = raw_from_out(out_neg);
    CHECK(fabsf(raw_neg + expect) < 1e-3f,
          "负方向 (err=-0.15cm) 原始值=%.5f° (期望 ≈-%.5f)", raw_neg, expect);

    /* 过渡带之外: 有效误差 = 误差 − 死区 */
    position_pid_init();
    float out_full = settle(5.0f, 0.0f, 100);
    float raw_full = raw_from_out(out_full);
    float expect_full = 5.0f - POS_PID_ERR_DEADBAND;
    CHECK(fabsf(raw_full - expect_full) < 0.01f,
          "过渡带外 (err=5cm) 原始值=%.5f° (期望 ≈%.2f = kp×(err−死区))",
          raw_full, expect_full);

    /* 死区边界: 误差从小到大扫过去, 输出不能跳一大截 */
    position_pid_init();
    float below = settle(POS_PID_ERR_DEADBAND - 0.01f, 0.0f, 100);
    position_pid_init();
    float above = settle(POS_PID_ERR_DEADBAND + 0.01f, 0.0f, 100);
    CHECK(fabsf(above - below) < 0.05f,
          "死区边界输出应连续 (%.5f → %.5f°)", below, above);

    /* 过渡带末端两侧必须连续 */
    position_pid_init();
    float band_below = settle(POS_PID_ERR_DEADBAND + POS_PID_ERR_SOFT_BAND - 0.01f, 0.0f, 100);
    position_pid_init();
    float band_above = settle(POS_PID_ERR_DEADBAND + POS_PID_ERR_SOFT_BAND + 0.01f, 0.0f, 100);
    CHECK(fabsf(band_above - band_below) < 0.05f,
          "过渡带末端两侧应连续 (%.5f → %.5f°)", band_below, band_above);
}

/**
 * @brief 用例 2: 速度阻尼项 (回位时的刹车)
 */
static void test_velocity_damping(void)
{
    printf("[2] 速度阻尼: 往前动时给的是反向 (刹车) 倾角\n");

    g_pos_kp = 0.0f;     /* 只留阻尼项 */
    g_pos_kd = 1.0f;
    g_pos_ki = 0.0f;

    position_pid_init();
    float out_fwd = settle(0.0f, 5.0f, 10);
    CHECK(out_fwd < 0.0f,
          "前进 5 cm/s 时输出=%.4f° (期望 < 0, 后倾刹车)", out_fwd);
    CHECK(fabsf(raw_from_out(out_fwd) + 5.0f) < 0.01f,
          "前进 5 cm/s 时原始值=%.4f° (期望 ≈-5)", raw_from_out(out_fwd));

    position_pid_init();
    float out_back = settle(0.0f, -5.0f, 10);
    CHECK(out_back > 0.0f,
          "后退 5 cm/s 时输出=%.4f° (期望 > 0, 前倾刹车)", out_back);

    /* 位置误差项与阻尼项叠加: 目标在前方 (err>0) 且正在前进 → 前倾更多 */
    g_pos_kp = 1.0f;
    position_pid_init();
    float out_both = settle(2.0f, 2.0f, 10);
    float raw_both = raw_from_out(out_both);
    CHECK(fabsf(raw_both - (2.0f - POS_PID_ERR_DEADBAND - 2.0f)) < 0.01f,
          "err=2cm v=2cm/s 时原始值=%.4f° (期望 ≈%.2f)",
          raw_both, 2.0f - POS_PID_ERR_DEADBAND - 2.0f);
}

/**
 * @brief 用例 3: 倾角上限 (安全阀)
 */
static void test_output_clamp(void)
{
    printf("[3] 倾角上限: 输出不超过 ±%.1f°\n", POS_PID_LEAN_MAX);

    g_pos_kp = 1.0f;
    g_pos_kd = 0.0f;
    g_pos_ki = 0.0f;

    position_pid_init();
    float out = settle(1000.0f, 0.0f, 100);
    CHECK(fabsf(out) <= POS_PID_LEAN_MAX + 1e-3f,
          "巨大误差下输出=%.4f° (期望 |x|<=%.1f)", out, POS_PID_LEAN_MAX);

    position_pid_init();
    float out_neg = settle(-1000.0f, 0.0f, 100);
    CHECK(out_neg >= -POS_PID_LEAN_MAX - 1e-3f,
          "巨大负误差下输出=%.4f° (期望 |x|<=%.1f)", out_neg, POS_PID_LEAN_MAX);

    /* 限幅不改变符号 */
    CHECK(out > 0.0f, "正向误差输出应为正 (实际 %.4f)", out);
    CHECK(out_neg < 0.0f, "负向误差输出应为负 (实际 %.4f)", out_neg);

    /* 速度项也不能把输出顶出上限 */
    position_pid_init();
    float out_v = settle(0.0f, -1000.0f, 100);
    CHECK(fabsf(out_v) <= POS_PID_LEAN_MAX + 1e-3f,
          "巨大反向速度下输出=%.4f° (期望 |x|<=%.1f)", out_v, POS_PID_LEAN_MAX);
}

/**
 * @brief 用例 4: 输出连续性 (硬夹会在上限附近跳变, 变成继电器)
 */
static void test_output_continuity(void)
{
    printf("[4] 输出连续性: 误差扫过去不能出现跳变\n");

    g_pos_kp = 1.0f;
    g_pos_kd = 0.0f;
    g_pos_ki = 0.0f;

    float prev = 0.0f;
    float max_jump = 0.0f;
    float prev_raw = 0.0f;
    for (int i = 0; i <= 400; i++) {
        float err = (float)i * 0.05f;      /* 0 ~ 20 cm */
        position_pid_init();
        float out = settle(err, 0.0f, 20);
        float raw = raw_from_out(out);
        if (i > 0) {
            float jump = fabsf(out - prev);
            if (jump > max_jump) max_jump = jump;
        }
        prev = out;
        prev_raw = raw;
    }
    /* 扫描步长 0.05 cm → 有效误差每步最多变 0.05 → 倾角每步最多变 0.05°,
     * 加上软饱和只会更小。留 0.2° 的余量。 */
    CHECK(max_jump < 0.2f,
          "误差从 0 扫到 20cm, 最大单步跳变=%.4f° (期望 < 0.2°)", max_jump);
    (void)prev_raw;
}

/**
 * @brief 用例 5: 积分抗饱和 + 死区内衰减
 */
static void test_integral(void)
{
    printf("[5] 积分: 顶到上限时冻结, 死区内退干净\n");

    g_pos_kp = 1.0f;
    g_pos_kd = 0.0f;
    g_pos_ki = 0.5f;

    position_pid_init();
    /* 巨大误差 → 输出顶在上限, 积分应该被冻结 (不能无限攒) */
    float out_big = settle(50.0f, 0.0f, 500);
    CHECK(fabsf(out_big) <= POS_PID_LEAN_MAX + 1e-3f,
          "大误差下输出=%.4f° (期望 |x|<=%.1f)", out_big, POS_PID_LEAN_MAX);

    /* 误差反号时积分残留会让车冲过头 → 反号后输出必须马上反号 */
    position_pid_init();
    settle(50.0f, 0.0f, 500);
    float out_rev = settle(-0.5f, 0.0f, 1);
    CHECK(out_rev < 0.0f,
          "误差反号后输出=%.4f° (期望 < 0, 不能被残留积分拖住)", out_rev);

    /* 死区内: 积分衰减, 输出回到 0 附近 */
    position_pid_init();
    float out_before = settle(2.0f, 0.0f, 200);
    float out_at_zero = settle(0.05f, 0.0f, 300);
    CHECK(fabsf(out_at_zero) < fabsf(out_before),
          "回正后输出应比偏的时候小 (%.4f → %.4f°)", out_before, out_at_zero);
    float out_long = settle(0.05f, 0.0f, 1000);
    CHECK(fabsf(out_long) < 0.05f,
          "回正 10 秒后输出=%.6f° (期望接近 0)", out_long);
}

/**
 * @brief 用例 6: 输出必须是有限值 (不能出 NaN/Inf)
 */
static void test_finite_output(void)
{
    printf("[6] 输出有限性: 极端输入也不能出 NaN/Inf\n");

    g_pos_kp = 1.0f;
    g_pos_kd = 1.0f;
    g_pos_ki = 0.1f;

    position_pid_init();
    float out = position_pid_update(0.01f, 100.0f, 100.0f);
    CHECK(!isnan(out) && !isinf(out),
          "输出应为有限值 (实际 %.3f)", out);

    position_pid_init();
    float out2 = position_pid_update(0.0f, -1e9f, 1e9f);
    CHECK(!isnan(out2) && !isinf(out2),
          "dt=0 时输出应为有限值 (实际 %.3f)", out2);
}

int main(void)
{
    printf("==== 位置保持环 (position_pid) 离线验证 ====\n\n");

    test_deadband_softening();
    test_velocity_damping();
    test_output_clamp();
    test_output_continuity();
    test_integral();
    test_finite_output();

    printf("\n==== %d/%d 通过, %d 失败 ====\n",
           g_pass + g_fail, g_pass + g_fail, g_fail);
    return g_fail == 0 ? 0 : 1;
}
