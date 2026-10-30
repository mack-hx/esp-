/**
 * @file test_lqr_pc.c
 * @brief LQR 状态反馈离线单元测试 (PC 上 gcc 直接跑, 不烧硬件)
 *
 * @note  覆盖:
 *   1) 倾角项/角速度项 —— u = KP*倾角误差 + KD*角速度, 过零且线性
 *   2) 位置项符号与饱和 —— 目标在前方 → 前倾; 向前运动 → 刹车; 不超过 ±POS_PID_LEAN_MAX
 *   3) 闭环仿真 —— 把 lqr.c 接到 40°/70°/100° 三个腿角的辨识模型上
 *      (含 15~20 ms 执行延迟、10 ms 采样保持、±60% 输出限幅),
 *      3° 倾斜与 6 cm 偏移都要回到零点; 输入增益 ±30%、延迟 ±5 ms 仍然稳定;
 *      开环必须发散 (说明模型和符号是对的, 测试不是空转)
 *
 * 编译运行:
 *   cd 0.41
 *   gcc -std=c99 -Wall -Wextra -DPARAM_OFFLINE_TEST \
 *       test/test_lqr_pc.c user/pid/lqr.c user/pid/pid.c \
 *       -o /tmp/lqr_test -I test -I user/pid -I user/param -lm
 *   /tmp/lqr_test
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "lqr.h"
#include "position_pid.h"
#include "pid.h"

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

static bool near(float a, float b, float tol) { return fabsf(a - b) <= tol; }

/* ========== 辨识模型 ==========
 * 状态 x = [位置 cm, 速度 cm/s, 倾角 °, 角速度 °/s], 输入 u = 电机输出 (%):
 *   v'  = avv*v + avp*p + avo*w + bv*u(t-Td)
 *   w'  = aov*v + aop*p + aoo*w + bo*u(t-Td)
 * 系数来自 logs/data/id_leg{40,70,100}_*handsoff*.csv 的闭环辨识 */
typedef struct {
    const char *name;
    float avv, avp, avo, aov, aop, aoo, bv, bo;
    int delay_ms;
} model_t;

static const model_t k_models[3] = {
    {"40deg",  -14.715f, -11.253f, 2.529f, -14.262f, 105.570f, -33.707f, -0.937f, -65.782f, 20},
    {"70deg",  -16.439f, -10.511f, 2.540f, -95.468f, 139.374f, -16.808f, -0.661f, -61.395f, 15},
    {"100deg", -15.207f, -12.848f, 2.117f, -93.570f, 158.050f, -20.873f, -0.321f, -79.737f, 15},
};

/* 模型状态的位置/速度单位是辨识时里程计的旧刻度; 真实厘米 = 模型单位 × POS_SCALE。
 * 仿真里喂给 lqr.c 之前乘上它, 阈值判断也按模型单位换算 */
#define POS_SCALE   (8.0f / 3.0f)

#define SIM_DT      0.0001f     /* 积分步长 0.1 ms */
#define CTRL_STEPS  100         /* 控制周期 10 ms */
#define RING_LEN    1024        /* 必须大于最大延迟步数 (25 ms = 250 步) */

/**
 * @brief 闭环仿真
 * @param m        被控对象模型
 * @param b_scale  输入增益缩放 (bv, bo 同乘)
 * @param delay_ms 执行延迟 (ms)
 * @param x0       初值 [位置, 速度, 倾角, 角速度]
 * @param closed   1 = 闭环 (lqr.c), 0 = 开环 (u=0)
 * @param seconds  仿真时长
 * @param out_peak_u 输出峰值 |u| (%), 可为 NULL
 * @param out_x    结束时的状态 [位置, 速度, 倾角, 角速度]
 * @return true = 全程未发散 (|倾角|<30°, |位置|<200 cm)
 */
static bool simulate(const model_t *m, float b_scale, int delay_ms, const float x0[4],
                     int closed, float seconds, float *out_peak_u, float out_x[4])
{
    double x[4] = {x0[0], x0[1], x0[2], x0[3]};
    static float ubuf[RING_LEN];                /* 环形缓冲: 存最近 RING_LEN 步的输出 */
    int delay_steps = delay_ms * 10;            /* 1 ms = 10 个步长 */
    int n_steps = (int)(seconds / SIM_DT);
    float u_cmd = 0.0f;
    float peak = 0.0f;
    bool ok = true;

    for (int n = 0; n < n_steps; n++) {
        if (n % CTRL_STEPS == 0) {
            u_cmd = 0.0f;
            if (closed) {
                /* 锚点 = 0, 站立不走: 目标倾角 = -位置环等效倾角 (前倾为正的位置环, 倾角前倾为负) */
                float lean = lqr_pos_lean(-(float)x[0] * POS_SCALE, (float)x[1] * POS_SCALE);
                u_cmd = lqr_balance_out((float)x[2] + lean, (float)x[3]);
                if (u_cmd >  60.0f) u_cmd =  60.0f;
                if (u_cmd < -60.0f) u_cmd = -60.0f;
            }
            if (fabsf(u_cmd) > peak) peak = fabsf(u_cmd);
        }
        ubuf[n % RING_LEN] = u_cmd;
        float u = (n >= delay_steps) ? ubuf[(n - delay_steps) % RING_LEN] : 0.0f;

        double dv = m->avv * x[1] + m->avp * x[2] + m->avo * x[3] + m->bv * b_scale * u;
        double dw = m->aov * x[1] + m->aop * x[2] + m->aoo * x[3] + m->bo * b_scale * u;
        x[0] += x[1] * SIM_DT;
        x[1] += dv * SIM_DT;
        x[2] += x[3] * SIM_DT;
        x[3] += dw * SIM_DT;

        if (fabs(x[2]) > 30.0 || fabs(x[0]) > 200.0 || !isfinite(x[0] + x[1] + x[2] + x[3])) {
            ok = false;
            break;
        }
    }
    if (out_peak_u) *out_peak_u = peak;
    if (out_x) for (int i = 0; i < 4; i++) out_x[i] = (float)x[i];
    return ok;
}

/* ========== 1) 倾角项 / 角速度项 ========== */
static void test_balance_out(void)
{
    printf("\n=== 倾角项 + 角速度项 ===\n");
    CHECK(near(lqr_balance_out(0.0f, 0.0f), 0.0f, 1e-6f), "误差和角速度都是 0 → 输出 0");
    CHECK(near(lqr_balance_out(2.0f, 0.0f), LQR_KP * 2.0f, 1e-4f), "倾角误差 2° → KP*2");
    CHECK(near(lqr_balance_out(0.0f, 50.0f), LQR_KD * 50.0f, 1e-4f), "角速度 50°/s → KD*50");
    CHECK(near(lqr_balance_out(-1.5f, 20.0f), LQR_KP * -1.5f + LQR_KD * 20.0f, 1e-4f),
          "两项叠加");
    CHECK(LQR_KP > 0.0f && LQR_KD > 0.0f, "KP、KD 为正 (车往哪边倒轮子往哪边追)");
}

/* ========== 2) 位置项 ========== */
static void test_pos_lean(void)
{
    printf("\n=== 位置项 ===\n");
    CHECK(near(lqr_pos_lean(0.0f, 0.0f), 0.0f, 1e-6f), "已在锚点且静止 → 0");
    CHECK(near(lqr_pos_lean(0.05f, 0.0f), 0.0f, 1e-6f), "死区内 → 0");
    CHECK(lqr_pos_lean(2.0f, 0.0f) > 0.0f, "锚点在前方 → 前倾 (正)");
    CHECK(lqr_pos_lean(-2.0f, 0.0f) < 0.0f, "锚点在后方 → 后倾 (负)");
    CHECK(lqr_pos_lean(0.0f, 10.0f) < 0.0f, "正在向前跑 → 刹车 (负)");
    CHECK(near(lqr_pos_lean(3.0f, 4.0f), -lqr_pos_lean(-3.0f, -4.0f), 1e-5f), "关于原点奇对称");

    float e = pid_soft_deadband(1.0f, LQR_ERR_DEADBAND, LQR_ERR_SOFT_BAND);
    float raw = (-LQR_KX * e) / LQR_KP;
    float expect = POS_PID_LEAN_MAX * tanhf(raw / POS_PID_LEAN_MAX);
    CHECK(near(lqr_pos_lean(1.0f, 0.0f), expect, 1e-5f),
          "1 cm 偏差 → %.3f° (期望 %.3f°)", (double)lqr_pos_lean(1.0f, 0.0f), (double)expect);

    CHECK(lqr_pos_lean(30.0f, 10.0f) < lqr_pos_lean(30.0f, 0.0f) - 0.5f,
          "大偏差下速度项仍起刹车作用");

    CHECK(lqr_pos_lean(1000.0f, 0.0f) <= POS_PID_LEAN_MAX + 1e-4f &&
          lqr_pos_lean(-1000.0f, 0.0f) >= -POS_PID_LEAN_MAX - 1e-4f,
          "极大偏差 → 不超过 ±%.1f°", (double)POS_PID_LEAN_MAX);

    int mono = 1;
    float prev = lqr_pos_lean(0.0f, 0.0f);
    for (int i = 1; i <= 200; i++) {
        float cur = lqr_pos_lean(i * 0.1f, 0.0f);
        if (cur < prev - 1e-6f) mono = 0;
        prev = cur;
    }
    CHECK(mono, "偏差从 0 扫到 20 cm → 输出单调不减");

    CHECK(isfinite(lqr_pos_lean(1e30f, -1e30f)), "极端输入 → 输出有限");

    /* 增益可在线设置: 速度项随 kv 线性变化, 默认值与宏一致, 负值按 0 */
    #define RAW_OF(l) (POS_PID_LEAN_MAX * atanhf((l) / POS_PID_LEAN_MAX))   /* tanh 之前的原始量 */
    float base = lqr_pos_lean(0.0f, 6.0f);
    lqr_set_pos_gains(-LQR_KX, 2.0f * -LQR_KV, LQR_AMAX);
    CHECK(near(RAW_OF(lqr_pos_lean(0.0f, 6.0f)), 2.0f * RAW_OF(base), 1e-4f), "kv 翻倍 → 速度项翻倍 (%.3f°)",
          (double)lqr_pos_lean(0.0f, 6.0f));
    lqr_set_pos_gains(-1.0f, -1.0f, -1.0f);
    CHECK(near(lqr_pos_lean(2.0f, 10.0f), 0.0f, 1e-6f), "负增益按 0 → 无位置项");
    lqr_set_pos_gains(-LQR_KX, -LQR_KV, LQR_AMAX);
    CHECK(near(lqr_pos_lean(0.0f, 6.0f), base, 1e-6f), "恢复默认增益 → 与宏一致");

    /* 速度项小速度渐隐: |v| ≤ LO 不参与, ≥ HI 完全参与, 中间连续单调 */
    CHECK(near(lqr_pos_lean(0.5f, LQR_VEL_LO), lqr_pos_lean(0.5f, 0.0f), 1e-6f) &&
          near(lqr_pos_lean(0.5f, -1.0f), lqr_pos_lean(0.5f, 0.0f), 1e-6f),
          "|v| ≤ %.1f cm/s → 速度项不参与", (double)LQR_VEL_LO);
    CHECK(near(RAW_OF(lqr_pos_lean(0.0f, LQR_VEL_HI)), (-LQR_KV) * (0.0f - LQR_VEL_HI) / LQR_KP, 1e-4f),
          "|v| ≥ %.1f cm/s → 速度项完全参与 (%.3f°)", (double)LQR_VEL_HI, (double)lqr_pos_lean(0.0f, LQR_VEL_HI));
    int fade_ok = 1;
    float fprev = lqr_pos_lean(0.0f, 0.0f);
    for (int i = 1; i <= 100; i++) {
        float f = lqr_pos_lean(0.0f, i * 0.07f);
        if (f > fprev + 1e-6f || fprev - f > 0.15f) fade_ok = 0;
        fprev = f;
    }
    CHECK(fade_ok, "速度 0~7 cm/s 刹车量连续且单调");
    #undef RAW_OF
}

/* ========== 刹车距离回位速度曲线 ========== */
static void test_return_profile(void)
{
    printf("\n=== 刹车距离回位速度曲线 ===\n");
    const float p = LQR_KX / LQR_KV;                 /* 线性区斜率 (1/s), 符号约定 KX、KV 同为负, 比值为正 */
    const float lin_dist = LQR_AMAX / (p * p);       /* 线性区边界偏差 (cm) */

    /* 线性区 (偏差 < lin_dist) 与关闭曲线 (amax=0) 完全一致 */
    float small_on = lqr_pos_lean(1.0f, 1.0f);
    lqr_set_pos_gains(-LQR_KX, -LQR_KV, 0.0f);
    float small_off = lqr_pos_lean(1.0f, 1.0f);
    lqr_set_pos_gains(-LQR_KX, -LQR_KV, LQR_AMAX);
    CHECK(1.0f < lin_dist && near(small_on, small_off, 1e-6f),
          "小偏差 (1 cm < %.1f cm) 与纯线性一致", (double)lin_dist);

    /* 大偏差: 期望回位速度被限住, 以期望速度回位时刹车量为 0 */
    float e = 20.0f;
    float e_soft = pid_soft_deadband(e, LQR_ERR_DEADBAND, LQR_ERR_SOFT_BAND);
    float v_des = sqrtf(2.0f * LQR_AMAX * (e_soft - 0.5f * lin_dist));
    if (v_des > LQR_VMAX) v_des = LQR_VMAX;
    CHECK(near(lqr_pos_lean(e, v_des), 0.0f, 1e-4f) && e_soft > lin_dist,
          "偏差 20 cm 以 %.1f cm/s 回位 → 倾角指令 0", (double)v_des);
    CHECK(lqr_pos_lean(e, v_des + 10.0f) < -0.2f, "回位比期望快 → 刹车");
    CHECK(lqr_pos_lean(e, 0.0f) > 0.2f, "静止 → 向锚点加速");

    /* 曲线连续且单调: 偏差从 0 扫到 40 cm */
    int ok = 1;
    float prev = lqr_pos_lean(0.0f, 0.0f);
    for (int i = 1; i <= 400; i++) {
        float cur = lqr_pos_lean(i * 0.1f, 0.0f);
        if (cur < prev - 1e-6f || fabsf(cur - prev) > 0.15f) ok = 0;
        prev = cur;
    }
    CHECK(ok, "偏差 0~40 cm 扫描 → 单调不减且无跳变");

    CHECK(near(lqr_pos_lean(e, 5.0f), -lqr_pos_lean(-e, -5.0f), 1e-5f), "大偏差关于原点奇对称");
    CHECK(isfinite(lqr_pos_lean(0.0f, 1e30f)) && lqr_pos_lean(0.0f, 1e30f) >= -POS_PID_LEAN_MAX - 1e-4f,
          "极大速度 → 输出有限且不超限");

    /* 比纯线性更早收力: 4 cm 偏差、静止时, 开启曲线的前倾指令比线性小 */
    lqr_set_pos_gains(-LQR_KX, -LQR_KV, 0.0f);
    float off = lqr_pos_lean(4.0f, 0.0f);
    lqr_set_pos_gains(-LQR_KX, -LQR_KV, LQR_AMAX);
    float on = lqr_pos_lean(4.0f, 0.0f);
    CHECK(on < off - 0.1f, "4 cm 偏差静止: 曲线 %.2f° < 线性 %.2f°", (double)on, (double)off);
}

/* ========== 位置积分项 ========== */
static void test_pos_integral(void)
{
    printf("\n=== 位置积分项 ===\n");
    const float dt = 0.01f;

    lqr_pos_integral_reset();
    CHECK(near(lqr_pos_integral(dt, 0.0f, 0.0f), 0.0f, 1e-6f), "无偏差 → 0");

    float v = 0.0f;
    for (int i = 0; i < 100; i++) v = lqr_pos_integral(dt, 1.3f, 0.0f);
    CHECK(near(v, LQR_POS_KI * 1.3f * 1.0f, 0.05f),
          "静止且偏差 1.3 cm 持续 1 s → %.2f° (期望 %.2f°)", (double)v,
          (double)(LQR_POS_KI * 1.3f));

    for (int i = 0; i < 2000; i++) v = lqr_pos_integral(dt, 1.3f, 0.0f);
    CHECK(near(v, LQR_POS_I_MAX, 1e-5f), "长时间累积 → 限幅 %.1f°", (double)LQR_POS_I_MAX);

    float v_move = v;
    for (int i = 0; i < 100; i++) v_move = lqr_pos_integral(dt, 1.3f, 5.0f);
    CHECK(near(v_move, LQR_POS_I_MAX * expf(-1.0f / LQR_POS_I_DECAY), 0.01f),
          "车在动 (5 cm/s) → 1 s 后衰减到 %.3f°", (double)v_move);

    lqr_pos_integral_reset();
    for (int i = 0; i < 100; i++) v = lqr_pos_integral(dt, 1.3f, 0.0f);
    float after = lqr_pos_integral(dt, -1.3f, 0.0f);
    CHECK(v > 0.5f && near(after, v * expf(-dt / LQR_POS_I_RELEASE), 1e-4f),
          "偏差变号 → 不瞬间清零, 按 %.1f s 衰减 (%.3f° → %.3f°)",
          (double)LQR_POS_I_RELEASE, (double)v, (double)after);
    for (int i = 0; i < 400; i++) after = lqr_pos_integral(dt, -1.3f, 0.0f);
    CHECK(fabsf(after) < 0.01f || after < 0.0f, "衰减到 0 后允许反向累积");

    lqr_pos_integral_reset();
    for (int i = 0; i < 100; i++) v = lqr_pos_integral(dt, 1.3f, 0.0f);
    float in_db = lqr_pos_integral(dt, 0.05f, 0.0f);
    CHECK(v > 0.5f && near(in_db, v * expf(-dt / LQR_POS_I_RELEASE), 1e-4f),
          "进入死区 → 不瞬间清零, 按 %.1f s 衰减 (%.3f° → %.3f°)",
          (double)LQR_POS_I_RELEASE, (double)v, (double)in_db);

    lqr_pos_integral_reset();
    for (int i = 0; i < 100; i++) lqr_pos_integral(dt, 1.3f, 0.0f);
    float far_prev = lqr_pos_integral(dt, 1.3f, 0.0f);
    float far_next = lqr_pos_integral(dt, 15.0f, 0.0f);
    CHECK(near(far_next, far_prev * expf(-dt / LQR_POS_I_DECAY), 1e-4f),
          "偏差 15 cm (> %.1f cm) 不累积, 按 %.1f s 衰减", (double)LQR_POS_I_BAND, (double)LQR_POS_I_DECAY);

    lqr_pos_integral_reset();
    float neg = 0.0f;
    for (int i = 0; i < 100; i++) neg = lqr_pos_integral(dt, -1.3f, 0.0f);
    CHECK(near(neg, -LQR_POS_KI * 1.3f, 0.05f), "关于原点奇对称 (%.2f°)", (double)neg);

    lqr_pos_integral_reset();
    CHECK(isfinite(lqr_pos_integral(dt, 1e30f, -1e30f)), "极端输入 → 输出有限");
    lqr_pos_integral_reset();
}

/* ========== 3) 闭环仿真 ========== */
static void test_closed_loop(void)
{
    printf("\n=== 闭环仿真 (三个腿角的辨识模型) ===\n");
    const float tilt[4]   = {0.0f, 0.0f, 3.0f, 0.0f};
    const float offset[4] = {6.0f, 0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 3; i++) {
        const model_t *m = &k_models[i];
        float xf[4], pk;

        bool ok = simulate(m, 1.0f, m->delay_ms, tilt, 0, 2.0f, NULL, xf);
        CHECK(!ok, "%s 开环 3° 倾斜 → 发散 (模型不稳定, 测试有效)", m->name);

        ok = simulate(m, 1.0f, m->delay_ms, tilt, 1, 6.0f, &pk, xf);
        CHECK(ok && fabsf(xf[2]) < 0.3f && fabsf(xf[0]) < 0.6f,
              "%s 3° 倾斜 → 6 s 后倾角 %.2f° 位置 %.2f cm, 峰值输出 %.0f%%",
              m->name, (double)xf[2], (double)xf[0], (double)pk);
        CHECK(pk <= 60.0f + 1e-3f, "%s 输出不超过 60%%", m->name);

        ok = simulate(m, 1.0f, m->delay_ms, offset, 1, 6.0f, &pk, xf);
        CHECK(ok && fabsf(xf[2]) < 0.3f && fabsf(xf[0]) < 0.6f,
              "%s 6 cm 偏移 → 6 s 后倾角 %.2f° 位置 %.2f cm, 峰值输出 %.0f%%",
              m->name, (double)xf[2], (double)xf[0], (double)pk);
    }
}

static void test_robustness(void)
{
    printf("\n=== 参数扰动 (输入增益 ±30%%, 延迟 ±5 ms) ===\n");
    const float tilt[4] = {0.0f, 0.0f, 3.0f, 0.0f};
    const float scale[2] = {0.7f, 1.3f};
    const int ddelay[2] = {-5, 5};

    for (int i = 0; i < 3; i++) {
        const model_t *m = &k_models[i];
        int stable = 0, total = 0;
        float xf[4];
        for (int a = 0; a < 2; a++) {
            total++;
            if (simulate(m, scale[a], m->delay_ms, tilt, 1, 6.0f, NULL, xf) &&
                fabsf(xf[2]) < 0.3f && fabsf(xf[0]) < 0.6f) stable++;
        }
        for (int a = 0; a < 2; a++) {
            total++;
            if (simulate(m, 1.0f, m->delay_ms + ddelay[a], tilt, 1, 6.0f, NULL, xf) &&
                fabsf(xf[2]) < 0.3f && fabsf(xf[0]) < 0.6f) stable++;
        }
        CHECK(stable == total, "%s 扰动 %d/%d 稳定", m->name, stable, total);
    }
}

int main(void)
{
    printf("========== lqr.c 离线测试 ==========\n");
    test_balance_out();
    test_pos_lean();
    test_return_profile();
    test_pos_integral();
    test_closed_loop();
    test_robustness();
    printf("\n========== %d 通过, %d 失败 ==========\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
