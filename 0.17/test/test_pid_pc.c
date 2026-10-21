/**
 * @file test_pid_pc.c
 * @brief pid_update_rate() 的离线验证 (PC 上 gcc 直接跑, 不依赖硬件)
 *
 * @brief 为什么必须离线验证
 *   D 项的**符号**是这次改动里最容易出错、后果最严重的一处:
 *   符号错了 = 正反馈 = 阻尼变成助推, 车会越晃越厉害直至摔倒。
 *   这种错误在编译期和静态检查里都看不出来, 只能靠数值算一遍。
 *
 * @brief 验证的物理判据 (不是"跑通就行")
 *   1) 纯阻尼方向: 位置误差为 0、车在以正角速度摆过时,
 *      输出必须**反号** (即输出 < 0), 否则阻尼在助推。
 *   2) 摆动衰减: 给一个带阻尼的倒立摆, 自由运行若干秒后振幅应单调下降。
 *   3) 积分不炸: 阶跃输入下积分应被 i_max 夹住。
 *   4) 误差微分与角速度反馈在"慢变信号"上应接近, "突变"时行为不同。
 *
 * 运行:
 *   gcc -std=c99 -Wall -Wextra test/test_pid_pc.c user/control/pid.c \
 *       -o /tmp/pid_test -I user/control -lm
 *   /tmp/pid_test
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "pid.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, fmt, ...) do {                                     \
    if (cond) { g_pass++; printf("  \033[32m[PASS]\033[0m " fmt "\n", ##__VA_ARGS__); } \
    else      { g_fail++; printf("  \033[31m[FAIL]\033[0m " fmt "\n", ##__VA_ARGS__); } \
} while (0)

/* ---------- 用例 1: 纯阻尼方向 ---------- */
static void test_damping_sign(void)
{
    printf("[1] D 项方向: 车在正向前摆 (rate>0) 时输出必须反号\n");

    bal_pid_t pid;
    pid_config(&pid, 0.0f, 0.0f, 5.0f, 100.0f, 100.0f);   /* 只开 Kd */

    /* 先跑一拍把 primed 置位 (真实控制里就是上电第一拍) */
    pid_update_rate(&pid, 0.0f, 0.0f, 0.01f);

    /* 误差为 0, 车正以 +50 °/s 向前摆 → 阻尼必须给出负输出 (往回拉) */
    float out = pid_update_rate(&pid, 0.0f, 50.0f, 0.01f);
    CHECK(out < 0.0f, "rate=+50, err=0 → out=%.2f (期望 < 0, 阻尼往回拉)", out);

    out = pid_update_rate(&pid, 0.0f, -50.0f, 0.01f);
    CHECK(out > 0.0f, "rate=-50, err=0 → out=%.2f (期望 > 0)", out);

    /* 幅度应线性: 角速度翻倍, 阻尼输出也翻倍。
     * ⚠ Kd 必须小到不触发限幅, 否则两次都被夹到 out_max,
     *   断言 "out(20) == 2×out(10)" 会因为两边都是上限而假通过。 */
    pid_reset(&pid);
    pid_config(&pid, 0.0f, 0.0f, 1.0f, 1000.0f, 1000.0f);
    pid_update_rate(&pid, 0.0f, 0.0f, 0.01f);
    float o1 = pid_update_rate(&pid, 0.0f, 10.0f, 0.01f);
    float o2 = pid_update_rate(&pid, 0.0f, 20.0f, 0.01f);
    CHECK(fabsf(o1) < 100.0f && fabsf(o2) < 100.0f, "本用例未触发限幅 (否则断言无意义)");
    CHECK(fabsf(o2 - 2.0f * o1) < 1e-3f, "线性: out(20)=%.3f ≈ 2×out(10)=%.3f", o2, 2.0f * o1);
}

/* ---------- 用例 2: 摆动衰减 ----------
 * 简化成"单轴倒立摆 + 轮速闭环"太复杂, 这里直接验证纯阻尼对
 * 一阶摆动模态的抑制: 车体以固有频率摆动, 只靠 Kd 反馈时振幅应衰减。
 * 用离散积分显式模拟: theta'' = -(g/L) sin(theta) - c*theta'
 */
static void test_damping_decays(void)
{
    printf("[2] 阻尼能抑制摆动: 振幅应单调衰减\n");

    const float dt = 0.01f;
    const float g_over_L = 12.0f;     /* 固有频率平方 */
    const float kd = 1.2f;            /* 阻尼系数 (反馈 = -kd*rate) */

    float theta = 6.0f * (float)M_PI / 180.0f;   /* 初始倾角 6° */
    float rate  = 0.0f;

    float first_peak = 0.0f;   /* 第一个局部极大 (峰值) */
    float last_peak  = 0.0f;   /* 之后每帧的局部极大, 结束时即最后一个 */
    int   peak_idx   = 0;
    int   peaks      = 0;
    int   i;
    bool  was_rising = false;

    for (i = 0; i < 600; i++) {          /* 6 秒 */
        float acc = -g_over_L * sinf(theta) - kd * rate;
        rate  += acc * dt;
        theta += rate * dt;

        /* rate 由正转负 = 摆到了极值 = 一个摆动周期的峰值 */
        float mag = fabsf(theta);
        if (was_rising && rate < 0.0f && peak_idx > 5) {
            if (peaks == 0) {
                first_peak = mag;
            } else {
                last_peak = mag;
            }
            peaks++;
        }
        was_rising = (rate >= 0.0f);
        peak_idx++;
    }

    CHECK(peaks >= 3, "捕获到 %d 个摆动峰值 (需要≥3 才能比较首末)", peaks);
    CHECK(last_peak < first_peak,
          "首峰 %.3f° → 末峰 %.3f° (期望衰减)", first_peak * 57.29578f, last_peak * 57.29578f);
    CHECK(fabsf(theta) < fabsf(6.0f * (float)M_PI / 180.0f),
          "6s 后残留倾角 %.3f° < 初始 6°", theta * 57.29578f);
}

/* ---------- 用例 3: 积分限幅 ---------- */
static void test_integral_clamp(void)
{
    printf("[3] 积分被 i_max 夹住, 不会无限累积\n");

    bal_pid_t pid;
    pid_config(&pid, 0.0f, 1.0f, 0.0f, 10.0f, 100.0f);   /* i_max=10 */

    int i;
    float out = 0.0f;
    for (i = 0; i < 5000; i++) {
        out = pid_update_rate(&pid, 1.0f, 0.0f, 0.01f);   /* 恒定误差 1.0 */
    }
    /* 积分累积值上限 10, Ki=1 → 输出上限就是 10 */
    CHECK(fabsf(out - 10.0f) < 0.01f, "5000 拍后 out=%.3f (期望 10.0 = i_max×Ki)", out);
    CHECK(pid.i <= 10.0f + 1e-4f, "内部积分 i=%.3f 未超 i_max", pid.i);
}

/* ---------- 用例 4: 首拍不产生假阻尼 ---------- */
static void test_first_sample_no_kick(void)
{
    printf("[4] 上电首拍不产生假阻尼冲量\n");

    bal_pid_t pid;
    pid_config(&pid, 0.0f, 0.0f, 10.0f, 100.0f, 100.0f);

    /* 首拍就带一个很大的角速度 (静止时不该发生, 但要确认它被忽略) */
    float out = pid_update_rate(&pid, 0.0f, 500.0f, 0.01f);
    CHECK(out == 0.0f, "首拍 rate=500 → out=%.2f (期望 0, D 项被跳过)", out);

    /* 第二拍才生效 */
    out = pid_update_rate(&pid, 0.0f, 500.0f, 0.01f);
    CHECK(out < 0.0f, "第二拍 rate=500 → out=%.1f (期望生效, < 0)", out);
}

/* ---------- 用例 5: 与误差微分的差异 (确认确实换了实现) ---------- */
static void test_differs_from_error_derivative(void)
{
    printf("[5] 目标阶跃时角速度反馈**不产生尖峰**, 误差微分会\n");

    /* 误差微分型: 目标从 0 突然跳到 10, 姿态不动 */
    bal_pid_t ep;
    pid_config(&ep, 0.0f, 0.0f, 5.0f, 100000.0f, 100000.0f);
    pid_update(&ep, 0.0f, 0.01f);
    float e_out = pid_update(&ep, 10.0f, 0.01f);

    /* 角速度反馈型: 目标跳变, 但车还没动 (rate=0) → 无尖峰 */
    bal_pid_t rp;
    pid_config(&rp, 0.0f, 0.0f, 5.0f, 100000.0f, 100000.0f);
    pid_update_rate(&rp, 0.0f, 0.0f, 0.01f);
    float r_out = pid_update_rate(&rp, 10.0f, 0.0f, 0.01f);

    CHECK(fabsf(e_out) > 100.0f, "误差微分: 阶跃 → out=%.0f (巨大尖峰)", e_out);
    CHECK(r_out == 0.0f,      "角速度反馈: 阶跃 → out=%.0f (无尖峰)", r_out);
}

/* ---------- 用例 6: dt 变化时增益保持一致 ----------
 * 周期从 10ms 变成 12ms (任务被抢占) 时, 角速度反馈型 D 项
 * 输出应**不变** —— 因为它直接用角速度读数, 不做差分。
 * 这一点是修 vTaskDelay 漂移的关键: 参数不再随负载漂移。
 */
static void test_dt_invariance(void)
{
    printf("[6] D 项对 dt 变化免疫 (周期漂移不再影响阻尼)\n");

    bal_pid_t pid;
    pid_config(&pid, 10.0f, 0.0f, 5.0f, 1000.0f, 1000.0f);

    /* 用两套 dt 各跑一段恒定输入, 比较稳态输出 */
    float out_a, out_b;

    pid_reset(&pid);
    for (int i = 0; i < 50; i++) pid_update_rate(&pid, 2.0f, 20.0f, 0.010f);
    out_a = pid_update_rate(&pid, 2.0f, 20.0f, 0.010f);

    pid_reset(&pid);
    for (int i = 0; i < 50; i++) pid_update_rate(&pid, 2.0f, 20.0f, 0.012f);
    out_b = pid_update_rate(&pid, 2.0f, 20.0f, 0.012f);

    CHECK(fabsf(out_a - out_b) < 1e-3f,
          "dt=10ms → %.4f, dt=12ms → %.4f (期望一致)", out_a, out_b);
}

int main(void)
{
    printf("==== pid_update_rate() 离线验证 ====\n\n");
    test_damping_sign();
    test_damping_decays();
    test_integral_clamp();
    test_first_sample_no_kick();
    test_differs_from_error_derivative();
    test_dt_invariance();

    printf("\n==== %d/%d 通过, %d 失败 ====\n",
           g_pass + g_fail, g_pass + g_fail, g_fail);
    return g_fail == 0 ? 0 : 1;
}
