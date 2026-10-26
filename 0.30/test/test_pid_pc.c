/**
 * @file test_pid_pc.c
 * @brief pid_update_rate() 的离线验证 (PC 上 gcc 直接跑, 不依赖硬件)
 *
 * @brief 为什么必须离线验证
 *   D 项的**符号**是最容易出错、后果最严重的一处:
 *   符号错了 = 负阻尼 = 车越晃越大直至摔倒。
 *   这种错误在编译期和静态检查里看不出来, 只能靠数值算一遍。
 *
 * @brief 验证的物理判据 (不是"跑通就行")
 *   1) D 项方向: 车体往前倒 (rate>0) 时, 电机必须**往倒的方向追得更凶**
 *      (输出与 rate 同号) —— 这才是阻尼; 往回收是帮着车继续倒。
 *   2) 闭环阻尼: 用真实控制器驱动"轮+倒立摆"模型自由释放,
 *      摆幅必须衰减; 把 D 符号取反作对照, 摆幅必须增长。
 *   3) 积分不炸: 阶跃输入下积分应被 i_max 夹住。
 *   4) 首拍不产生假阻尼冲量。
 *   5) 误差微分与角速度反馈在"慢变信号"上应接近, "突变"时行为不同。
 *   6) dt 变化时增益保持一致。
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

/* ---------- 用例 1: D 项方向 ---------- */
static void test_damping_sign(void)
{
    printf("[1] D 项方向: 车在往前倒 (rate>0) 时电机必须往倒的方向追得更凶\n");

    bal_pid_t pid;
    pid_config(&pid, 0.0f, 0.0f, 5.0f, 100.0f, 100.0f);   /* 只开 Kd */

    /* 先跑一拍把 primed 置位 (真实控制里就是上电第一拍) */
    pid_update_rate(&pid, 0.0f, 0.0f, 0.01f);

    /* 误差为 0, 车正以 +50 °/s 往前倒 → 输出必须为正 (往前追) */
    float out = pid_update_rate(&pid, 0.0f, 50.0f, 0.01f);
    CHECK(out > 0.0f, "rate=+50, err=0 → out=%.2f (期望 > 0, 往前追 = 阻尼)", out);

    out = pid_update_rate(&pid, 0.0f, -50.0f, 0.01f);
    CHECK(out < 0.0f, "rate=-50, err=0 → out=%.2f (期望 < 0)", out);

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

/* ---------- 用例 2: 闭环阻尼 (轮 + 倒立摆) ----------
 * 用真实的 pid_update_rate() 驱动一个"轮子 + 倒立摆"模型:
 *     θ'' = W0²·θ − K·u        (u = 控制器输出 / 100, 正 = 轮子往前追)
 *     W0² = g/L  (L = 重心离轮轴 0.12m), K = 电机加速度系数
 * 闭环稳定要求 D 项为正: u 里含 +kd·θ̇ → θ'' 里出现 −K·kd·θ̇ = 阻尼项。
 * 符号取反就变成负阻尼, 振幅必然增长。
 */
#define PLANT_W0SQ  81.75f     /* (g/L), L = 0.12 m */
#define PLANT_K     91.8f      /* 电机加速度系数 (100% 输出时的等效值) */
#define SIM_DT      0.005f     /* 仿真步长 (比控制周期更细) */
#define CTRL_DT     0.01f      /* 控制周期 10ms */

/**
 * @brief 跑一次闭环仿真
 * @param kp     P 系数 (%/°)
 * @param kd     D 系数 (%/(°/s))
 * @param d_flip true = 用**错**的 D 符号 (负号), 作对照组
 * @param early_max [出参] 前 2 秒的最大振幅 (弧度)
 * @param late_max  [出参] 后 2 秒的最大振幅 (弧度)
 */
static void run_closed_loop(float kp, float kd, bool d_flip,
                            float *early_max, float *late_max)
{
    float theta = 6.0f * (float)M_PI / 180.0f;   /* 初始倾角 6° */
    float rate  = 0.0f;
    float ctrl_dt_acc = 0.0f;

    bal_pid_t pid;
    pid_config(&pid, kp, 0.0f, kd, 1000.0f, 100.0f);

    float early = 0.0f, late = 0.0f;
    int   i;

    for (i = 0; i < 2000; i++) {          /* 10 秒 */
        /* --- 控制周期 (每 2 个仿真步算一次) --- */
        ctrl_dt_acc += SIM_DT;
        if (ctrl_dt_acc >= CTRL_DT - 1e-6f) {
            ctrl_dt_acc = 0.0f;

            float err_deg  = theta * 180.0f / (float)M_PI;
            float rate_deg = rate  * 180.0f / (float)M_PI;
            float cmd = pid_update_rate(&pid, err_deg, rate_deg, CTRL_DT);
            if (d_flip) {
                /* 对照组: 手动把 D 项取负 (= 2026-10-25 修之前的错误符号) */
                cmd = pid.kp * err_deg + pid.ki * pid.i - pid.kd * rate_deg;
                if (cmd >  pid.out_max) cmd =  pid.out_max;
                if (cmd < -pid.out_max) cmd = -pid.out_max;
            }
            float u = cmd / 100.0f;        /* 归一化成 -1~1 */

            float thdd = PLANT_W0SQ * theta - PLANT_K * u;
            rate += thdd * SIM_DT;
            theta += rate * SIM_DT;
        }

        /* 前 2 秒 / 后 2 秒分别记最大振幅: 阻尼正确时后者必须更小 */
        float mag = fabsf(theta);
        if (i < 400) {
            if (mag > early) early = mag;
        } else if (i >= 1600) {
            if (mag > late) late = mag;
        }
    }

    *early_max = early;
    *late_max  = late;
}

static void test_closed_loop_damping(void)
{
    printf("[2] 闭环阻尼: 真实控制器驱动倒立摆, 振幅必须衰减 (对照: 符号反了必增长)\n");

    float early = 0.0f, late = 0.0f;

    run_closed_loop(12.0f, 1.5f, false, &early, &late);
    CHECK(late < early,
          "正确符号: 后 2 秒振幅 %.3f° < 前 2 秒 %.3f° (阻尼生效)",
          late * 57.29578f, early * 57.29578f);

    float good_late = late;
    run_closed_loop(12.0f, 1.5f, true, &early, &late);
    CHECK(late > good_late,
          "对照 (D 符号取反): 后 2 秒振幅 %.3f° > 正确符号的 %.3f° (负阻尼, 越晃越大)",
          late * 57.29578f, good_late * 57.29578f);
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
    CHECK(out > 0.0f, "第二拍 rate=500 → out=%.1f (期望生效, > 0)", out);
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

/* ---------- 用例 7: 抗积分饱和 (2026-10-26) ----------
 * 输出顶在限幅上时, 误差继续同向积分会让 i 越攒越大;
 * 等误差反向, 系统要先把这段"退积分"走完才能响应 → 输出长时间卡在限幅上。
 * 现象: 车先站住, 站一会儿后积分顶死, 然后开始高频抖。
 */
static void test_anti_windup(void)
{
    printf("[7] 输出饱和时冻结积分 (抗积分饱和)\n");

    bal_pid_t pid;
    pid_config(&pid, 0.0f, 1.0f, 0.0f, 1000.0f, 10.0f);   /* i_max=1000, out_max=10 */

    /* 恒定误差 +1.0 → 输出很快顶到 +10, 此后积分必须冻结 */
    float out = 0.0f;
    for (int i = 0; i < 5000; i++) {
        out = pid_update_rate(&pid, 1.0f, 0.0f, 0.01f);
    }
    CHECK(fabsf(out - 10.0f) < 0.01f, "恒定误差 → out=%.3f (期望顶在 +10)", out);
    CHECK(pid.i < 12.0f,
          "积分被冻结在 i=%.3f (期望 ≈10 = out_max/Ki; 无抗饱和时会攒到 i_max=1000)", pid.i);

    /* 误差反向: 积分要能退回来 (不能被永久冻住)。
     * err=-1 每拍让 i 减 0.01, 1200 拍后 i 从 +10 走到 -10 (被限幅夹住)。 */
    for (int i = 0; i < 1200; i++) {
        out = pid_update_rate(&pid, -1.0f, 0.0f, 0.01f);
    }
    CHECK(pid.i < 0.0f, "误差反向后积分退回到 i=%.3f (期望 < 0)", pid.i);
    CHECK(out < 0.0f, "误差反向后输出 out=%.3f (期望 < 0, 响应恢复)", out);
}

/* ---------- 用例 8: 运行中改参数不清积分 ----------
 * App 上拖 Kp 滑块必须立刻生效, 但不能把已经攒起来的积分清掉
 * (清掉的话积分项会从 0 重新爬, 输出出现台阶)。
 */
static void test_set_gains_keeps_integral(void)
{
    printf("[8] pid_set_gains() 只改系数, 保留积分历史\n");

    bal_pid_t pid;
    pid_config(&pid, 0.0f, 1.0f, 0.0f, 100.0f, 100.0f);

    for (int i = 0; i < 50; i++) pid_update_rate(&pid, 1.0f, 0.0f, 0.01f);
    float i_before = pid.i;
    CHECK(i_before > 0.4f, "积分已攒到 i=%.3f (期望 ≈0.5)", i_before);

    /* 运行中改参数 (模拟 App 拖滑块) */
    pid_set_gains(&pid, 20.0f, 2.0f, 1.5f, 100.0f, 60.0f);
    CHECK(fabsf(pid.i - i_before) < 1e-6f,
          "改参数后积分保持 i=%.3f (期望 %.3f, 不能被清掉)", pid.i, i_before);
    CHECK(pid.kp == 20.0f && pid.ki == 2.0f && pid.kd == 1.5f && pid.out_max == 60.0f,
          "新系数已生效: kp=%.1f ki=%.1f kd=%.1f out_max=%.1f",
          (double)pid.kp, (double)pid.ki, (double)pid.kd, (double)pid.out_max);

    /* 改参数后误差不变, 输出应按新系数立刻变化 (参数实时生效) */
    float out = pid_update_rate(&pid, 1.0f, 0.0f, 0.01f);
    CHECK(out > 20.0f, "新 Kp 立刻生效: err=1° → out=%.2f (期望 > 20)", out);
}

int main(void)
{
    printf("==== pid_update_rate() 离线验证 ====\n\n");
    test_damping_sign();
    test_closed_loop_damping();
    test_integral_clamp();
    test_first_sample_no_kick();
    test_differs_from_error_derivative();
    test_dt_invariance();
    test_anti_windup();
    test_set_gains_keeps_integral();

    printf("\n==== %d/%d 通过, %d 失败 ====\n",
           g_pass + g_fail, g_pass + g_fail, g_fail);
    return g_fail == 0 ? 0 : 1;
}
