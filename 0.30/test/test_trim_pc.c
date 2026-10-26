/**
 * @file test_trim_pc.c
 * @brief 配平表 trim.c 的离线单元测试 (PC 上直接 gcc 跑, 不依赖硬件)
 *
 * @note  编译运行:
 *      cd 0.11
 *      gcc -std=c99 -Wall -Wextra test/test_trim_pc.c user/trim/trim.c \
 *          -o /tmp/trim_test -I test -I user/trim -DTRIM_OFFLINE_TEST
 *      /tmp/trim_test
 *
 * @note  -DTRIM_OFFLINE_TEST 让 trim.h 带上 test/trim_offline.h 的桩头,
 *        并让 trim.c 跳过 NVS 段 —— 测的是**纯逻辑**。
 *
 * @note  为什么要有这个测试
 *   插值和边界处理是"看起来对但会静默算错"的重灾区:
 *   越界不夹端点 → 补偿突变; 空表返回哨兵值 → 上层当成有效数据用。
 *   这些光看代码看不出来, 必须跑边界值。
 *
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-05
 */

#include "trim.h"   /* 桩头由它自己带进来, 不要在这里 include trim_offline.h */
static int g_pass = 0, g_fail = 0;

#define CHECK(cond, fmt, ...) do {                                  \
    if (cond) { g_pass++; }                                          \
    else { g_fail++; printf("  [FAIL] " fmt "\n", ##__VA_ARGS__); } \
} while (0)

/** @brief 浮点比较 (带容差) */
static int near(float a, float b, float eps)
{
    float d = a - b;
    if (d < 0) d = -d;
    return d <= eps;
}

static void banner(const char *name)
{
    printf("\n=== %s ===\n", name);
}

/* ================= 用例 1: 空表 ================= */
static void test_empty(void)
{
    banner("空表: 不能崩, 补偿必须为 0");
    trim_clear();

    CHECK(trim_get()->count == 0, "count 应为 0");
    CHECK(trim_is_usable(2) == false, "空表不可用");
    CHECK(near(trim_pitch_span(), 0.0f, 1e-6f), "span 应为 0");

    /* 少于 2 个点时补偿恒为 0 —— 这是"没标定也能安全跑"的保证 */
    CHECK(near(trim_pitch(80.0f, 80.0f), 0.0f, 1e-6f), "空表 trim_pitch 应为 0");

    trim_set_point(80.0f, 1.23f);
    CHECK(trim_is_usable(2) == false, "1 个点仍不可用");
    CHECK(near(trim_pitch(80.0f, 80.0f), 0.0f, 1e-6f),
          "只有 1 个点时 trim_pitch 应为 0 (点数不足不补偿)");
}

/* ================= 用例 2: 补偿完全抵消腿弯曲 ================= */
static void test_comp_cancel(void)
{
    banner("补偿正确性: 修正后处处恒为参考值");
    trim_clear();

    /* 模拟真实情况: 腿越弯, 平衡姿态越歪 (pitch 单调上升) */
    trim_set_point(30.0f, 0.70f);
    trim_set_point(45.0f, 1.60f);
    trim_set_point(60.0f, 2.40f);
    trim_set_point(80.0f, 3.20f);
    trim_set_point(100.0f, 4.50f);

    const float ref = 60.0f;       /* 参考角 = 舵机中心角 */
    float base = trim_pitch(ref, ref);
    CHECK(near(base, 0.0f, 1e-6f), "参考角处补偿应为 0, 实得 %.4f", (double)base);

    /* 逐点验算: 原始 pitch − 补偿 应该处处相等 (=参考点的 pitch) */
    const float want = 2.40f;      /* 60° 处的 pitch */
    const float degs[] = {30.0f, 45.0f, 60.0f, 80.0f, 100.0f};
    const float raws[] = {0.70f, 1.60f, 2.40f, 3.20f, 4.50f};
    for (int i = 0; i < 5; i++) {
        float fixed = raws[i] - trim_pitch(degs[i], ref);
        CHECK(near(fixed, want, 1e-4f),
              "%.0f° 修正后应为 %.2f, 实得 %.4f",
              (double)degs[i], (double)want, (double)fixed);
    }
}

/* ================= 用例 3: 插值 ================= */
static void test_interp(void)
{
    banner("线性插值: 中间点应在两端之间");
    trim_clear();
    trim_set_point(40.0f, 1.0f);
    trim_set_point(80.0f, 3.0f);

    /* (40+80)/2 = 60 处应为中点 2.0; 减去参考角 40 的 1.0 → 补偿 1.0 */
    CHECK(near(trim_pitch(60.0f, 40.0f), 1.0f, 1e-4f),
          "60° 相对 40° 的补偿应为 1.0, 实得 %.4f", (double)trim_pitch(60.0f, 40.0f));

    /* 四分之一处 */
    CHECK(near(trim_pitch(50.0f, 40.0f), 0.5f, 1e-4f),
          "50° 相对 40° 的补偿应为 0.5, 实得 %.4f", (double)trim_pitch(50.0f, 40.0f));

    /* 精确命中端点 */
    CHECK(near(trim_pitch(40.0f, 80.0f), -2.0f, 1e-4f),
          "40° 相对 80° 的补偿应为 -2.0, 实得 %.4f", (double)trim_pitch(40.0f, 80.0f));
}

/* ================= 用例 4: 越界夹端点 ================= */
static void test_clamp(void)
{
    banner("越界: 夹到端点, 不外推, 补偿不突变");
    trim_clear();
    trim_set_point(40.0f, 1.0f);
    trim_set_point(80.0f, 3.0f);

    /* 小于首点 → 夹到 40° 的值; 大于末点 → 夹到 80° 的值 */
    CHECK(near(trim_pitch(20.0f, 40.0f), 0.0f, 1e-4f),
          "20° (<首点) 应夹到 40°, 相对 40° 补偿为 0");
    CHECK(near(trim_pitch(25.0f, 40.0f), 0.0f, 1e-4f),
          "25° (<首点) 应夹到 40°");

    /* 关键: 越界一点和越界很多, 补偿值必须**一样** (夹端点, 不外推)。
     * 如果这里不等, 说明代码在偷偷外推, 机构到死点后曲线形状会完全变。 */
    float comp_a = trim_pitch(120.0f, 40.0f);
    float comp_b = trim_pitch(200.0f, 40.0f);
    CHECK(near(comp_a, comp_b, 1e-6f),
          "越界补偿必须与越界多少无关 (%.4f vs %.4f)", (double)comp_a, (double)comp_b);
    CHECK(near(comp_a, 2.0f, 1e-4f),
          "越界应夹到 80° (pitch 3.0), 相对 40° 补偿应为 2.0, 实得 %.4f",
          (double)comp_a);

    /* 边界值本身要连续: 79.9 和 80.1 (后者越界) 补偿应几乎连续 */
    float c1 = trim_pitch(79.9f, 40.0f);
    float c2 = trim_pitch(80.1f, 40.0f);
    CHECK(near(c1, c2, 0.01f),
          "跨越末点时补偿应连续 (%.4f vs %.4f)", (double)c1, (double)c2);
}

/* ================= 用例 5: 乱序插入自动排序 ================= */
static void test_sorted_insert(void)
{
    banner("插入: 乱序录入自动排成升序");
    trim_clear();

    trim_set_point(80.0f, 3.0f);
    trim_set_point(40.0f, 1.0f);
    trim_set_point(100.0f, 5.0f);
    trim_set_point(60.0f, 2.0f);

    const trim_t *t = trim_get();
    CHECK(t->count == 4, "应有 4 个点, 实得 %u", t->count);
    for (int i = 0; i + 1 < (int)t->count; i++) {
        CHECK(t->pts[i].servo_deg < t->pts[i + 1].servo_deg,
              "第 %d→%d 项未升序 (%.0f ≥ %.0f)", i, i + 1,
              (double)t->pts[i].servo_deg, (double)t->pts[i + 1].servo_deg);
    }
    /* 排序正确的话, 60° 的插值必须落在 40 和 80 之间 */
    CHECK(near(trim_pitch(60.0f, 40.0f), 1.0f, 1e-4f),
          "60° 相对 40° 应为 1.0, 实得 %.4f", (double)trim_pitch(60.0f, 40.0f));
}

/* ================= 用例 6: 同角度覆盖 ================= */
static void test_overwrite(void)
{
    banner("覆盖: 同角度重录不增加点数");
    trim_clear();
    trim_set_point(60.0f, 2.0f);
    trim_set_point(60.0f, 2.5f);
    trim_set_point(60.0f, 2.8f);

    const trim_t *t = trim_get();
    CHECK(t->count == 1, "同角度应覆盖, count 仍为 1, 实得 %u", t->count);
    CHECK(near(t->pts[0].pitch_deg, 2.8f, 1e-4f),
          "应保留最后一次的值 2.8, 实得 %.4f", (double)t->pts[0].pitch_deg);
}

/* ================= 用例 7: 删除 ================= */
static void test_delete(void)
{
    banner("删除: 删中间项后剩余仍升序");
    trim_clear();
    trim_set_point(40.0f, 1.0f);
    trim_set_point(60.0f, 2.0f);
    trim_set_point(80.0f, 3.0f);

    trim_del_point(60.0f);
    const trim_t *t = trim_get();
    CHECK(t->count == 2, "删 1 个应剩 2 个, 实得 %u", t->count);
    CHECK(near(t->pts[0].servo_deg, 40.0f, 1e-4f), "剩 [40, 80]");
    CHECK(near(t->pts[1].servo_deg, 80.0f, 1e-4f), "剩 [40, 80]");

    /* 删不存在的角度不算错 */
    trim_del_point(123.0f);
    CHECK(trim_get()->count == 2, "删不存在的点不应改变 count");
}

/* ================= 用例 8: 合法性检查 ================= */
static void test_validation(void)
{
    banner("校验: 俯仰角超范围要拒收, 点数上限要拦住");
    trim_clear();

    CHECK(trim_set_point(60.0f, 999.0f) == ESP_ERR_INVALID_ARG,
          "俯仰角 999° 应被拒");
    CHECK(trim_set_point(60.0f, -999.0f) == ESP_ERR_INVALID_ARG,
          "俯仰角 -999° 应被拒");
    CHECK(trim_get()->count == 0, "被拒的点不应进表");

    /* 边界内的值要收 */
    CHECK(trim_set_point(60.0f, 30.0f) == ESP_OK, "30° (上边界) 应接受");
    CHECK(trim_set_point(60.0f, -30.0f) == ESP_OK, "-30° (下边界) 应接受");

    /* 填满上限 */
    trim_clear();
    for (int i = 0; i < TRIM_MAX_POINTS; i++) {
        CHECK(trim_set_point(30.0f + i, 0.5f + i * 0.1f) == ESP_OK,
              "第 %d 个点应接受", i + 1);
    }
    CHECK(trim_get()->count == TRIM_MAX_POINTS, "应刚好装满 %d 个", TRIM_MAX_POINTS);
    CHECK(trim_set_point(30.0f + TRIM_MAX_POINTS, 1.0f) == ESP_ERR_NO_MEM,
          "超出上限应被拒");
    CHECK(trim_get()->count == TRIM_MAX_POINTS, "被拒后 count 不变");
}

/* ================= 用例 9: 无变化的数据不可用 ================= */
static void test_no_variation(void)
{
    banner("防呆: 所有点 Pitch 一样 → 判不可用");
    trim_clear();
    trim_set_point(40.0f, 2.0f);
    trim_set_point(60.0f, 2.0f);
    trim_set_point(80.0f, 2.0f);
    CHECK(near(trim_pitch_span(), 0.0f, 1e-6f), "span 应为 0");
    CHECK(trim_is_usable(2) == false,
          "没有变化的表没有信息量, 应判不可用 (否则会一直返回一个常数补偿)");

    /* 恢复一点变化后应可用 */
    trim_set_point(80.0f, 2.5f);
    CHECK(trim_pitch_span() > 0.4f, "span 应变为 0.5");
    CHECK(trim_is_usable(2) == true, "有变化的表应可用");

    /* 极小但非零的变化应该**允许** —— 实际换角度时 Pitch 变化确实可能不大。
     * 门槛卡太死会把合法数据判死。 */
    trim_clear();
    trim_set_point(40.0f, 2.00f);
    trim_set_point(80.0f, 2.30f);
    CHECK(trim_is_usable(2) == true,
          "0.3° 的变化是真实的, 不该被判死 (门槛 0.05°)");
}

/* ================= 用例 10: U 型数据 (真实腿机构就是这样) ================= */
static void test_u_shape(void)
{
    banner("U 型数据: 俯仰角先降后升也要正常工作");
    trim_clear();
    /* 模拟: 30°→2.5, 45°→1.8, 60°→1.0 (最低), 80°→1.2, 100°→2.0 */
    trim_set_point(30.0f,  2.5f);
    trim_set_point(45.0f,  1.8f);
    trim_set_point(60.0f,  1.0f);
    trim_set_point(80.0f,  1.2f);
    trim_set_point(100.0f, 2.0f);

    CHECK(trim_is_usable(2) == true, "U 型是真实腿机构的形状, 应判可用");

    const float ref = 60.0f;   /* U 型最低点做参考角 */
    const float want = 1.0f;
    const float degs[] = {30.0f, 45.0f, 60.0f, 80.0f, 100.0f};
    const float raws[] = {2.5f, 1.8f, 1.0f, 1.2f, 2.0f};
    for (int i = 0; i < 5; i++) {
        float fixed = raws[i] - trim_pitch(degs[i], ref);
        CHECK(near(fixed, want, 1e-4f),
              "U 型 %.0f° 修正后应为 %.2f, 实得 %.4f",
              (double)degs[i], (double)want, (double)fixed);
    }
}

/* ================= 用例 11: 线性近似会丢多少 (对照实验) ================= */
static void test_linear_loss(void)
{
    banner("对照: U 型数据用线性近似会丢多少补偿量");
    trim_clear();
    trim_set_point(30.0f,  2.5f);
    trim_set_point(45.0f,  1.8f);
    trim_set_point(60.0f,  1.0f);
    trim_set_point(80.0f,  1.2f);
    trim_set_point(100.0f, 2.0f);

    const trim_t *t = trim_get();

    /* 最小二乘拟合 pitch = k*deg + b */
    float sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int i = 0; i < (int)t->count; i++) {
        float x = t->pts[i].servo_deg, y = t->pts[i].pitch_deg;
        sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    float n = (float)t->count;
    float k = (n * sxy - sx * sy) / (n * sxx - sx * sx);
    float b = (sy - k * sx) / n;

    /* 对比: 在 30° 处, 查表补偿 vs 线性补偿, 差多少 */
    const float ref = 60.0f;
    float table_c = trim_pitch(30.0f, ref);
    float lin_c   = (k * 30.0f + b) - (k * 60.0f + b);
    float err = table_c - lin_c;

    printf("  30° 处: 查表补偿 %.3f°, 线性补偿 %.3f°, 差 %.3f°\n",
           (double)table_c, (double)lin_c, (double)err);
    CHECK(fabsf(err) > 0.1f,
          "U 型用线性近似应该会明显丢失补偿量 (差 %.3f°)", (double)err);
    printf("  → 这就是 0.11 坚持用查表而不是直线的原因\n");
}

int main(void)
{
    printf("========== trim.c 离线测试 ==========\n");

    test_empty();
    test_comp_cancel();
    test_interp();
    test_clamp();
    test_sorted_insert();
    test_overwrite();
    test_delete();
    test_validation();
    test_no_variation();
    test_u_shape();
    test_linear_loss();

    printf("\n========== %d 通过, %d 失败 ==========\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
