/* test/test_legmap_pc.c
 *
 * 腿高查表逻辑的离线验证 (Linux/macOS 直接 gcc 编译运行)
 * 验证项 (全部必须通过):
 *   1. 双表独立: 录入左表不影响右表
 *   2. U 型曲线合法 (单峰 ≤1 个拐点) → usable=true
 *   3. 来回抖 (≥2 拐点) → usable=false
 *   4. 越界查表 → 夹到端点, 不外推
 *   5. 左右腿不齐时补偿正确性:
 *      两腿独立的 pitch-φ 关系, 补偿 = L.pitch(φ) - L.pitch(home) + R.pitch(φ) - R.pitch(home)
 *   6. clear/clear_all/parse_leg/leg_char
 *   7. 非法 leg_id 被安全挡掉
 *
 * 编译: cd 1.0 && \
 *   gcc -std=c99 -Wall -DLEGMAP_OFFLINE_TEST \
 *       test/test_legmap_pc.c user/legmap/legmap.c \
 *       -o /tmp/legmap_test \
 *       -I test -I user/legmap
 * 运行: /tmp/legmap_test
 */

/* 这份桩头必须在 user/legmap 的 esp_err.h 之前被包含 */
#include "legmap_offline.h"

/* 离线版不需要 (也不存在) nvs_flash.h; 在 legmap.c 的 #ifndef LEGMAP_OFFLINE_TEST 段
 * 里压根不会被引用, 所以这里不引。 */
#include "legmap.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>

/* ============== 测试用例 ============== */

static int g_pass = 0, g_fail = 0;

#define EXPECT_NEAR(a, b, eps) do { \
    double _a = (a), _b = (b); \
    if (fabs(_a - _b) > (eps)) { \
        printf("  [FAIL] %s:%d  %.6f != %.6f (diff %.6f > %.6f)\n", \
               __FILE__, __LINE__, _a, _b, fabs(_a - _b), (double)(eps)); \
        g_fail++; \
    } else { \
        g_pass++; \
    } \
} while (0)

#define EXPECT_TRUE(cond) do { \
    if (!(cond)) { printf("  [FAIL] %s:%d  expect true: %s\n", \
                            __FILE__, __LINE__, #cond); g_fail++; } \
    else g_pass++; \
} while (0)

#define EXPECT_FALSE(cond) do { \
    if ((cond)) { printf("  [FAIL] %s:%d  expect false: %s\n", \
                           __FILE__, __LINE__, #cond); g_fail++; } \
    else g_pass++; \
} while (0)

static void test_independent_tables(void)
{
    printf("[1] 双表独立\n");
    legmap_clear_all();

    /* 录入 4 个点到 LEG_LEFT */
    for (int i = 0; i < 4; i++) {
        float deg = 30.0f + i * 20.0f;
        legmap_upsert_point(LEG_LEFT, deg, 60.0f + i, 0.1f * i);
    }
    /* 录入 5 个点到 LEG_RIGHT, 直距更大一些 */
    for (int i = 0; i < 5; i++) {
        float deg = 30.0f + i * 15.0f;
        legmap_upsert_point(LEG_RIGHT, deg, 70.0f + i, 0.2f * i);
    }

    EXPECT_TRUE(legmap_get_one(LEG_LEFT)->count == 4);
    EXPECT_TRUE(legmap_get_one(LEG_RIGHT)->count == 5);

    /* 互不污染: 查右腿的 height 不该出现左腿的 60+ */
    float h_r = legmap_height(LEG_RIGHT, 50.0f);
    EXPECT_TRUE(h_r >= 70.0f && h_r <= 75.0f);

    float h_l = legmap_height(LEG_LEFT, 50.0f);
    EXPECT_NEAR(h_l, 61.0f, 1e-3);   /* 左腿 50° 处精确插值应为 61 */
}

static void test_u_shape_legal(void)
{
    printf("[2] U 型曲线 → usable=true\n");
    legmap_clear_all();

    /* U 型: 30°=80, 50°=70, 70°=60(谷), 90°=72, 100°=85 */
    float ds[] = {30, 50, 70, 90, 100};
    float hs[] = {80, 70, 60, 72, 85};
    float ps[] = {-0.5, -1.5, -2.0, -1.2, -0.3};
    for (int i = 0; i < 5; i++) {
        legmap_upsert_point(LEG_LEFT, ds[i], hs[i], ps[i]);
    }
    EXPECT_TRUE(legmap_is_usable(LEG_LEFT, 2));
}

static void test_zigzag_rejected(void)
{
    printf("[3] 来回抖 → usable=false\n");
    legmap_clear_all();

    /* 高度来回: 60, 70, 65, 75 → 两个拐点 */
    float hs[] = {60, 70, 65, 75};
    for (int i = 0; i < 4; i++) {
        legmap_upsert_point(LEG_LEFT, 30.0f + i * 20.0f, hs[i], 0.1f * i);
    }
    EXPECT_FALSE(legmap_is_usable(LEG_LEFT, 2));
}

static void test_clamp_no_extrapolate(void)
{
    printf("[4] 越界查表 → 夹到端点\n");
    legmap_clear_all();

    legmap_upsert_point(LEG_LEFT, 40.0f, 50.0f, -0.3f);
    legmap_upsert_point(LEG_LEFT, 60.0f, 60.0f,  0.2f);
    legmap_upsert_point(LEG_LEFT, 80.0f, 75.0f,  1.2f);

    /* 小于首点: 夹到 40° 的值 */
    EXPECT_NEAR(legmap_height(LEG_LEFT, 25.0f), 50.0f, 1e-3);
    /* 大于末点: 夹到 80° 的值 */
    EXPECT_NEAR(legmap_height(LEG_LEFT, 95.0f), 75.0f, 1e-3);
    /* 中间: 插值 */
    EXPECT_NEAR(legmap_height(LEG_LEFT, 50.0f), 55.0f, 1e-3);
    EXPECT_NEAR(legmap_height(LEG_LEFT, 70.0f), 67.5f, 1e-3);
}

static void test_lr_pitch_compensation(void)
{
    printf("[5] 左右两腿 Pitch 补偿正确性\n");
    legmap_clear_all();

    /* 左右两腿拍-φ 关系不同, 模拟装配不齐 */
    float degs[] = {30, 50, 70, 90, 100};
    /* 左腿: pitch 摆动 -2 ~ +1 */
    float pL[] = {-2.0, -1.5, -0.8,  0.5,  1.0};
    /* 右腿: pitch 摆动 -1 ~ +2 (与左腿不一致, 这就是要分两表的原因) */
    float pR[] = {-1.0, -0.7, -0.1,  1.4,  2.0};

    for (int i = 0; i < 5; i++) {
        legmap_upsert_point(LEG_LEFT,  degs[i], 80.0f + i, pL[i]);
        legmap_upsert_point(LEG_RIGHT, degs[i], 80.0f + i, pR[i]);
    }
    EXPECT_TRUE(legmap_is_usable(LEG_LEFT, 2));
    EXPECT_TRUE(legmap_is_usable(LEG_RIGHT, 2));

    float home = 70.0f;
    /* 真实 MPU 在 φ 处读到的 pitch (物理上 = 左腿贡献 + 右腿贡献) */
    /* 在任意 φ, 补偿应等于 "L.pitch(φ) + R.pitch(φ) - L.pitch(home) - R.pitch(home)" */
    for (int i = 0; i < 5; i++) {
        float phi = degs[i];
        float comp_L = legmap_pitch_comp(LEG_LEFT,  phi, home);
        float comp_R = legmap_pitch_comp(LEG_RIGHT, phi, home);
        float total = comp_L + comp_R;
        float expect = (pL[i] + pR[i]) - (pL[2] + pR[2]);  /* home=70° 对应 pL[2],pR[2] */
        EXPECT_NEAR(total, expect, 1e-3);
    }

    /* 边界点 (越界): 夹到端点 */
    float comp_25 = legmap_pitch_comp(LEG_LEFT, 25.0f, home);
    /* 25° 夹到 30° → pL[0]=-2.0; home=70° → pL[2]=-0.8; 差 -1.2 */
    EXPECT_NEAR(comp_25, -2.0f - (-0.8f), 1e-3);
}

static void test_clear_apis(void)
{
    printf("[6] clear / clear_all / parse_leg / leg_char\n");
    legmap_clear_all();
    legmap_upsert_point(LEG_LEFT,  50.0f, 60.0f, 0.0f);
    legmap_upsert_point(LEG_RIGHT, 60.0f, 70.0f, 0.0f);
    EXPECT_TRUE(legmap_get_one(LEG_LEFT)->count == 1);
    EXPECT_TRUE(legmap_get_one(LEG_RIGHT)->count == 1);

    legmap_clear(LEG_LEFT);
    EXPECT_TRUE(legmap_get_one(LEG_LEFT)->count == 0);
    EXPECT_TRUE(legmap_get_one(LEG_RIGHT)->count == 1);  /* 不受影响 */

    legmap_upsert_point(LEG_LEFT, 50.0f, 60.0f, 0.0f);
    legmap_clear_all();
    EXPECT_TRUE(legmap_get_one(LEG_LEFT)->count == 0);
    EXPECT_TRUE(legmap_get_one(LEG_RIGHT)->count == 0);

    /* parse_leg */
    leg_id_t l;
    EXPECT_TRUE(legmap_parse_leg("L",   &l) == ESP_OK && l == LEG_LEFT);
    EXPECT_TRUE(legmap_parse_leg("r",   &l) == ESP_OK && l == LEG_RIGHT);
    EXPECT_TRUE(legmap_parse_leg("LEFT",  &l) == ESP_OK && l == LEG_LEFT);
    EXPECT_TRUE(legmap_parse_leg("right", &l) == ESP_OK && l == LEG_RIGHT);
    EXPECT_TRUE(legmap_parse_leg("X",   &l) == ESP_ERR_INVALID_ARG);

    /* leg_char */
    EXPECT_TRUE(legmap_leg_char(LEG_LEFT)  == 'L');
    EXPECT_TRUE(legmap_leg_char(LEG_RIGHT) == 'R');
}

static void test_invalid_leg(void)
{
    printf("[7] 非法 leg_id 都被安全挡掉\n");
    legmap_clear_all();
    /* 这些不该崩 */
    EXPECT_TRUE(legmap_height((leg_id_t)99, 50.0f) == LEGMAP_HEIGHT_UNKNOWN);
    EXPECT_TRUE(legmap_pitch_comp((leg_id_t)99, 50.0f, 70.0f) == 0.0f);
    EXPECT_TRUE(!legmap_is_usable((leg_id_t)99, 2));
    EXPECT_TRUE(legmap_upsert_point((leg_id_t)99, 50.0f, 60.0f, 0.0f) == ESP_ERR_INVALID_ARG);
    EXPECT_TRUE(legmap_clear((leg_id_t)99) == ESP_ERR_INVALID_ARG);
}

int main(void)
{
    /* 注: 不调 legmap_init (会触发 NVS), 直接用核心 API 验证。 */
    test_independent_tables();
    test_u_shape_legal();
    test_zigzag_rejected();
    test_clamp_no_extrapolate();
    test_lr_pitch_compensation();
    test_clear_apis();
    test_invalid_leg();

    printf("\n==== %d/%d 通过, %d 失败 ====\n",
           g_pass, g_pass + g_fail, g_fail);
    return g_fail == 0 ? 0 : 1;
}