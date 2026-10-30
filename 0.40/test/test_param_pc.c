/**
 * @file test_param_pc.c
 * @brief 参数表 param.c 的离线单元测试 (PC 上直接 gcc 跑, 不依赖硬件)
 *
 * @note  编译运行:
 *      cd 0.40
 *      gcc -std=c99 -Wall -Wextra test/test_param_pc.c user/param/param.c \
 *          -o /tmp/param_test -I test -I user/param -DPARAM_OFFLINE_TEST
 *      /tmp/param_test
 *
 * @note  -DPARAM_OFFLINE_TEST 让 param.h 带上 user/param/param_offline.h 桩头,
 *        并让 param.c 跳过 NVS 段 —— 测的是**纯逻辑**。
 *
 * @note  为什么要有这个测试 (2026-10-26 的 bug 直接命中这里)
 *   App 的"恢复默认"以前是把 App 内置的默认值表整份下发, 而那份表
 *   早就跟车端脱节 (还是 v7 的老值: spd_kp=20/spd_ki=30/motor_max=30),
 *   结果速度环被意外打开 → 车只朝一个方向转, 根本稳不住。
 *   现在改成下发 {"_reset":true} 由车端自己恢复, 这段解析逻辑必须可靠:
 *   "_reset" 要能真的把整表打回默认值, "_v" 不能被当成参数, 未知 key 不能
 *   把后面的正常参数吃掉。这些都必须跑出来才算数。
 *
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-26
 */

#include "param.h"   /* 桩头由它自己带进来, 不要在这里 include param_offline.h */
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

/* ================= 用例 1: 默认值表 ================= */
static void test_defaults(void)
{
    banner("默认值表: 必须跟 g_param_items 一致");
    param_reset_default();

    /* 这几个是 2026-10-26 出事的关键参数:
     * 速度环默认非 0 (App 方向键改走速度环了: 目标轮速 → 目标倾角 → 平衡环,
     *   增益还是 0 的话方向键等于没反应),
     * 输出上限默认 60% (方向修正后实测 30% 接不住车身) */
    CHECK(near(param_get_float(PARAM_PID_BAL_KP), 12.0f, 1e-6f), "bal_kp 默认应 12");
    CHECK(near(param_get_float(PARAM_PID_BAL_KI),  0.5f, 1e-6f), "bal_ki 默认应 0.5");
    CHECK(near(param_get_float(PARAM_PID_BAL_KD),  0.4f, 1e-6f), "bal_kd 默认应 0.4");
    /* ⚠ bal_kd 是 0.32 的重点: 默认值必须跟 App 侧 paramDef.js 一致。
     * 真机实测 (kd=1.5): 平衡内环出现饱和极限环, 输出常驻 ±60% 打摆,
     * 陀螺读到机身 8~12Hz 摆动 → D 项放大 → 自激, 车站不住。
     * kd=0.4 实测: 输出标准差 0.12%, 饱和帧 0% → 死稳。
     * 断言写死 0.4 是为了"别又把默认值调回去" —— 调参走 App, 别动默认值。 */
    CHECK(near(g_param_items[PARAM_PID_BAL_KD].def, 0.4f, 1e-6f),
          "bal_kd 的 def 应为 0.4 (实际 %.2f)",
          (double)g_param_items[PARAM_PID_BAL_KD].def);
    /* 位置保持环默认打开: pos_kp = 1.5, pos_kd = 0.5, pos_ki = 0。 */
    CHECK(near(param_get_float(PARAM_POS_KP), 1.5f, 1e-6f), "pos_kp 默认应 1.5 (位置保持环默认开)");
    CHECK(near(param_get_float(PARAM_POS_KD), 0.5f, 1e-6f), "pos_kd 默认应 0.5 (阻尼项默认开)");
    CHECK(near(param_get_float(PARAM_POS_KI), 0.0f, 1e-6f), "pos_ki 默认应 0");
    /* pos_kp 的 step 是 0.1 → 显示小数位 1 (需与 App 一致, 见 check_param_sync.py) */
    CHECK(g_param_items[PARAM_POS_KP].decimals == 1,
          "pos_kp 的 decimals 应为 1 (step=0.1), 实际 %u",
          (unsigned)g_param_items[PARAM_POS_KP].decimals);
    /* 速度环增益: 具体数值用 g_param_items 里的 def 比对 (不在这里写死),
     * 免得调完默认值忘了改测试。写死的值只用来卡住"别又调回激进值":
     *   0.15/0.05 在仿真里会让车身倾角波动中位 10.7° (走起来就是剧烈抖)。 */
    CHECK(near(param_get_float(PARAM_PID_SPD_KP),
               g_param_items[PARAM_PID_SPD_KP].def, 1e-6f), "spd_kp 默认应等于 def");
    CHECK(near(param_get_float(PARAM_PID_SPD_KI),
               g_param_items[PARAM_PID_SPD_KI].def, 1e-6f), "spd_ki 默认应等于 def");
    CHECK(param_get_float(PARAM_PID_SPD_KP) <= 0.10f,
          "spd_kp 默认不该超过 0.10 (太大走起来会剧烈摇), 实际 %.3f",
          (double)param_get_float(PARAM_PID_SPD_KP));
    CHECK(param_get_float(PARAM_PID_SPD_KI) <= 0.02f,
          "spd_ki 默认不该超过 0.02 (太大容易积分饱和), 实际 %.3f",
          (double)param_get_float(PARAM_PID_SPD_KI));
    CHECK(near(param_get_float(PARAM_PID_SPD_KD),  0.0f, 1e-6f), "spd_kd 默认应 0");
    CHECK(near(param_get_float(PARAM_MOTOR_OUT_MAX), 60.0f, 1e-6f), "motor_max 默认应 60");
    CHECK(near(param_get_float(PARAM_MOTOR_DEADBAND), 4.0f, 1e-6f), "deadzone 默认应 4");

    /* gyro_lpf 默认必须 0.8 (2026-10-26 "一碰就剧烈抖" 的修复):
     * 这个低通是加在 D 项 (唯一的阻尼源) 上的, 系数越小延迟越大。
     * 0.2 → 时间常数 50ms → 3Hz 处丢 54° 相位 → 扰动一来就过冲。
     * MPU 自带的 41Hz 数字低通已经够了, 这里不需要再滤。 */
    CHECK(near(param_get_float(PARAM_GYRO_LPF), 0.8f, 1e-6f),
          "gyro_lpf 默认应 0.8 (低通延迟会削弱阻尼)");

    /* 每一项的 def 都必须真的被填进去 (漏一项就是白板默认值) */
    for (int i = 0; i < PARAM_MAX; i++) {
        const param_item_t *it = &g_param_items[(param_id_t)i];
        CHECK(it != NULL, "第 %d 项应有描述", i);
        if (!it) continue;
        CHECK(near(param_get_float((param_id_t)i), it->def, 1e-6f),
              "第 %d 项 (%s) 恢复默认后应等于 def", i, it->key);
    }
}

/* ================= 用例 2: 范围钳位 ================= */
static void test_clamp(void)
{
    banner("范围钳位: 超界自动夹到 min/max, NaN/Inf 必须拒绝");
    param_reset_default();

    param_set_float(PARAM_PID_BAL_KP, 9999.0f);
    CHECK(near(param_get_float(PARAM_PID_BAL_KP), 200.0f, 1e-6f), "超上限应夹到 200");

    param_set_float(PARAM_PID_BAL_KP, -50.0f);
    CHECK(near(param_get_float(PARAM_PID_BAL_KP), 0.0f, 1e-6f), "低于下限应夹到 0");

    param_set_float(PARAM_PID_BAL_KP, 12.0f);
    CHECK(near(param_get_float(PARAM_PID_BAL_KP), 12.0f, 1e-6f), "正常值应写入");

    /* NaN / Inf: 夹不住, 进了 PID 就是静默算错 → 必须拒绝 (原值保持不变) */
    param_set_float(PARAM_PID_BAL_KP, 12.0f);
    param_set_float(PARAM_PID_BAL_KP, 0.0f / 0.0f);
    CHECK(near(param_get_float(PARAM_PID_BAL_KP), 12.0f, 1e-6f),
          "NaN 不应写入 (实际 %.2f)", (double)param_get_float(PARAM_PID_BAL_KP));

    param_set_float(PARAM_PID_BAL_KP, 1.0f / 0.0f);
    CHECK(near(param_get_float(PARAM_PID_BAL_KP), 12.0f, 1e-6f),
          "Inf 不应写入 (实际 %.2f)", (double)param_get_float(PARAM_PID_BAL_KP));

    /* 非法 id */
    CHECK(param_set_float(PARAM_MAX, 1.0f) == ESP_ERR_INVALID_ARG, "非法 id 应报错");
    CHECK(param_get_float(PARAM_MAX) == 0.0f, "非法 id 读取应返回 0");
}

/* ================= 用例 3: JSON 批量设置 ================= */
static void test_from_json(void)
{
    banner("JSON 批量设置: 能写进去, 未知 key 不能吃掉后面的参数");
    param_reset_default();

    esp_err_t ret = param_from_json("{\"bal_kp\":20,\"bal_ki\":3,\"servo_c\":75}");
    CHECK(ret == ESP_OK, "合法 JSON 应返回 OK");
    CHECK(near(param_get_float(PARAM_PID_BAL_KP), 20.0f, 1e-6f), "bal_kp 应为 20");
    CHECK(near(param_get_float(PARAM_PID_BAL_KI),  3.0f, 1e-6f), "bal_ki 应为 3");
    CHECK(near(param_get_float(PARAM_SERVO_CENTER), 75.0f, 1e-6f), "servo_c 应为 75");

    /* 未知 key 夹在中间: 它后面的正常参数必须照样被读到。
     * 这是本次 bug 的同类风险: App 多带一个 "_v" 就把后面的参数全丢了 */
    param_reset_default();
    char json[128];
    snprintf(json, sizeof(json), "{\"_v\":%d,\"bal_kp\":15}",
             param_get_data_version());
    ret = param_from_json(json);
    CHECK(ret == ESP_OK, "带未知 key 的 JSON 应返回 OK");
    CHECK(near(param_get_float(PARAM_PID_BAL_KP), 15.0f, 1e-6f),
          "未知 key 后面的参数仍要生效 (实际 %.2f)",
          (double)param_get_float(PARAM_PID_BAL_KP));

    /* 空 / 非法输入不应崩 */
    CHECK(param_from_json("") == ESP_ERR_INVALID_ARG, "空串应报错");
    CHECK(param_from_json(NULL) == ESP_ERR_INVALID_ARG, "NULL 应报错");
    CHECK(param_from_json("{\"unknown_key\":1}") == ESP_ERR_INVALID_ARG,
          "只有未知 key 时应报错 (一个参数都没设上)");
}

/* ================= 用例 4: "_reset" 恢复默认 ================= */
static void test_reset_key(void)
{
    banner("\"_reset\" 恢复默认: 车端自己把整表打回默认值");
    param_reset_default();

    /* 先模拟一次"被 App 下发旧默认值"的事故现场 */
    param_set_float(PARAM_PID_SPD_KP, 20.0f);
    param_set_float(PARAM_PID_SPD_KI, 30.0f);
    param_set_float(PARAM_MOTOR_OUT_MAX, 30.0f);
    param_set_float(PARAM_PID_BAL_KP, 40.0f);
    CHECK(near(param_get_float(PARAM_PID_SPD_KP), 20.0f, 1e-6f), "前提: spd_kp 被改成 20");

    /* App 现在下发 {"_reset":true} */
    esp_err_t ret = param_from_json("{\"_reset\":true}");
    CHECK(ret == ESP_OK, "_reset 应返回 OK");
    /* 跟 g_param_items 里的 def 比, 不写死数值 (默认值会随仿真结果调整) */
    CHECK(near(param_get_float(PARAM_PID_SPD_KP),
               g_param_items[PARAM_PID_SPD_KP].def, 1e-6f),
          "reset 后 spd_kp 应回到默认 (实际 %.3f)",
          (double)param_get_float(PARAM_PID_SPD_KP));
    CHECK(near(param_get_float(PARAM_PID_SPD_KI),
               g_param_items[PARAM_PID_SPD_KI].def, 1e-6f),
          "reset 后 spd_ki 应回到默认 (实际 %.3f)",
          (double)param_get_float(PARAM_PID_SPD_KI));
    CHECK(near(param_get_float(PARAM_MOTOR_OUT_MAX), 60.0f, 1e-6f),
          "reset 后 motor_max 应回到默认 60");
    CHECK(near(param_get_float(PARAM_PID_BAL_KP), 12.0f, 1e-6f),
          "reset 后 bal_kp 应回到默认 12");

    /* 数字形式也要认 ({"_reset":1}) */
    param_set_float(PARAM_PID_SPD_KP, 20.0f);
    ret = param_from_json("{\"_reset\":1}");
    CHECK(ret == ESP_OK, "_reset:1 应返回 OK");
    CHECK(near(param_get_float(PARAM_PID_SPD_KP),
               g_param_items[PARAM_PID_SPD_KP].def, 1e-6f), "_reset:1 也应重置");

    /* _reset:0 表示"不重置" */
    param_set_float(PARAM_PID_SPD_KP, 20.0f);
    param_from_json("{\"_reset\":0}");
    CHECK(near(param_get_float(PARAM_PID_SPD_KP), 20.0f, 1e-6f), "_reset:0 不应重置");

    /* _reset 后面还能跟别的参数: 先重置, 再按后面的值覆盖 */
    param_set_float(PARAM_PID_SPD_KP, 20.0f);
    ret = param_from_json("{\"_reset\":true,\"bal_kp\":22}");
    CHECK(ret == ESP_OK, "_reset + 其它键应返回 OK");
    CHECK(near(param_get_float(PARAM_PID_BAL_KP), 22.0f, 1e-6f),
          "_reset 后面的 bal_kp 应为 22");
    CHECK(near(param_get_float(PARAM_PID_SPD_KP),
               g_param_items[PARAM_PID_SPD_KP].def, 1e-6f),
          "_reset 后 spd_kp 应为默认值");
}

/* ================= 用例 5: map JSON 带版本号 ================= */
static void test_map_json(void)
{
    banner("map JSON: 必须带 _v 版本号, 且长度不超平台限制");
    param_reset_default();

    char buf[512];
    int n = param_to_map_json(buf, sizeof(buf));
    CHECK(n > 0, "应生成 JSON");
    CHECK(buf[n] == '\0', "字符串应正确截断");

    /* 带 "_v" */
    CHECK(strstr(buf, "\"_v\":") != NULL, "map JSON 应带版本号 _v");

    /* 平台物模型里 values 字段限长 512, 超了平台会拒收整个应答 */
    CHECK(n < 512, "map JSON 长度 %d 应小于 512", n);

    /* 每个参数都要在里面 */
    for (int i = 0; i < PARAM_MAX; i++) {
        const param_item_t *it = &g_param_items[(param_id_t)i];
        char needle[24];
        snprintf(needle, sizeof(needle), "\"%s\":", it->key);
        CHECK(strstr(buf, needle) != NULL, "map JSON 应含 %s", it->key);
    }

    /* 版本号要跟 param_get_data_version() 一致 */
    char expect[24];
    snprintf(expect, sizeof(expect), "\"_v\":%u", (unsigned)param_get_data_version());
    CHECK(strstr(buf, expect) != NULL, "map JSON 的 _v 应等于 %u",
          (unsigned)param_get_data_version());
}

/* ================= 用例 6: 旧版 App 写参数的提示标记 ================= */

/**
 * @brief 读"旧版 App 写过参数"的标记
 * @return 1=有标记, 0=无标记, -1=取 map JSON 失败
 * @note  标记只通过 map JSON 的 "_w":1 字段对外可见, 这里按同样的方式读回来
 */
static int stale_flag(void)
{
    char buf[512];
    int n = param_to_map_json(buf, sizeof(buf));
    if (n <= 0) return -1;
    return strstr(buf, "\"_w\":1") != NULL;
}

static void test_stale_flag(void)
{
    banner("旧版 App 提示标记: 只提示不拦截, 标记随 _v 自动消解");
    param_reset_default();
    param_mark_stale_write(false);

    /* 没有 "_v" 的下发 (旧版 App) → 由上层 (onenet_client) 打标记 */
    param_mark_stale_write(true);
    CHECK(stale_flag() == 1, "旧版 App 写过参数后标记应为真");

    char buf[512];
    int n = param_to_map_json(buf, sizeof(buf));
    CHECK(n > 0 && strstr(buf, "\"_w\":1") != NULL,
          "map JSON 应带上 \"_w\":1 提示标记");

    /* 新版 App 下发 (带 _v = 当前版本) → 标记自动清掉
     * ⚠ 版本号从 param_get_data_version() 取, 不要写死数字:
     *   参数表版本一改 (PARAM_DATA_VERSION +1), 写死的版本就变成"旧版 App",
     *   测试会以很难看懂的方式失败。 */
    char json[128];
    snprintf(json, sizeof(json), "{\"_v\":%d,\"bal_kp\":12}",
             param_get_data_version());
    param_from_json(json);
    CHECK(stale_flag() == 0, "带正确 _v 的下发应清掉标记");

    /* App 版本比车端旧 → 重新标记 */
    param_from_json("{\"_v\":7,\"bal_kp\":12}");
    CHECK(stale_flag() == 1, "带旧版本号 _v 的下发应重新标记");

    /* 新版 App 恢复默认 (带 _v) → 也不该被当成旧版 */
    snprintf(json, sizeof(json), "{\"_reset\":true,\"_v\":%d}",
             param_get_data_version());
    param_from_json(json);
    CHECK(stale_flag() == 0, "带 _v 的恢复默认不应被标记");

    /* 标记状态下的 map JSON 长度仍然要够短 (平台限 512) */
    param_mark_stale_write(true);
    n = param_to_map_json(buf, sizeof(buf));
    CHECK(n > 0 && n < 512, "带标记的 map JSON 长度 %d 应小于 512", n);
    param_mark_stale_write(false);
}

/* ================= 用例 7: 步进吸附 (速度环增益) =================
 * 2026-10-26 的坑: spd_kp/spd_ki/spd_kd 的 step 是 0.5/0.5/0.1,
 * param_set_float() 用 snap_to_step() 把值吸附到 step 的整数倍:
 *   设 0.06 → min(0)+round(0.06/0.5)*0.5 = **0.0**
 *   设 0.3  → 0.5
 * → App 上拖出来的中间值一个都设不进去 (静默失效)。
 * step 改成 0.01 之后, 细一点的值必须能原样写进去。 */
static void test_step_snap(void)
{
    banner("步进吸附: 速度环增益的 step 必须是 0.01 (否则中间值被吸掉)");
    param_reset_default();

    /* step 本身要改对 */
    CHECK(near(g_param_items[PARAM_PID_SPD_KP].step, 0.01f, 1e-6f),
          "spd_kp 的 step 应为 0.01 (实际 %.3f)",
          (double)g_param_items[PARAM_PID_SPD_KP].step);
    CHECK(near(g_param_items[PARAM_PID_SPD_KI].step, 0.01f, 1e-6f),
          "spd_ki 的 step 应为 0.01 (实际 %.3f)",
          (double)g_param_items[PARAM_PID_SPD_KI].step);
    CHECK(near(g_param_items[PARAM_PID_SPD_KD].step, 0.01f, 1e-6f),
          "spd_kd 的 step 应为 0.01 (实际 %.3f)",
          (double)g_param_items[PARAM_PID_SPD_KD].step);

    /* 关键值: 0.06 不能再被吸成 0 */
    param_set_float(PARAM_PID_SPD_KP, 0.06f);
    CHECK(near(param_get_float(PARAM_PID_SPD_KP), 0.06f, 1e-6f),
          "spd_kp 设 0.06 应保持 0.06 (实际 %.3f)",
          (double)param_get_float(PARAM_PID_SPD_KP));

    param_set_float(PARAM_PID_SPD_KI, 0.02f);
    CHECK(near(param_get_float(PARAM_PID_SPD_KI), 0.02f, 1e-6f),
          "spd_ki 设 0.02 应保持 0.02 (实际 %.3f)",
          (double)param_get_float(PARAM_PID_SPD_KI));

    param_set_float(PARAM_PID_SPD_KD, 0.03f);
    CHECK(near(param_get_float(PARAM_PID_SPD_KD), 0.03f, 1e-6f),
          "spd_kd 设 0.03 应保持 0.03 (实际 %.3f)",
          (double)param_get_float(PARAM_PID_SPD_KD));

    /* 走 JSON 下发的路径 (App 实际走这条) 也要一样 */
    param_reset_default();
    param_from_json("{\"spd_kp\":0.06,\"spd_ki\":0.02,\"spd_kd\":0.03}");
    CHECK(near(param_get_float(PARAM_PID_SPD_KP), 0.06f, 1e-6f),
          "JSON 下发 spd_kp=0.06 应保持 0.06 (实际 %.3f)",
          (double)param_get_float(PARAM_PID_SPD_KP));
    CHECK(near(param_get_float(PARAM_PID_SPD_KI), 0.02f, 1e-6f),
          "JSON 下发 spd_ki=0.02 应保持 0.02 (实际 %.3f)",
          (double)param_get_float(PARAM_PID_SPD_KI));
    CHECK(near(param_get_float(PARAM_PID_SPD_KD), 0.03f, 1e-6f),
          "JSON 下发 spd_kd=0.03 应保持 0.03 (实际 %.3f)",
          (double)param_get_float(PARAM_PID_SPD_KD));
}

int main(void)
{
    printf("参数表 param.c 离线测试\n");

    test_defaults();
    test_clamp();
    test_from_json();
    test_reset_key();
    test_map_json();
    test_stale_flag();
    test_step_snap();

    printf("\n========== %d 通过, %d 失败 ==========\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
