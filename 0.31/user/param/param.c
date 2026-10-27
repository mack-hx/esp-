/**
 * @file param.c
 * @brief 全局可调参数表实现 (内存数组 + NVS 持久化)
 *
 * 0.13 起, 19 → 16 个参数 (删除 ang_zero / bal_sign / gyro_sign)
 * PARAM_DATA_VERSION 6 → 7
 * 0.31 起, 16 → 17 个参数 (新增 move_rpm_max), PARAM_DATA_VERSION = 14
 * 0.31 位置环: 17 → 20 个参数 (新增 wheel_dia / pos_kp / pos_ki),
 * PARAM_DATA_VERSION = 15
 * 0.31 位置环默认关闭 (pos_kp / pos_ki 默认 0), PARAM_DATA_VERSION = 16
 *
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-07
 */

#include "param.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef PARAM_OFFLINE_TEST
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_log.h>
static const char *TAG = "PARAM";
#endif

/* ========== 存储常量 ========== */
#ifndef PARAM_OFFLINE_TEST
#define NVS_NAMESPACE   "param"
#define NVS_KEY_VALUES  "v"     /* 存所有 float 的 blob */
#define NVS_KEY_VERSION "ver"   /* 表结构版本, 表变了就忽略旧数据 */
#define NVS_KEY_STALE   "stale" /* 标记: 最近一次写参数来自旧版 App */
#endif
#define PARAM_DATA_VERSION  16  /* 每次增删参数就 +1
                                 * v2~v5: 0.1 的历史版本 (已作废)
                                 * v6:    0.11 删掉转向环/servo_l/pwm_max/deadband 等
                                 * v7:    0.13 再删 ang_zero/bal_sign/gyro_sign (烘焙成常量)
                                 * v8:    0.30 平衡环 D 项符号修正 + 速度环默认关,
                                 *         旧默认值 (Kp40/Ki12/死区8) 必须作废,
                                 *         否则 NVS 里的旧值会让新代码继续按错的参数跑
                                 * v9:    0.30 电机方向修正后 motor_max 默认 30→60,
                                 *         同样要作废旧 blob
                                 * v10:   0.30 App 的"恢复默认"曾把 v7 的旧默认值
                                 *         (spd_kp=20/spd_ki=30/motor_max=30/deadzone=8)
                                 *         写回车端并落盘 → 速度环被意外打开, 车一直朝一个
                                 *         方向跑。这里再作废一次 blob, 让这些坏值失效
                                 * v11:   0.30 "一碰就剧烈抖" 的修复: gyro_lpf 默认
                                 *         0.2→0.8 (陀螺低通的延迟加在 D 项上 = 加在
                                 *         唯一的阻尼源上), 死区补偿方式也改了。
                                 *         blob 里的旧值会让新代码继续用 0.2 → 再作废一次
                                 * v12:   0.30 App 方向键改走速度环: 速度环默认增益从 0
                                 *         改成 spd_kp=0.15 / spd_ki=0.05 (之前是 0,
                                 *         速度环等于没开, 方向键只能走直驱)。
                                 *         blob 里的旧值会让方向键没反应 → 再作废一次
                                 * v13:   0.30 速度环增益按仿真结果调保守:
                                 *         spd_kp 0.15→0.06, spd_ki 0.05→0.01,
                                 *         走路时目标倾角上限 12°→4°。
                                 *         (0.15/0.05 仿真里车身倾角波动中位 10.7°,
                                 *          走起来就是"剧烈抖")
                                 * v14:   0.31 减速比默认改成 30 (编码器装在**电机轴**上,
                                 *         减速箱 30:1; 之前默认 1.0 → 轮速被算大 30 倍,
                                 *         速度环直接饱和, 车自己跑偏),
                                 *         并新增"走路目标轮速上限" move_rpm_max。
                                 *         NVS 里的旧 blob 必须作废 —— 否则车会继续
                                 *         用旧的 gear=1 跑, 速度环一开就饱和。
                                 * v15:   0.31 位置环: 新增 wheel_dia (轮径 6.5 cm) /
                                 *         pos_kp / pos_ki 三个参数。
                                 *         NVS blob 布局变了, 旧数据必须作废
                                 * v16:   0.31 位置环**默认关闭**: pos_kp / pos_ki 默认
                                 *         改成 0 (之前是 1.0 / 0.05)。
                                 *         原因: 真机上"站直时车身抖动"排查下来, 位置环
                                 *         是 0.30→0.31 唯一新增的闭环 —— 站直时车必然
                                 *         缓慢漂移, 漂过 1cm 死区位置环就开始纠偏
                                 *         (积分项能到 ±10rpm) → 目标轮速被顶出去 →
                                 *         速度环积分跟着追 → 车前后摆。
                                 *         位置环先按"默认关, 需要时从 App 打开"处理。
                                 *         NVS blob 内容变了, 旧数据必须作废
                                 * NVS blob 布局变了, 旧数据必须作废
                                 *
                                 * ⚠ 2026-10-26 收尾: spd_kp/spd_ki/spd_kd 的 step
                                 *   从 0.5/0.5/0.1 改成 0.01 (App 上拖出来的中间值被吸附掉,
                                 *   静默设不进去)。step 只影响"怎么吸附",
                                 *   **不改变 blob 布局和默认值** → 版本号仍然是 14, 不动。
                                 *   (App 侧 PARAM_VER 也仍然是 14, 两边必须同步改 step) */

/* 参数值数组 (下标 = param_id_t, 与 g_param_items 严格一一对应) */
static float s_values[PARAM_MAX];

/* 标记: 最近一次参数写入来自"过期版本的 App" (下发里没有 "_v", 或 "_v" 比车端版本旧)。
 * 只做提示用, **不拦截写入** (2026-10-26 用户要求: 提示即可, 不拒绝)。
 * 这个标记随 param_get 的 "_w" 字段带给 App, App 参数页据此显示黄色提示条,
 * 用户就知道"参数可能被旧版 App 改坏了, 建议点一次恢复默认"。
 * 标记会跟参数一起存进 NVS, 断电重启也保留 (旧 App 写完还会调 param_save)。 */
static bool s_stale_write = false;

/* ========== 参数描述表 ========== */
const param_item_t g_param_items[] = {
    /* ---------- 平衡环 (直立) ---------- */
    { PARAM_PID_BAL_KP, "bal_kp", "Balance Kp", "",  12.0f,   0.0f,  200.0f,  0.5f, 1, PARAM_GRP_BALANCE,
      "越大越硬。太小扶不倒, 太大高频抖。这是**最需要真机试出来的**参数" },
    { PARAM_PID_BAL_KI, "bal_ki", "Balance Ki", "",   0.5f,   0.0f,  100.0f,  0.5f, 1, PARAM_GRP_BALANCE,
      "消除配平残余偏差。太大容易积分饱和 (车会先猛冲再回不来)" },
    { PARAM_PID_BAL_KD, "bal_kd", "Balance Kd", "",   1.5f,   0.0f,   30.0f,  0.1f, 1, PARAM_GRP_BALANCE,
      "角速度阻尼。车晃得停不下来就加大, 出现高频嗡嗡声就减小" },

    /* ---------- 速度环 (前进后退) ----------
     * ⚠ 2026-10-26: 下面三个增益的 step 从 0.5/0.5/0.1 改成 **0.01**。
     * 原因: param_set_float() 会用 snap_to_step() 把下发值吸附到 step 的整数倍,
     *   step=0.5 时想设 0.06 → min(0)+round(0.06/0.5)*0.5 = **0.0**,
     *   想设 0.3 → 0.5 —— App 上拖出来的中间值**一个都设不进去** (静默失效)。
     *   (设成 0 还不算彻底坏: user/pid/speed_pid.h 里有兜底常量 SPD_PID_KP_DEFAULT=0.06,
     *    看起来"还能用", 但用户根本调不动。)
     * ⚠ step 只是"怎么吸附", 不影响 blob 布局和默认值 → PARAM_DATA_VERSION 不用动。 */
    { PARAM_PID_SPD_KP, "spd_kp", "Speed Kp",   "",   0.06f,   0.0f,  200.0f,  0.01f, 2, PARAM_GRP_SPEED,
      "走多快 (°/rpm)。App 方向键就走这个环: 目标轮速 → 目标倾角 → "
      "平衡环 (平衡环全程不撒手)。调大走得快, 但目标倾角被限制在 4° 以内 "
      "(安全阀), 调太大只会让车左右猛摇。步进 0.01 —— 步进太粗的话 "
      "App 拖出来的中间值会被吸附掉 (比如想设 0.06 却被吸成 0)" },
    { PARAM_PID_SPD_KI, "spd_ki", "Speed Ki",   "",   0.01f,   0.0f,  300.0f,  0.01f, 2, PARAM_GRP_SPEED,
      "消除稳态速度误差 (走不到目标速度时慢慢补倾角)。太大容易积分饱和, "
      "也会让车越走越歪。步进 0.01 (步进太粗的话中间值会被吸附掉)" },
    { PARAM_PID_SPD_KD, "spd_kd", "Speed Kd",   "",   0.0f,    0.0f,   30.0f,  0.01f, 2, PARAM_GRP_SPEED,
      "速度变化阻尼。默认 0: 编码器量化台阶会让 D 项噪声很大。步进 0.01" },

    /* ---------- 限幅 / 摔倒 / 滤波 / 减速比 ---------- */
    { PARAM_MOTOR_OUT_MAX, "motor_max", "Motor Out Max", "%", 60.0f,  0.0f,  100.0f, 1.0f, 0, PARAM_GRP_LIMIT,
      "电机总输出上限。2026-10-25 起默认 60%: 方向修正后实测能自己站住, "
      "但倾角大一点就要更大出力才接得住 (Kp=12 时 5° 就顶格)" },
    { PARAM_SPEED_OUT_MAX, "spdout_max", "Speed Out Max", "°", 40.0f, 0.0f,  100.0f, 1.0f, 0, PARAM_GRP_LIMIT,
      "速度环能给出的最大倾角目标 (°)。车本来就有配平偏差时别调太大" },
    { PARAM_ANGLE_LIMIT, "ang_limit", "Angle Limit", "°", 25.0f,  5.0f,   60.0f, 1.0f, 0, PARAM_GRP_LIMIT,
      "超过这个倾角判定摔倒, 立刻停电机" },
    { PARAM_GYRO_LPF,    "gyro_lpf",  "Gyro LPF",    "",  0.8f,   0.0f,    1.0f, 0.01f,2, PARAM_GRP_LIMIT,
      "陀螺仪低通系数 (D 项用的角速度)。**越小越平滑但延迟越大, "
      "延迟大了阻尼会变弱 → 一碰就剧烈抖**。默认 0.8 (时间常数 12ms): "
      "MPU 自带的 41Hz 数字低通已经够用, 这里再滤只是白白加延迟" },
    { PARAM_ENCODER_GEAR,"gear",      "Gear Ratio",  "",  30.0f,  0.1f,   50.0f, 0.1f, 1, PARAM_GRP_LIMIT,
      "电机转速 / 轮转速。本机编码器装在**电机轴**上, 减速箱 30:1, 所以填 30。"
      "填错会让轮速算错 30 倍 (速度环直接饱和, 车自己跑偏)" },

    /* ---------- 电机死区 ---------- */
    { PARAM_MOTOR_DEADBAND, "deadzone", "Motor Deadband", "%", 4.0f, 0.0f, 30.0f, 1.0f, 0, PARAM_GRP_LIMIT,
      "电机最小启动 PWM (硬件死区)。**要填电机真正能转起来的最小占空比**, "
      "填小了补偿不够 (小输出时电机不转, 车会突然猛冲), 填大了会一直有底噪扭矩。 "
      "2026-10-26 起补偿改成'加上'死区, 所以这个值填得越准越好" },

    /* ---------- 启动门控 ---------- */
    { PARAM_ARM_PITCH,   "arm_pitch", "Arm Pitch",   "°",   8.0f,  1.0f,   30.0f, 1.0f, 0, PARAM_GRP_LIMIT,
      "启动门控: 倾角小于这个值才允许输出。防止歪着启动直接摔" },
    { PARAM_ARM_RATE,    "arm_rate",  "Arm Rate",    "°/s", 20.0f,  2.0f,  100.0f, 1.0f, 0, PARAM_GRP_LIMIT,
      "启动门控: 角速度小于这个值才允许输出。防止推着车启动" },
    { PARAM_ARM_TIME,    "arm_time",  "Arm Time",    "ms", 500.0f,  0.0f, 3000.0f,100.0f,0, PARAM_GRP_LIMIT,
      "启动门控: 需要连续稳定这么久才真正输出。0=不等(不推荐)" },

    /* ---------- 舵机 ---------- */
    { PARAM_SERVO_CENTER, "servo_c",  "Servo Center", "°", 80.0f,  30.0f,  120.0f, 1.0f, 0, PARAM_GRP_LIMIT,
      "舵机中心角。也是配平表的参考角 (补偿在这一档为 0)" },

    /* ---------- 走路目标轮速上限 (0.31 新增, 追加在末尾) ---------- */
    { PARAM_MOVE_RPM_MAX, "move_rpm_max", "Move RPM Max", "rpm", 100.0f, 0.0f, 333.0f, 1.0f, 1, PARAM_GRP_LIMIT,
      "App 方向键 100% 档位对应的目标轮速 (rpm)。本机轮子最高转速 333 rpm, 默认取 100 (约 30%); "
      "再大目标倾角会顶到 4° 安全阀, 车会左右猛摇。0 = 用代码里的默认值" },

    /* ---------- 里程计 / 位置环 (0.31 新增, 追加在末尾) ---------- */
    { PARAM_WHEEL_DIA, "wheel_dia", "Wheel Diameter", "cm", 6.5f, 1.0f, 30.0f, 0.1f, 1, PARAM_GRP_LIMIT,
      "**轮子直径** (cm)。里程计靠它把编码器脉冲换算成位移: "
      "距离 = 脉冲数 / (44 × gear) × π × 轮径。本机实测 6.5 cm。"
      "填错会让位置反馈整体缩放错误 (位置环纠偏力度不对)" },
    { PARAM_POS_KP, "pos_kp", "Position Kp", "rpm/cm", 0.0f, 0.0f, 50.0f, 0.1f, 2, PARAM_GRP_SPEED,
      "位置环比例增益 (rpm/cm): 位置偏差 1 cm 时给速度环加多少 rpm 的纠偏。"
      "位置环在速度环外面 (位置误差 → 目标轮速修正 → 速度环 → 平衡环)。"
      "**默认 0 = 位置环关闭** (站直抖动排查的结论: 位置环默认开会引起前后摆)。"
      "要用就把 pos_kp 从 0 往上慢慢加, 一次加一点, 推一下车看是否走回原位" },
    { PARAM_POS_KI, "pos_ki", "Position Ki", "rpm/(cm·s)", 0.0f, 0.0f, 10.0f, 0.01f, 2, PARAM_GRP_SPEED,
      "位置环积分增益: 消除站住之后慢慢偏这种位置漂移。"
      "**默认 0**。太大容易积分饱和 (车会突然窜出去找位置)。步进 0.01" },
};

/**
 * @brief 把某值按 step 对齐 (避免滑块传出 12.37 这种值)
 */
static float snap_to_step(float val, float min, float step)
{
    if (step <= 0.0f) return val;
    return min + roundf((val - min) / step) * step;
}

esp_err_t param_reset_default(void)
{
    for (int i = 0; i < PARAM_ITEM_COUNT; i++) {
        s_values[i] = g_param_items[i].def;
    }
    return ESP_OK;
}

#ifndef PARAM_OFFLINE_TEST   /* 离线测试不编译 NVS 这段, 见 test/test_param_pc.c */

esp_err_t param_load(void)
{
    param_reset_default();

    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "NVS 无数据, 用默认值");
        return ESP_OK;
    }

    /* 检查表结构版本: 表改过就丢弃旧数据, 用新默认值。
     * 必须在读 blob 之前查 —— 旧 blob 的下标顺序和现在不一样了, 读了会错位 */
    uint16_t ver = 0;
    size_t len = sizeof(ver);
    if (nvs_get_u16(h, NVS_KEY_VERSION, &ver) != ESP_OK || ver != PARAM_DATA_VERSION) {
        ESP_LOGW(TAG, "参数表版本不匹配 (%u != %d), 用默认值", ver, PARAM_DATA_VERSION);
        nvs_close(h);
        /* 顺手把默认值写回去: 不然这份旧 blob 会一直留在 NVS 里,
         * 每次开机都刷这条告警。写回去之后状态就收敛了。 */
        param_save();
        return ESP_OK;
    }

    /* 读 blob, 长度必须正好等于当前参数个数 */
    float tmp[PARAM_MAX];
    len = sizeof(tmp);
    if (nvs_get_blob(h, NVS_KEY_VALUES, tmp, &len) != ESP_OK || len != sizeof(tmp)) {
        ESP_LOGW(TAG, "参数 blob 长度不符, 用默认值");
        nvs_close(h);
        return ESP_OK;
    }

    for (int i = 0; i < PARAM_ITEM_COUNT; i++) {
        s_values[i] = tmp[i];
    }

    /* 读"旧版 App 写过参数"的提示标记 (没有这个键 = 正常, 标记为 0) */
    uint8_t stale = 0;
    if (nvs_get_u8(h, NVS_KEY_STALE, &stale) == ESP_OK) {
        s_stale_write = (stale != 0);
    }

    nvs_close(h);

    ESP_LOGI(TAG, "已从 NVS 载入 %d 个参数", PARAM_ITEM_COUNT);
    return ESP_OK;
}

esp_err_t param_save(void)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_blob(h, NVS_KEY_VALUES, s_values, sizeof(s_values));
    if (ret == ESP_OK) {
        uint16_t ver = PARAM_DATA_VERSION;
        ret = nvs_set_u16(h, NVS_KEY_VERSION, ver);
    }
    if (ret == ESP_OK) {
        ret = nvs_set_u8(h, NVS_KEY_STALE, s_stale_write ? 1 : 0);
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(h);
    }
    nvs_close(h);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "参数已保存到 NVS");
    } else {
        ESP_LOGE(TAG, "参数保存失败: %s", esp_err_to_name(ret));
    }
    return ret;
}

#endif // PARAM_OFFLINE_TEST

const param_item_t *param_get_item(param_id_t id)
{
    if ((int)id < 0 || (int)id >= PARAM_ITEM_COUNT) return NULL;
    return &g_param_items[id];
}

uint8_t param_count(void)
{
    return PARAM_ITEM_COUNT;
}

uint16_t param_get_data_version(void)
{
    return PARAM_DATA_VERSION;
}

void param_mark_stale_write(bool on)
{
    s_stale_write = on;
}

bool param_stale_write(void)
{
    return s_stale_write;
}

float param_get_float(param_id_t id)
{
    if ((int)id < 0 || (int)id >= PARAM_ITEM_COUNT) return 0.0f;
    return s_values[id];
}

esp_err_t param_set_float(param_id_t id, float val)
{
    if ((int)id < 0 || (int)id >= PARAM_ITEM_COUNT) return ESP_ERR_INVALID_ARG;
    /* NaN/Inf 必须挡住: 它们能通过 clamp, 进了 PID 就会静默算出垃圾输出 */
    if (isnan(val) || isinf(val)) return ESP_ERR_INVALID_ARG;

    const param_item_t *it = &g_param_items[id];
    if (val < it->min) val = it->min;
    if (val > it->max) val = it->max;
    s_values[id] = snap_to_step(val, it->min, it->step);
    return ESP_OK;
}

int param_to_json(char *buf, uint32_t len)
{
    if (buf == NULL || len == 0) return 0;
    int n = snprintf(buf, len, "{\"n\":%u,\"items\":[", (unsigned)PARAM_ITEM_COUNT);
    if (n < 0 || (uint32_t)n >= len) return 0;

    for (int i = 0; i < PARAM_ITEM_COUNT; i++) {
        const param_item_t *it = &g_param_items[i];
        int w = snprintf(buf + n, len - n,
                         "%s{\"key\":\"%s\",\"label\":\"%s\",\"unit\":\"%s\","
                         "\"v\":%.*f,\"def\":%.*f,\"min\":%.*f,\"max\":%.*f,"
                         "\"step\":%.*f,\"grp\":%d,\"hint\":\"%s\"}",
                         i ? "," : "",
                         it->key, it->label, it->unit,
                         it->decimals, (double)s_values[i],
                         it->decimals, (double)it->def,
                         it->decimals, (double)it->min,
                         it->decimals, (double)it->max,
                         it->decimals, (double)it->step,
                         (int)it->grp,
                         it->hint);
        if (w <= 0 || (uint32_t)(n + w) >= len) return n;   /* 缓冲区不够, 截断 */
        n += w;
    }

    if ((uint32_t)(n + 2) < len) {
        buf[n++] = ']';
        buf[n++] = '}';
        buf[n]   = '\0';
    }
    return n;
}

int param_to_map_json(char *buf, uint32_t len)
{
    if (buf == NULL || len == 0) return 0;
    int n = snprintf(buf, len, "{");
    if (n < 0 || (uint32_t)n >= len) return 0;

    for (int i = 0; i < PARAM_ITEM_COUNT; i++) {
        const param_item_t *it = &g_param_items[i];
        int w = snprintf(buf + n, len - n, "%s\"%s\":%.*f",
                         i ? "," : "", it->key,
                         it->decimals, (double)s_values[i]);
        if (w <= 0 || (uint32_t)(n + w) >= len) return n;   /* 缓冲区不够, 截断 */
        n += w;
    }

    /* 带上参数表版本号, 让 App 能判断自己内置的默认值表是不是过期了。
     * 2026-10-26: App 的"恢复默认"下发的是 App 本地的旧默认值 (v7 那一套:
     * spd_kp=20/spd_ki=30/motor_max=30/deadzone=8), 把车端调好的参数覆盖掉,
     * 速度环被意外打开 → 车一直朝一个方向跑。
     * App 拿到这个版本号, 跟自己内置的不一致就该提示用户, 而不是闷头下发。 */
    {
        int w = snprintf(buf + n, len - n, ",\"_v\":%d", PARAM_DATA_VERSION);
        if (w <= 0 || (uint32_t)(n + w) >= len) return n;
        n += w;
    }

    /* 提示标记: 最近一次写参数来自旧版 App (只提示, 车端不拦截)。
     * App 拿到 "_w" 就在参数页显示提示条, 让用户去点"恢复默认"。 */
    if (s_stale_write) {
        int w = snprintf(buf + n, len - n, ",\"_w\":1");
        if (w <= 0 || (uint32_t)(n + w) >= len) return n;
        n += w;
    }

    if ((uint32_t)(n + 2) < len) {
        buf[n++] = '}';
        buf[n]   = '\0';
    }
    return n;
}

/* ================= JSON 解析 (只认 "key": 数字 这种最简形式) =================
 * 不用 cJSON / mbedtls: 调参数据全是 ASCII 数字 + 引号, 手写扫描 60 行搞定,
 * 省一个组件依赖。格式示例: {"bal_kp":12.5,"bal_ki":3,"servo_c":75}
 *
 * ⚠ 特殊键 "_reset": 值为 1 / true 时, 先把整张表恢复成**车端自己的默认值**,
 *   再继续应用后面的其它键。
 *   2026-10-26: App 的"恢复默认"以前是把 App 内置的默认值表整份下发,
 *   而那份表早就跟车端脱节 (还是 v7 的老值), 结果把调好的参数覆盖成
 *   "速度环开着 + 输出上限 30%" → 车只朝一个方向转, 根本稳不住。
 *   改成让车端自己恢复默认值, App 就再也不会下发过期数据。 */
esp_err_t param_from_json(const char *json)
{
    if (json == NULL) return ESP_ERR_INVALID_ARG;

    const char *p = json;
    int applied = 0;

    while ((p = strchr(p, '"')) != NULL) {
        p++;                                    /* 跳过开引号 */
        const char *key = p;
        const char *key_end = strchr(p, '"');
        if (key_end == NULL) break;
        p = key_end + 1;

        /* 找冒号 */
        while (*p == ' ') p++;
        if (*p != ':') continue;
        p++;
        while (*p == ' ') p++;

        /* 按 key 找对应参数, 找到就吃后面的数字 */
        char name[16];
        size_t klen = (size_t)(key_end - key);
        if (klen == 0 || klen >= sizeof(name)) continue;
        memcpy(name, key, klen);
        name[klen] = '\0';

        /* --- 特殊键: "_reset" → 用车端默认值整表重置 --- */
        if (strcmp(name, "_reset") == 0) {
            if (strncmp(p, "true", 4) == 0) {
                param_reset_default();
                applied++;
                p += 4;
            } else {
                char *endp = NULL;
                float v = strtof(p, &endp);
                if (endp == p) break;           /* 数值非法 */
                if (v != 0.0f) {
                    param_reset_default();
                    applied++;
                }
                p = endp;
            }
            continue;
        }

        /* --- 特殊键: "_v" → App 内置的参数表版本号, 只用来判断是不是旧版 App --- */
        if (strcmp(name, "_v") == 0) {
            char *endp = NULL;
            float v = strtof(p, &endp);
            if (endp == p) break;               /* 数值非法 */
            /* 版本比车端旧 → 标记一下 (只提示, 不拒绝写入) */
            s_stale_write = ((int)v < PARAM_DATA_VERSION);
            p = endp;
            continue;
        }

        /* 未知 key (比如其它保留键) 直接忽略: 下面的 for 循环找不到同名参数
         * 就什么也不做, p 停在数值处, 外层 while 继续找下一个引号。 */

        for (int i = 0; i < PARAM_ITEM_COUNT; i++) {
            if (strcmp(g_param_items[i].key, name) != 0) continue;

            /* strtof 会自己跳过前导空格, 遇到非法字符返回 0 */
            char *endp = NULL;
            float v = strtof(p, &endp);
            if (endp == p) break;               /* 数值非法 */
            if (param_set_float((param_id_t)i, v) == ESP_OK) applied++;
            p = endp;
            break;
        }
    }

#ifndef PARAM_OFFLINE_TEST
    ESP_LOGI(TAG, "批量设置参数 %d 个", applied);
#endif
    return (applied > 0) ? ESP_OK : ESP_ERR_INVALID_ARG;
}
