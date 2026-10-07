/**
 * @file param.c
 * @brief 全局可调参数表实现 (内存数组 + NVS 持久化)
 *
 * 1.3 起, 19 → 16 个参数 (删除 ang_zero / bal_sign / gyro_sign)
 * PARAM_DATA_VERSION 6 → 7
 *
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-07
 */

#include "param.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_log.h>

static const char *TAG = "PARAM";

/* ========== 存储常量 ========== */
#define NVS_NAMESPACE   "param"
#define NVS_KEY_VALUES  "v"     /* 存所有 float 的 blob */
#define NVS_KEY_VERSION "ver"   /* 表结构版本, 表变了就忽略旧数据 */
#define PARAM_DATA_VERSION  7   /* 每次增删参数就 +1
                                 * v2~v5: 1.0 的历史版本 (已作废)
                                 * v6:    1.1 删掉转向环/servo_l/pwm_max/deadband 等
                                 * v7:    1.3 再删 ang_zero/bal_sign/gyro_sign (烘焙成常量)
                                 * NVS blob 布局变了, 旧数据必须作废 */

/* 参数值数组 (下标 = param_id_t, 与 g_param_items 严格一一对应) */
static float s_values[PARAM_MAX];

/* ========== 参数描述表 ========== */
const param_item_t g_param_items[] = {
    /* ---------- 平衡环 (直立) ---------- */
    { PARAM_PID_BAL_KP, "bal_kp", "Balance Kp", "",  40.0f,   0.0f,  200.0f,  0.5f, 1, PARAM_GRP_BALANCE,
      "越大越硬。太小扶不倒, 太大高频抖。这是**最需要真机试出来的**参数" },
    { PARAM_PID_BAL_KI, "bal_ki", "Balance Ki", "",  12.0f,   0.0f,  100.0f,  0.5f, 1, PARAM_GRP_BALANCE,
      "消除配平残余偏差。太大容易积分饱和 (车会先猛冲再回不来)" },
    { PARAM_PID_BAL_KD, "bal_kd", "Balance Kd", "",   2.0f,   0.0f,   30.0f,  0.1f, 1, PARAM_GRP_BALANCE,
      "角速度阻尼。车晃得停不下来就加大, 出现高频嗡嗡声就减小" },

    /* ---------- 速度环 (前进后退) ---------- */
    { PARAM_PID_SPD_KP, "spd_kp", "Speed Kp",   "",  20.0f,   0.0f,  200.0f,  0.5f, 1, PARAM_GRP_SPEED,
      "走多快。太大车会猛地窜出去" },
    { PARAM_PID_SPD_KI, "spd_ki", "Speed Ki",   "",  30.0f,   0.0f,  300.0f,  0.5f, 1, PARAM_GRP_SPEED,
      "消除稳态速度误差" },
    { PARAM_PID_SPD_KD, "spd_kd", "Speed Kd",   "",   0.0f,   0.0f,   30.0f,  0.1f, 1, PARAM_GRP_SPEED,
      "速度变化阻尼" },

    /* ---------- 限幅 / 摔倒 / 滤波 / 减速比 ---------- */
    { PARAM_MOTOR_OUT_MAX, "motor_max", "Motor Out Max", "%", 30.0f,  0.0f,  100.0f, 1.0f, 0, PARAM_GRP_LIMIT,
      "电机总输出上限。**默认只给 30% 是故意的**: 调试时宁小勿大, 站起来了再往上加" },
    { PARAM_SPEED_OUT_MAX, "spdout_max", "Speed Out Max", "°", 40.0f, 0.0f,  100.0f, 1.0f, 0, PARAM_GRP_LIMIT,
      "速度环能给出的最大倾角目标 (°)。车本来就有配平偏差时别调太大" },
    { PARAM_ANGLE_LIMIT, "ang_limit", "Angle Limit", "°", 25.0f,  5.0f,   60.0f, 1.0f, 0, PARAM_GRP_LIMIT,
      "超过这个倾角判定摔倒, 立刻停电机" },
    { PARAM_GYRO_LPF,    "gyro_lpf",  "Gyro LPF",    "",  0.2f,   0.0f,    1.0f, 0.01f,2, PARAM_GRP_LIMIT,
      "陀螺仪低通系数。越小越平滑但延迟越大, 延迟大了阻尼会变弱" },
    { PARAM_ENCODER_GEAR,"gear",      "Gear Ratio",  "",  1.0f,   0.1f,   50.0f, 0.1f, 1, PARAM_GRP_LIMIT,
      "电机转速 / 轮转速。带减速箱的填 >1" },

    /* ---------- 电机死区 ---------- */
    { PARAM_MOTOR_DEADBAND, "deadzone", "Motor Deadband", "%", 8.0f, 0.0f, 30.0f, 1.0f, 0, PARAM_GRP_LIMIT,
      "电机最小启动 PWM。太小电机转不动 (车会慢慢歪然后突然猛冲), 太大车会一顿一顿" },

    /* ---------- 启动门控 ---------- */
    { PARAM_ARM_PITCH,   "arm_pitch", "Arm Pitch",   "°",   8.0f,  1.0f,   30.0f, 1.0f, 0, PARAM_GRP_LIMIT,
      "启动门控: 倾角小于这个值才允许输出。防止歪着启动直接摔" },
    { PARAM_ARM_RATE,    "arm_rate",  "Arm Rate",    "°/s", 20.0f,  2.0f,  100.0f, 1.0f, 0, PARAM_GRP_LIMIT,
      "启动门控: 角速度小于这个值才允许输出。防止推着车启动" },
    { PARAM_ARM_TIME,    "arm_time",  "Arm Time",    "ms", 500.0f,  0.0f, 3000.0f,100.0f,0, PARAM_GRP_LIMIT,
      "启动门控: 需要连续稳定这么久才真正输出。0=不等(不推荐)" },

    /* ---------- 舵机 ---------- */
    { PARAM_SERVO_CENTER, "servo_c",  "Servo Center", "°", 80.0f,  30.0f,  100.0f, 1.0f, 0, PARAM_GRP_LIMIT,
      "舵机中心角。也是配平表的参考角 (补偿在这一档为 0)" },
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

const param_item_t *param_get_item(param_id_t id)
{
    if ((int)id < 0 || (int)id >= PARAM_ITEM_COUNT) return NULL;
    return &g_param_items[id];
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

/* ================= JSON 解析 (只认 "key": 数字 这种最简形式) =================
 * 不用 cJSON / mbedtls: 调参数据全是 ASCII 数字 + 引号, 手写扫描 60 行搞定,
 * 省一个组件依赖。格式示例: {"bal_kp":12.5,"bal_ki":3,"servo_c":75} */
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

    ESP_LOGI(TAG, "批量设置参数 %d 个", applied);
    return (applied > 0) ? ESP_OK : ESP_ERR_INVALID_ARG;
}
