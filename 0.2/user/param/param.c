/**
 * @file param.c
 * @brief 全局可调参数表实现 (内存数组 + NVS 持久化)
 * @author ESP32 机械狗项目
 * @date 2026-10-04
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
#define NVS_KEY_VALUES  "v"         /* 存所有 float 的 blob */
#define NVS_KEY_VERSION "ver"       /* 表结构版本, 表变了就忽略旧数据 */
#define PARAM_DATA_VERSION  1       /* 每次改 g_param_items 的数量/含义就 +1 */

/* 参数值数组 (下标 = param_id_t, 与 g_param_items 严格一一对应) */
static float s_values[PARAM_MAX];

/* ========== 参数描述表 ==========
 * 以后加参数: 在 param.h 的 PARAM_ID 加枚举, 然后在这里加一行, 网页自动出现新滑块 */
const param_item_t g_param_items[] = {
    /* ---------- 平衡环 (直立) ---------- */
    { PARAM_PID_BAL_KP,  "bal_kp",  "Balance Kp",  "",  40.0f,   0.0f,  200.0f,  0.5f, 1, PARAM_GRP_BALANCE,
      "越大越硬, 太小扶不倒, 太大高频抖" },
    { PARAM_PID_BAL_KI,  "bal_ki",  "Balance Ki",  "",  12.0f,   0.0f,  100.0f,  0.5f, 1, PARAM_GRP_BALANCE,
      "消除持续倾斜, 太大容易积分饱和" },
    { PARAM_PID_BAL_KD,  "bal_kd",  "Balance Kd",  "",   2.0f,   0.0f,   30.0f,  0.1f, 1, PARAM_GRP_BALANCE,
      "角速度阻尼, 抑制超调振荡" },
    /* ---------- 速度环 (前进后退) ---------- */
    { PARAM_PID_SPD_KP,  "spd_kp",  "Speed Kp",    "",  20.0f,   0.0f,  200.0f,  0.5f, 1, PARAM_GRP_SPEED,
      "速度响应快慢" },
    { PARAM_PID_SPD_KI,  "spd_ki",  "Speed Ki",    "",  30.0f,   0.0f,  300.0f,  0.5f, 1, PARAM_GRP_SPEED,
      "消除稳态速度误差" },
    { PARAM_PID_SPD_KD,  "spd_kd",  "Speed Kd",    "",   0.0f,   0.0f,   30.0f,  0.1f, 1, PARAM_GRP_SPEED,
      "速度变化阻尼" },
    /* ---------- 转向环 (左右转) ---------- */
    { PARAM_PID_STEER_KP,"steer_kp","Steer Kp",    "",  15.0f,   0.0f,  150.0f,  0.5f, 1, PARAM_GRP_STEER,
      "转向响应快慢" },
    { PARAM_PID_STEER_KI,"steer_ki","Steer Ki",    "",   0.0f,   0.0f,  100.0f,  0.5f, 1, PARAM_GRP_STEER,
      "消除持续偏航" },
    { PARAM_PID_STEER_KD,"steer_kd","Steer Kd",    "",   1.0f,   0.0f,   30.0f,  0.1f, 1, PARAM_GRP_STEER,
      "转向阻尼" },
    /* ---------- 限幅 / 死区 / 滤波 ---------- */
    { PARAM_MOTOR_OUT_MAX, "motor_max", "Motor Out Max", "%", 100.0f,  0.0f,  100.0f, 1.0f, 0, PARAM_GRP_LIMIT,
      "电机总输出上限, 调试时先压低防止打飞" },
    { PARAM_SPEED_OUT_MAX, "spdout_max","Speed Out Max","",  40.0f,  0.0f,  100.0f, 1.0f, 0, PARAM_GRP_LIMIT,
      "速度环输出映射到倾角目标的角度上限 (°)" },
    { PARAM_ANGLE_LIMIT,   "ang_limit",  "Angle Limit", "°",  25.0f,  5.0f,   60.0f, 1.0f, 0, PARAM_GRP_LIMIT,
      "超过这个倾角判定摔倒, 立刻停电机" },
    { PARAM_ANGLE_ZERO,    "ang_zero",   "Angle Zero",  "°",   0.0f, -20.0f,   20.0f, 0.5f, 1, PARAM_GRP_LIMIT,
      "机械零点偏置, 车静止调它让 Pitch 显示 0" },
    { PARAM_GYRO_LPF,      "gyro_lpf",   "Gyro LPF",    "",    0.2f,  0.0f,    1.0f, 0.01f,2, PARAM_GRP_LIMIT,
      "陀螺仪低通系数, 越小越平滑但延迟越大" },
    { PARAM_SPEED_DEADBAND,"deadband",   "Speed Deadband","",  1.0f,  0.0f,   20.0f, 0.5f, 1, PARAM_GRP_LIMIT,
      "编码器速度死区, 抑制原地来回抖" },
    { PARAM_ENCODER_GEAR,  "gear",       "Gear Ratio",  "",   1.0f,  0.1f,   50.0f, 0.1f, 1, PARAM_GRP_LIMIT,
      "电机转速 / 轮子转速, 减速箱填 >1" },
    /* ---------- 硬件调试 ---------- */
    { PARAM_SERVO_CENTER,  "servo_c",   "Servo Center", "°", 80.0f, 30.0f,  100.0f, 1.0f, 0, PARAM_GRP_LIMIT,
      "舵机中心角" },
    { PARAM_SERVO_LIMIT,   "servo_l",   "Servo Limit",  "°",  10.0f,  0.0f,   50.0f, 1.0f, 0, PARAM_GRP_LIMIT,
      "舵机摆动幅度 (围绕中心角左右各多少度)" },
    { PARAM_MOTOR_PWM_MAX, "pwm_max",   "Motor PWM Max","%", 100.0f,  0.0f,  100.0f, 1.0f, 0, PARAM_GRP_LIMIT,
      "电机 PWM 硬件上限" },
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

    /* 检查表结构版本: 表改过就丢弃旧数据, 用新默认值 */
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
 * 不用 cJSON / mbedtls 库: 调参数据格式是我们自己定的 (全是 ASCII 数字 + 引号),
 * 手写扫描 60 行搞定, 省一个组件依赖。
 * 格式示例: {"bal_kp":12.5,"bal_ki":3,"servo_c":75} */
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
