/**
 * @file trim.c
 * @brief 配平表实现: 查表 + 线性插值 + NVS 持久化
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-05
 */

#include "trim.h"

#ifndef TRIM_OFFLINE_TEST
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#endif

#include <string.h>

#ifndef TRIM_OFFLINE_TEST
static const char *TAG = "TRIM";
#endif
/* ========== 存储常量 ========== */
#ifndef TRIM_OFFLINE_TEST
#define NVS_NAMESPACE    "trim"
#define NVS_KEY_DATA     "t"     /* 存 trim_t 的 blob */
#define NVS_KEY_VERSION  "ver"   /* 表结构版本, 变了就丢弃旧数据 */
#endif
#define TRIM_DATA_VERSION 1      /* v1: 一维表 (0.1 的 legmap 是 v2 双路, 完全不同) */

/* 俯仰角"完全没变化"的判定阈值 (°)
 * 低于这个跨度说明数据里没有信息 (车没扶到不同姿态 / MPU 没工作) */
#define TRIM_SPAN_MIN     0.05f

/* ========== 全局数据 ========== */
static trim_t s_trim;

/* ================= 内部查表 ================= */

/**
 * @brief 在升序表里找 servo_deg 的位置
 * @param m     表指针
 * @param deg   目标舵机角
 * @param found [出参] true=已存在同角度的点
 * @return 位置下标 (0..count); found 为 true 时返回已存在那条的下标
 */
static uint8_t find_slot(const trim_t *m, float deg, bool *found)
{
    *found = false;
    for (uint8_t i = 0; i < m->count; i++) {
        if (m->pts[i].servo_deg == deg) {
            *found = true;
            return i;
        }
        if (m->pts[i].servo_deg > deg) {
            return i;       /* 插到这条前面 */
        }
    }
    return m->count;       /* 插到末尾 */
}

/**
 * @brief 按舵机角查表中的俯仰角, 支持精确命中和线性插值
 * @param m    表指针
 * @param deg  舵机角 (°)
 * @return 查到的俯仰角 (°)
 */
static float lookup(const trim_t *m, float deg)
{
    if (m == NULL || m->count == 0) return 0.0f;
    if (m->count == 1)           return m->pts[0].pitch_deg;

    if (deg <= m->pts[0].servo_deg)              return m->pts[0].pitch_deg;
    if (deg >= m->pts[m->count - 1].servo_deg)   return m->pts[m->count - 1].pitch_deg;

    /* 相邻两点之间线性插值
     *
     * ⚠ 条件用 deg >= d0 && deg <= d1 而不是 deg > d0 && deg < d1。
     *   严格不等号会让**恰好落在中间某个点**的查询 (deg == pts[i].servo_deg)
     *   在两个区间都不满足 → 掉到函数末尾 → 返回**最后一个点**的值。
     *   表现: 查 60° 拿到的是 100° 的俯仰角, 而且除了末点以外全都查错。
     *   用闭区间后, deg == d0 时 t=0, deg == d1 时 t=1, 精确命中自然正确。 */
    for (uint8_t i = 0; i + 1 < m->count; i++) {
        float d0 = m->pts[i].servo_deg;
        float d1 = m->pts[i + 1].servo_deg;
        if (deg >= d0 && deg <= d1) {
            float t = (d1 > d0) ? (deg - d0) / (d1 - d0) : 0.0f;
            return m->pts[i].pitch_deg + t * (m->pts[i + 1].pitch_deg - m->pts[i].pitch_deg);
        }
    }
    /* 兜底: 表已保证严格升序, 所以能走到这里说明数据异常。
     * 返回最后一个点的值 (而不是 0), 免得突然冒出个巨大的补偿。 */
    return m->pts[m->count - 1].pitch_deg;
}

/* ================= 对外 API ================= */

const trim_t *trim_get(void)
{
    return &s_trim;
}

float trim_pitch(float servo_deg, float ref_deg)
{
    if (s_trim.count < 2) return 0.0f;   /* 点太少, 退回不补偿 */
    return lookup(&s_trim, servo_deg) - lookup(&s_trim, ref_deg);
}

esp_err_t trim_set_point(float servo_deg, float pitch_deg)
{
    if (pitch_deg < TRIM_PITCH_MIN || pitch_deg > TRIM_PITCH_MAX) {
#ifndef TRIM_OFFLINE_TEST
        ESP_LOGW(TAG, "俯仰角 %.1f° 超出合理范围 %.0f~%.0f",
                 (double)pitch_deg, TRIM_PITCH_MIN, TRIM_PITCH_MAX);
#endif
        return ESP_ERR_INVALID_ARG;
    }

    bool found = false;
    uint8_t slot = find_slot(&s_trim, servo_deg, &found);

    if (found) {                       /* 覆盖已有那条 */
        s_trim.pts[slot].pitch_deg = pitch_deg;
        return ESP_OK;
    }

    if (s_trim.count >= TRIM_MAX_POINTS) {
#ifndef TRIM_OFFLINE_TEST
        ESP_LOGW(TAG, "配平表已满 (%d 个点)", TRIM_MAX_POINTS);
#endif
        return ESP_ERR_NO_MEM;
    }

    /* 后移腾位置, 从后往前挪避免覆盖 */
    for (uint8_t i = s_trim.count; i > slot; i--) {
        s_trim.pts[i] = s_trim.pts[i - 1];
    }
    s_trim.pts[slot].servo_deg = servo_deg;
    s_trim.pts[slot].pitch_deg = pitch_deg;
    s_trim.count++;
    return ESP_OK;
}

esp_err_t trim_del_point(float servo_deg)
{
    for (uint8_t i = 0; i < s_trim.count; i++) {
        if (s_trim.pts[i].servo_deg != servo_deg) continue;
        /* 前移补洞 */
        for (uint8_t j = i; j + 1 < s_trim.count; j++) {
            s_trim.pts[j] = s_trim.pts[j + 1];
        }
        s_trim.count--;
        memset(&s_trim.pts[s_trim.count], 0, sizeof(s_trim.pts[0]));
        return ESP_OK;
    }
    return ESP_OK;   /* 本来就没有, 也算成功 */
}

esp_err_t trim_clear(void)
{
    memset(&s_trim, 0, sizeof(s_trim));
    return ESP_OK;
}

esp_err_t trim_erase_nvs(void)
{
#ifndef TRIM_OFFLINE_TEST
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return ESP_OK;   /* namespace 不存在直接返回 OK */
    }
    esp_err_t ret = nvs_erase_all(h);
    if (ret == ESP_OK) {
        ret = nvs_commit(h);
    }
    nvs_close(h);
    ESP_LOGI(TAG, "NVS trim 数据已擦除");
    return ret;
#else
    /* 离线测试没有 NVS, 直接返回 OK */
    return ESP_OK;
#endif
}

float trim_pitch_span(void)
{
    if (s_trim.count == 0) return 0.0f;

    float lo = s_trim.pts[0].pitch_deg;
    float hi = lo;
    for (uint8_t i = 1; i < s_trim.count; i++) {
        if (s_trim.pts[i].pitch_deg < lo) lo = s_trim.pts[i].pitch_deg;
        if (s_trim.pts[i].pitch_deg > hi) hi = s_trim.pts[i].pitch_deg;
    }
    return hi - lo;
}

bool trim_is_usable(uint8_t min_points)
{
    if (s_trim.count < min_points) return false;

    /* 俯仰角跨度极小 → 数据里没有信息, 查出来的补偿恒等于常数, 没用。
     * 正常的换角度配平至少有零点几度的变化, 所以 0.05° 是很宽松的门槛。 */
    if (trim_pitch_span() < TRIM_SPAN_MIN) {
#ifndef TRIM_OFFLINE_TEST
        ESP_LOGW(TAG, "表内俯仰角跨度只有 %.3f°, 数据没变化 —— "
                      "确认每次都真的把车身扶到了不同姿态", (double)trim_pitch_span());
#endif
        return false;
    }
    return true;
}

/* ================= NVS 持久化 ================= */
#ifndef TRIM_OFFLINE_TEST   /* 离线测试不编译这段, 见 test/test_trim_pc.c */

esp_err_t trim_load(void)
{
    memset(&s_trim, 0, sizeof(s_trim));

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "无已存配平表 (不补偿, 用 ang_zero 配平)");
        return ESP_OK;
    }

    uint16_t ver = 0;
    size_t len = sizeof(ver);
    if (nvs_get_u16(h, NVS_KEY_VERSION, &ver) != ESP_OK || ver != TRIM_DATA_VERSION) {
        ESP_LOGW(TAG, "配平表版本不符 (%u != %d), 用空表", ver, TRIM_DATA_VERSION);
        nvs_close(h);
        return ESP_OK;
    }

    trim_t tmp;
    memset(&tmp, 0, sizeof(tmp));
    len = sizeof(tmp);
    if (nvs_get_blob(h, NVS_KEY_DATA, &tmp, &len) != ESP_OK || len != sizeof(tmp)) {
        ESP_LOGW(TAG, "配平表长度不符, 用空表");
        nvs_close(h);
        return ESP_OK;
    }
    nvs_close(h);

    /* 校验: 点数不能超过容量, 且必须严格升序 (NVS 数据损坏的兜底) */
    if (tmp.count > TRIM_MAX_POINTS) {
        ESP_LOGW(TAG, "配平表点数 %u 越界, 用空表", tmp.count);
        return ESP_OK;
    }
    for (uint8_t i = 0; i + 1 < tmp.count; i++) {
        if (tmp.pts[i].servo_deg >= tmp.pts[i + 1].servo_deg) {
            ESP_LOGW(TAG, "配平表未按角度升序, 数据损坏, 用空表");
            return ESP_OK;
        }
    }

    s_trim = tmp;
    ESP_LOGI(TAG, "已载入配平表: %u 个点, 俯仰角跨度 %.2f°",
             s_trim.count, (double)trim_pitch_span());
    return ESP_OK;
}

esp_err_t trim_save(void)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_blob(h, NVS_KEY_DATA, &s_trim, sizeof(s_trim));
    if (ret == ESP_OK) {
        ret = nvs_set_u16(h, NVS_KEY_VERSION, TRIM_DATA_VERSION);
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(h);
    }
    nvs_close(h);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "配平表 %u 个点已保存", s_trim.count);
    } else {
        ESP_LOGE(TAG, "配平表保存失败: %s", esp_err_to_name(ret));
    }
    return ret;
}

#endif // TRIM_OFFLINE_TEST
