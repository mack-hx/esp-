/**
 * @file legmap.c
 * @brief 腿高/直距查表实现 (左/右两路独立, 查表 + 线性插值 + NVS 持久化)
 * @author ESP32 机械狗项目
 * @date 2026-10-05
 */

#include "legmap.h"

#ifndef LEGMAP_OFFLINE_TEST   /* 离线测试时不引入 ESP-IDF 头 */
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#endif

#include <string.h>
#include <ctype.h>

static const char *TAG = "LEGMAP";

/* ========== 存储常量 ========== */
#ifndef LEGMAP_OFFLINE_TEST
#define NVS_NAMESPACE   "legmap"
#define NVS_KEY_DATA    "m"          /* 存 legmap_all_t 的 blob */
#define NVS_KEY_VERSION "ver"        /* 表结构版本, 变了就丢弃旧数据 */
#endif
#define LEGMAP_DATA_VERSION  2       /* v2: 改为双路表 (左/右各一份) */

/* ========== 全局数据 ========== */
static legmap_all_t s_all;

/* 直距单调性检查的容差 (mm): 允许量尺/机构回差造成的小幅非单调 */
#define LEGMAP_MONO_TOL   1.0f

/* ========== 内部辅助 ========== */

static legmap_t *map_of(leg_id_t leg)
{
    if ((unsigned)leg >= (unsigned)LEG_COUNT) return NULL;
    return &s_all.legs[leg];
}

char legmap_leg_char(leg_id_t leg)
{
    switch (leg) {
        case LEG_LEFT:  return 'L';
        case LEG_RIGHT: return 'R';
        default:        return '?';
    }
}

esp_err_t legmap_parse_leg(const char *s, leg_id_t *out)
{
    if (s == NULL || out == NULL) return ESP_ERR_INVALID_ARG;
    /* 跳过空白 */
    while (*s == ' ' || *s == '\t') s++;
    if (*s == '\0') return ESP_ERR_INVALID_ARG;

    /* 取首字符做大小写不敏感比较 */
    char c = (char)tolower((unsigned char)*s);
    if (c == 'l') { *out = LEG_LEFT;  return ESP_OK; }
    if (c == 'r') { *out = LEG_RIGHT; return ESP_OK; }

    /* 全字符串比较, 兼容 "LEFT"/"RIGHT" */
    char buf[8];
    size_t i = 0;
    while (s[i] && i < sizeof(buf)-1) {
        buf[i] = (char)tolower((unsigned char)s[i]);
        i++;
    }
    buf[i] = '\0';
    if (strcmp(buf, "left") == 0)  { *out = LEG_LEFT;  return ESP_OK; }
    if (strcmp(buf, "right") == 0) { *out = LEG_RIGHT; return ESP_OK; }
    return ESP_ERR_INVALID_ARG;
}

/**
 * @brief 在升序表里找 servo_deg 的插入位置
 * @param m     表指针
 * @param deg   目标舵机角
 * @param found [出参] true=已存在同角度的点
 * @return 插入位置下标 (0..count); found 为 true 时返回已存在那条的下标
 */
static uint8_t map_find_slot(const legmap_t *m, float deg, bool *found)
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
 * @brief 在升序表里按角度查一个点, 返回下标; 找不到返回 -1
 */
static int map_find_exact(const legmap_t *m, float deg)
{
    for (uint8_t i = 0; i < m->count; i++) {
        if (m->pts[i].servo_deg == deg) return (int)i;
    }
    return -1;
}

/* ================= 查表与插值 ================= */

/**
 * @brief 按舵机角查表中的某个字段, 支持精确命中和线性插值
 * @param m         表指针
 * @param deg       舵机角 (°)
 * @param get_field 取值回调: 传入点, 返回该点对应字段的值
 * @param ok        [出参] false=查询失败 (表空/越界)
 * @return 查到的值
 * @note  越界时夹到端点, 不做外推 (理由同 legmap_height)
 */
static float map_lookup(const legmap_t *m, float deg,
                        float (*get_field)(const legmap_point_t *),
                        bool *ok)
{
    *ok = false;
    if (m == NULL || m->count == 0) return 0.0f;

    /* 只有 1 个点: 直接给它的值 */
    if (m->count == 1) {
        *ok = true;
        return get_field(&m->pts[0]);
    }

    /* 落在范围外: 夹到端点 */
    if (deg <= m->pts[0].servo_deg) {
        *ok = true;
        return get_field(&m->pts[0]);
    }
    if (deg >= m->pts[m->count - 1].servo_deg) {
        *ok = true;
        return get_field(&m->pts[m->count - 1]);
    }

    /* 精确命中 */
    int exact = map_find_exact(m, deg);
    if (exact >= 0) {
        *ok = true;
        return get_field(&m->pts[exact]);
    }

    /* 相邻两点之间线性插值 */
    for (uint8_t i = 0; i + 1 < m->count; i++) {
        float d0 = m->pts[i].servo_deg;
        float d1 = m->pts[i + 1].servo_deg;
        if (deg > d0 && deg < d1) {
            float t = (deg - d0) / (d1 - d0);
            float v0 = get_field(&m->pts[i]);
            float v1 = get_field(&m->pts[i + 1]);
            *ok = true;
            return v0 + t * (v1 - v0);
        }
    }
    return 0.0f;   /* 理论上到不了这里 */
}

static float get_field_height(const legmap_point_t *p) { return p->height_mm; }
static float get_field_pitch(const legmap_point_t *p) { return p->pitch_deg; }

float legmap_height(leg_id_t leg, float servo_deg)
{
    legmap_t *m = map_of(leg);
    if (m == NULL) return LEGMAP_HEIGHT_UNKNOWN;
    bool ok = false;
    float h = map_lookup(m, servo_deg, get_field_height, &ok);
    return ok ? h : LEGMAP_HEIGHT_UNKNOWN;
}

float legmap_pitch_comp(leg_id_t leg, float servo_deg, float ref_deg)
{
    legmap_t *m = map_of(leg);
    if (m == NULL || m->count < 2) return 0.0f;   /* 点太少, 退回不补偿 */

    /* 两个角度对应的 Pitch 求差
     * ⚠ 高度单调, 所以"按角度查 Pitch"和"按高度查 Pitch"等价,
     *   直接按角度插值即可, 不必先算腿高再反查 —— 少一层误差。 */
    bool ok_now = false, ok_ref = false;
    float p_now = map_lookup(m, servo_deg, get_field_pitch, &ok_now);
    float p_ref = map_lookup(m, ref_deg,   get_field_pitch, &ok_ref);

    if (!ok_now || !ok_ref) return 0.0f;
    return p_now - p_ref;
}

bool legmap_is_usable(leg_id_t leg, uint8_t min_points)
{
    const legmap_t *m = (const legmap_t *)map_of(leg);
    if (m == NULL) return false;
    if (m->count < min_points) return false;

    /* 直距对角度必须是**单峰**的: 先降后升 (或反过来), 最多一个拐点。
     *
     * ⚠ 四连杆的真实直距曲线就是 U 型 —— 腿伸直(大角度)时电机↔舵机轴心
     *   距离远, 腿弯曲(小角度)到某个中间位置时距离最近, 再往下又开始张开。
     *   所以**不能要求单调递增**, 那样会误杀所有合法的标定数据。
     *
     * 要检查的是"有没有来回抖" —— 真实的 U 型曲线上,
     * 局部高度变化在拐点处会变号, 但**不会变号两次以上**。
     * 变号两次以上 = 数据录乱了 (尺子读数错、腿没到位又采了一次)。
     *
     * LEGMAP_MONO_TOL 吸收量尺误差和机构回差, 免得 ±0.5mm 的抖动被当拐点。 */
    uint8_t turns = 0;        /* 已确认的拐点数 */
    int8_t   last_sign = 0;   /* 上一次高度变化的方向: -1降 +1升 0未知 */

    for (uint8_t i = 0; i + 1 < m->count; i++) {
        float dh = m->pts[i + 1].height_mm - m->pts[i].height_mm;

        if (dh >  LEGMAP_MONO_TOL)      { if (last_sign == -1) turns++; last_sign =  1; }
        else if (dh < -LEGMAP_MONO_TOL) { if (last_sign ==  1) turns++; last_sign = -1; }
        /* |dh| <= 容差: 视为持平, 不更新 last_sign, 免得噪声被计入拐点 */

        if (turns > 1) {
            ESP_LOGW(TAG, "[%c] 直距曲线出现 %d 个拐点(应≤1), 数据可能录错: "
                          "%d°=%.1fmm → %d°=%.1fmm",
                     legmap_leg_char(leg), turns,
                     (int)m->pts[i].servo_deg,     (double)m->pts[i].height_mm,
                     (int)m->pts[i + 1].servo_deg, (double)m->pts[i + 1].height_mm);
            return false;
        }
    }
    return true;
}

/* ================= 增删改 ================= */

esp_err_t legmap_upsert_point(leg_id_t leg, float servo_deg, float height_mm, float pitch_deg)
{
    legmap_t *m = map_of(leg);
    if (m == NULL) return ESP_ERR_INVALID_ARG;

    if (height_mm < LEGMAP_HEIGHT_MIN || height_mm > LEGMAP_HEIGHT_MAX) {
        ESP_LOGW(TAG, "[%c] 直距 %.1fmm 超出合理范围 %.0f~%.0f",
                 legmap_leg_char(leg), (double)height_mm,
                 LEGMAP_HEIGHT_MIN, LEGMAP_HEIGHT_MAX);
        return ESP_ERR_INVALID_ARG;
    }

    bool found = false;
    uint8_t slot = map_find_slot(m, servo_deg, &found);

    if (found) {
        /* 覆盖已有那条 */
        m->pts[slot].height_mm = height_mm;
        m->pts[slot].pitch_deg = pitch_deg;
        m->pts[slot].valid      = true;
        return ESP_OK;
    }

    if (m->count >= LEGMAP_MAX_POINTS) {
        ESP_LOGW(TAG, "[%c] 标定点已满 (%d 个)", legmap_leg_char(leg), LEGMAP_MAX_POINTS);
        return ESP_ERR_NO_MEM;
    }

    /* 后移腾位置, 从后往前挪避免覆盖 */
    for (uint8_t i = m->count; i > slot; i--) {
        m->pts[i] = m->pts[i - 1];
    }
    m->pts[slot].servo_deg = servo_deg;
    m->pts[slot].height_mm = height_mm;
    m->pts[slot].pitch_deg = pitch_deg;
    m->pts[slot].valid      = true;
    m->count++;
    return ESP_OK;
}

esp_err_t legmap_clear(leg_id_t leg)
{
    legmap_t *m = map_of(leg);
    if (m == NULL) return ESP_ERR_INVALID_ARG;
    memset(m, 0, sizeof(*m));
    return ESP_OK;
}

esp_err_t legmap_clear_all(void)
{
    memset(&s_all, 0, sizeof(s_all));
    return ESP_OK;
}

const legmap_all_t *legmap_get(void)
{
    return &s_all;
}

const legmap_t *legmap_get_one(leg_id_t leg)
{
    return (const legmap_t *)map_of(leg);
}

/* ================= NVS 持久化 ================= */
#ifndef LEGMAP_OFFLINE_TEST   /* 离线测试桩, 见 test/test_legmap_pc.c */

esp_err_t legmap_load(void)
{
    memset(&s_all, 0, sizeof(s_all));

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "无已存标定数据 (用默认值, 即不补偿)");
        return ESP_OK;
    }

    uint16_t ver = 0;
    size_t len = sizeof(ver);
    if (nvs_get_u16(h, NVS_KEY_VERSION, &ver) != ESP_OK || ver != LEGMAP_DATA_VERSION) {
        ESP_LOGW(TAG, "标定数据版本不符 (%u != %d), 用默认值", ver, LEGMAP_DATA_VERSION);
        nvs_close(h);
        return ESP_OK;
    }

    legmap_all_t tmp;
    memset(&tmp, 0, sizeof(tmp));
    len = sizeof(tmp);
    if (nvs_get_blob(h, NVS_KEY_DATA, &tmp, &len) != ESP_OK || len != sizeof(tmp)) {
        ESP_LOGW(TAG, "标定数据长度不符, 用默认值");
        nvs_close(h);
        return ESP_OK;
    }
    nvs_close(h);

    /* 逐点校验 valid 位, 防止损坏数据里混进未初始化的记录 */
    for (uint8_t k = 0; k < LEG_COUNT; k++) {
        uint8_t n = 0;
        for (uint8_t i = 0; i < LEGMAP_MAX_POINTS; i++) {
            if (tmp.legs[k].pts[i].valid) n++;
        }
        if (n != tmp.legs[k].count) {
            ESP_LOGW(TAG, "[%c] 标定数据内部不一致 (count=%u, valid=%u), 用默认值",
                     legmap_leg_char((leg_id_t)k), tmp.legs[k].count, n);
            memset(&s_all, 0, sizeof(s_all));
            return ESP_OK;
        }
    }

    s_all = tmp;
    ESP_LOGI(TAG, "已载入 L:%u 点, R:%u 点",
             s_all.legs[LEG_LEFT].count, s_all.legs[LEG_RIGHT].count);
    return ESP_OK;
}

esp_err_t legmap_save(void)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_blob(h, NVS_KEY_DATA, &s_all, sizeof(s_all));
    if (ret == ESP_OK) {
        ret = nvs_set_u16(h, NVS_KEY_VERSION, LEGMAP_DATA_VERSION);
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(h);
    }
    nvs_close(h);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "L:%u R:%u 个标定点已保存到 NVS",
                 s_all.legs[LEG_LEFT].count, s_all.legs[LEG_RIGHT].count);
    } else {
        ESP_LOGE(TAG, "标定数据保存失败: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t legmap_init(void)
{
    return legmap_load();
}

#endif /* LEGMAP_OFFLINE_TEST */