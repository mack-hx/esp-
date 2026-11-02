/**
 * @file ble_ctrl.c
 * @brief BLE 直控实现 (NimBLE 外设), 协议见 ble_ctrl.h
 */
#include "ble_ctrl.h"
#include "onenet_client.h"   /* remote_move / remote_stop / remote_stand */
#include "balance.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLE";

static uint8_t  s_own_addr_type;
static volatile uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;  /* 当前连接句柄 */
static volatile bool     s_notify_on = false;               /* App 是否订阅了通知 */
static uint16_t s_chr_handle;                               /* 0xFFE1 特征值句柄 */
static volatile int64_t  s_move_ms = 0;    /* 最近一次非零移动指令的时刻 (ms), 0 = 无 */

static void ble_advertise(void);

/**
 * @brief 解析并执行一条 App 指令
 * @param s   指令文本 (以 '\0' 结尾)
 */
static void ble_handle_cmd(const char *s)
{
    if (strncmp(s, "M:", 2) == 0) {
        float v = strtof(s + 2, NULL);
        remote_move(v);
        s_move_ms = (v != 0.0f) ? esp_timer_get_time() / 1000 : 0;
    } else if (strcmp(s, "STOP") == 0) {
        s_move_ms = 0;
        remote_stop();
    } else if (strcmp(s, "STAND") == 0) {
        remote_stand();
    } else {
        ESP_LOGW(TAG, "未知指令: %s", s);
    }
}

/** @brief 0xFFE1 访问回调: 只处理写 */
static int chr_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;
    char buf[32];
    uint16_t len = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf) - 1, &len) != 0) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    buf[len] = '\0';
    ble_handle_cmd(buf);
    return 0;
}

static const struct ble_gatt_svc_def s_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0xFFE0),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid       = BLE_UUID16_DECLARE(0xFFE1),
                .access_cb  = chr_access,
                .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_chr_handle,
            },
            { 0 }
        },
    },
    { 0 }
};

/** @brief GAP 事件: 连接 / 断开 / 订阅 */
static int gap_event(struct ble_gap_event *ev, void *arg)
{
    switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (ev->connect.status == 0) {
            s_conn = ev->connect.conn_handle;
            remote_set_ble_link(true);
            ESP_LOGI(TAG, "已连接");
        } else {
            ble_advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "断开 (reason=%d)", ev->disconnect.reason);
        s_conn = BLE_HS_CONN_HANDLE_NONE;
        s_notify_on = false;
        remote_set_ble_link(false);
        if (s_move_ms) { s_move_ms = 0; remote_move(0.0f); }
        ble_advertise();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (ev->subscribe.attr_handle == s_chr_handle) {
            s_notify_on = ev->subscribe.cur_notify;
        }
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_advertise();
        break;
    default:
        break;
    }
    return 0;
}

/** @brief 开始广播 (广播包带名字 + 服务 UUID 0xFFE0) */
static void ble_advertise(void)
{
    struct ble_hs_adv_fields f = {0};
    f.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    f.name = (uint8_t *)BLE_DEVICE_NAME;
    f.name_len = strlen(BLE_DEVICE_NAME);
    f.name_is_complete = 1;
    f.uuids16 = (ble_uuid16_t[]) { BLE_UUID16_INIT(0xFFE0) };
    f.num_uuids16 = 1;
    f.uuids16_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&f);
    if (rc != 0) { ESP_LOGE(TAG, "广播数据设置失败 %d", rc); return; }

    struct ble_gap_adv_params p = {0};
    p.conn_mode = BLE_GAP_CONN_MODE_UND;
    p.disc_mode = BLE_GAP_DISC_MODE_GEN;
    p.itvl_min = 160;  /* 100ms (单位 0.625ms) */
    p.itvl_max = 320;  /* 200ms */
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &p, gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "广播启动失败 %d, 将自动重试", rc);
    }
}

static void on_sync(void)
{
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    ble_advertise();
    ESP_LOGI(TAG, "广播中: %s", BLE_DEVICE_NAME);
}

static void host_task(void *arg)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/** @brief pitch 通知 + 移动指令超时检查, 周期 BLE_NOTIFY_MS */
static void notify_task(void *arg)
{
    char buf[16];
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(BLE_NOTIFY_MS));
        if (s_move_ms && esp_timer_get_time() / 1000 - s_move_ms > BLE_MOVE_TIMEOUT_MS) {
            s_move_ms = 0;
            remote_move(0.0f);
            ESP_LOGW(TAG, "移动指令超时, 已归零");
        }
        uint16_t conn = s_conn;
        if (conn == BLE_HS_CONN_HANDLE_NONE || !s_notify_on) continue;
        bal_data_t d;
        if (balance_get_data(&d) != ESP_OK) continue;
        int n = snprintf(buf, sizeof(buf), "P:%.1f", (double)d.pitch);
        struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, n);
        if (om) ble_gatts_notify_custom(conn, s_chr_handle, om);
    }
}

esp_err_t ble_ctrl_start(void)
{
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init 失败: %s", esp_err_to_name(ret));
        return ret;
    }
    ble_hs_cfg.sync_cb = on_sync;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    int rc = ble_gatts_count_cfg(s_svcs);
    if (rc == 0) rc = ble_gatts_add_svcs(s_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT 注册失败 %d", rc);
        return ESP_FAIL;
    }
    ble_svc_gap_device_name_set(BLE_DEVICE_NAME);

    nimble_port_freertos_init(host_task);
    /* 通知任务绑 core 0, 优先级 3 */
    xTaskCreatePinnedToCore(notify_task, "ble_notify", 3072, NULL, 3, NULL, 0);
    return ESP_OK;
}

bool ble_ctrl_is_connected(void)
{
    return s_conn != BLE_HS_CONN_HANDLE_NONE;
}
