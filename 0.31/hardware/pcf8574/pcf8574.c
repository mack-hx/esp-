/**
 * @file pcf8574.c
 * @brief PCF8574 I2C IO 扩展芯片驱动实现
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#include "pcf8574.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "PCF8574";

/* 按键事件回调 */
static pcf8574_btn_cb_t btn_callback = NULL;

/* ========== 消抖参数 ========== */
/**
 * 时间窗消抖: candidate 状态 (与 last_stable 不同) 持续 DEBOUNCE_MS
 * (推荐 30ms) 后, 才更新 last_stable 并派发边沿事件.
 *
 * 设计依据:
 * - 机械按键抖动期一般 5-20ms (按下瞬间多次通断)
 * - 人手短按 50-100ms, 长按 200ms+
 * - 30ms 持续确认: 抖动期会被过滤 (持续时间不够), 短按仍可被识别
 * - scan 函数推荐 50ms 周期调用 (与 lcd_ui_loop 一致), 抖动样本会被刷掉
 *
 * 实际效果:
 *   单次短按 100ms: scan 在 50ms, 100ms 时读到 0xFE, last_stable 在 50ms 时
 *                   已更新; scan 在 150ms 时读到 0xFF, last_stable 在 200ms 时更新
 *                   → 完整识别"按下-释放"周期
 *   抖动 10ms: scan 期间只采到 1 次 0xFE (50ms 周期), 持续时间 < 30ms → 被过滤
 */
#define DEBOUNCE_MS              30   // 状态变化需持续 N ms 才确认
#define PULLUP_STABLE_MS         2    // PCF8574 内部上拉稳定时间 (替代原 10ms)

/**
 * @brief 内部消抖状态机 (时间窗消抖)
 *
 * 设计: candidate 状态 (与 last_stable 不同) 持续 DEBOUNCE_MS 才确认.
 *   - 抖动期短 (<DEBOUNCE_MS) 被自动过滤
 *   - 人手短按 (>= DEBOUNCE_MS) 完整识别
 *   - 按住期间不再触发 (last_stable 不变, 不产生边沿)
 *
 * 字段说明:
 *   - state: 仅用于日志 (不再用于控制流)
 *   - last_stable: 最近一次稳定的按键字节 (bit=1 表示按下, 全松开=0xFF)
 *   - candidate: 当前 raw 偏离 last_stable 后的目标值 (若无偏离 == last_stable)
 *   - candidate_start_tick: candidate 起始时刻 (xTaskGetTickCount 单位)
 */
typedef enum {
    KEY_STATE_IDLE = 0,
    KEY_STATE_PRESSED,
} key_fsm_state_t;

static struct {
    key_fsm_state_t state;
    uint8_t         last_stable;           // 最近一次稳定的按键字节 (bit=1 表示按下)
    uint8_t         candidate;             // 当前候选状态 (消抖中, 与 last_stable 不同)
    TickType_t      candidate_start_tick;  // candidate 起始时刻
} s_key_fsm = {
    .state                 = KEY_STATE_IDLE,
    .last_stable           = 0xFF,          // 全松开
    .candidate             = 0xFF,
    .candidate_start_tick  = 0,
};

/**
 * @brief 写入 1 字节 (用于 PCF8574 输出)
 */
static esp_err_t pcf8574_write_byte(uint8_t data)
{
    return i2c_master_write_to_device(PCF8574_I2C_NUM, PCF8574_I2C_ADDR,
                                      &data, 1, pdMS_TO_TICKS(1000));
}

/**
 * @brief 读取 1 字节 (含预写 0xFF 刷新 + 全低重试 + 兜底)
 * @note  PCF8574 是准双向 IO, 内部 100µA 上拉很弱.
 *       当某个引脚被按键拉低时, 公共 VCC 跌落会导致其他引脚被误读为低.
 *       (即使有外部 4.7kΩ 上拉, 多键同时按下仍可能跌落)
 *
 *       解决办法:
 *         1. 读前先写 0xFF 强制所有引脚为高电平 (刷新内部锁存器)
 *         2. 等待 2ms 让外部 4.7kΩ 上拉稳定 (替代原 10ms 内部上拉等待)
 *         3. 读取真实电平
 *         4. 如果读到 0x00 (全低, VCC 跌落), 重试一次 (再次写 0xFF + 等 2ms + 读)
 *         5. 仍读到 0x00 → 视为全松开 (返回 0xFF, 保守策略)
 *
 *       耗时: 正常 ~3-4ms (2ms 等待 + 2 次 I2C 传输)
 *             重试 ~6-8ms
 */
static esp_err_t pcf8574_read_byte(uint8_t *data)
{
    if (data == NULL) return ESP_ERR_INVALID_ARG;

    for (int attempt = 0; attempt < 2; attempt++) {
        // 1. 强制所有引脚为高电平 (刷新内部锁存器)
        uint8_t all_high = 0xFF;
        esp_err_t ret = i2c_master_write_to_device(PCF8574_I2C_NUM, PCF8574_I2C_ADDR,
                                                  &all_high, 1, pdMS_TO_TICKS(1000));
        if (ret != ESP_OK) return ret;

        // 2. 等待外部 4.7kΩ 上拉稳定 (2ms 足够, 内部 100µA 上拉配合 4.7k 外部上拉
        //    RC 时间常数约 τ ≈ 1kΩ × 20pF ≈ 20ns, 1ms 已是 50000τ, 完全稳定)
        vTaskDelay(pdMS_TO_TICKS(PULLUP_STABLE_MS));

        // 3. 读取真实电平
        ret = i2c_master_read_from_device(PCF8574_I2C_NUM, PCF8574_I2C_ADDR,
                                          data, 1, pdMS_TO_TICKS(1000));
        if (ret != ESP_OK) return ret;

        // 4. 如果不是全低, 正常返回
        if (*data != 0x00) {
            return ESP_OK;
        }
        // 否则 retry (第 2 次仍 0x00 才走兜底)
    }

    // 5. 兜底 - 视为全松开 (保守策略)
    *data = 0xFF;
    return ESP_OK;
}

/**
 * @brief 初始化 I2C 总线 (PCF8574 专用)
 */
static esp_err_t pcf8574_i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PCF8574_SDA_GPIO,
        .scl_io_num = PCF8574_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = PCF8574_I2C_FREQ,
    };

    esp_err_t ret = i2c_param_config(PCF8574_I2C_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 参数配置失败");
        return ret;
    }

    ret = i2c_driver_install(PCF8574_I2C_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 驱动安装失败");
        return ret;
    }

    ESP_LOGI(TAG, "I2C 初始化成功 (SCL=%d, SDA=%d, %dkHz)",
             PCF8574_SCL_GPIO, PCF8574_SDA_GPIO, PCF8574_I2C_FREQ / 1000);
    return ESP_OK;
}

/**
 * @brief 初始化 PCF8574
 */
esp_err_t pcf8574_init(void)
{
    esp_err_t ret = pcf8574_i2c_init();
    if (ret != ESP_OK) return ret;

    // 探测设备
    uint8_t dummy;
    ret = pcf8574_read_byte(&dummy);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCF8574 设备未响应 (地址 0x%02X)", PCF8574_I2C_ADDR);
        return ret;
    }

    // 写 0xFF (高电平), 让所有引脚作为输入时处于上拉状态
    // 注: PCF8574 是准双向 IO, 输出 1 时为输入
    ret = pcf8574_write_byte(0xFF);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCF8574 初始化写入失败");
        return ret;
    }

    // 重置内部消抖状态机
    s_key_fsm.state                = KEY_STATE_IDLE;
    s_key_fsm.last_stable          = 0xFF;
    s_key_fsm.candidate            = 0xFF;
    s_key_fsm.candidate_start_tick = 0;

    ESP_LOGI(TAG, "PCF8574 初始化成功 (INT=%d, 地址=0x%02X, 消抖=%dms)",
             PCF8574_INT_GPIO, PCF8574_I2C_ADDR, DEBOUNCE_MS);
    return ESP_OK;
}

/**
 * @brief 读取所有引脚状态 (原始读取, 不含消抖)
 */
esp_err_t pcf8574_read_all(uint8_t *state)
{
    if (state == NULL) return ESP_ERR_INVALID_ARG;
    return pcf8574_read_byte(state);
}

/**
 * @brief 读取单个引脚状态
 */
esp_err_t pcf8574_read_pin(pcf8574_pin_t pin, bool *state)
{
    if (pin > PCF8574_P7 || state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data;
    esp_err_t ret = pcf8574_read_byte(&data);
    if (ret != ESP_OK) return ret;

    *state = (data >> pin) & 0x01;
    return ESP_OK;
}

/**
 * @brief 检测按键是否按下
 */
esp_err_t pcf8574_is_pressed(btn_id_t id, bool *pressed)
{
    if (id > SW_K6 || pressed == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return pcf8574_read_pin((pcf8574_pin_t)id, pressed);
}

/**
 * @brief 扫描按键状态 (含时间窗消抖 + 按下/释放边沿事件)
 *
 * 算法 (时间窗消抖, 推荐 50ms 周期调用):
 *   1. 读 raw 当前按键字节
 *   2. 若 raw == last_stable: candidate = last_stable, 重置计时
 *      若 raw != last_stable 且 raw != candidate: candidate = raw, 重置计时
 *      否则 (raw 持续 == candidate): 检查累计持续时长
 *        - 达到 DEBOUNCE_MS → 更新 last_stable, 派发边沿
 *
 * @param pressed_mask 输出: 当前稳定按下的按键位图 (bit=1 表示按下)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t pcf8574_scan_buttons(uint8_t *pressed_mask)
{
    if (pressed_mask == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t raw;
    esp_err_t ret = pcf8574_read_byte(&raw);
    if (ret != ESP_OK) {
        *pressed_mask = s_key_fsm.last_stable;  // 失败时返回上次稳定值
        return ret;
    }

    // 时间窗消抖: candidate 状态持续 DEBOUNCE_MS 后确认
    TickType_t now = xTaskGetTickCount();

    if (raw == s_key_fsm.last_stable) {
        // 与稳定状态一致: 重置 candidate
        s_key_fsm.candidate            = s_key_fsm.last_stable;
        s_key_fsm.candidate_start_tick = now;
    } else if (raw != s_key_fsm.candidate) {
        // candidate 切换 (例如抖动跳到另一个值): 重新开始计时
        s_key_fsm.candidate            = raw;
        s_key_fsm.candidate_start_tick = now;
    } else {
        // candidate 持续中: 检查是否达到 DEBOUNCE_MS
        TickType_t elapsed = (now >= s_key_fsm.candidate_start_tick)
                           ? (now - s_key_fsm.candidate_start_tick)
                           : 0;
        if (pdTICKS_TO_MS(elapsed) >= DEBOUNCE_MS) {
            // 确认状态变化, 派发边沿
            uint8_t prev_stable = s_key_fsm.last_stable;
            s_key_fsm.last_stable = raw;

            bool prev_any_pressed = (prev_stable != 0xFF);
            bool curr_any_pressed = (raw != 0xFF);

            if (!prev_any_pressed && curr_any_pressed) {
                s_key_fsm.state = KEY_STATE_PRESSED;
                ESP_LOGI(TAG, "按下: 0x%02X (位图)", raw);
                if (btn_callback) {
                    for (uint8_t i = 0; i < 8; i++) {
                        if ((raw & (1 << i)) && !(prev_stable & (1 << i))) {
                            btn_callback((btn_id_t)i, BTN_EVENT_PRESS);
                        }
                    }
                }
            } else if (prev_any_pressed && !curr_any_pressed) {
                s_key_fsm.state = KEY_STATE_IDLE;
                ESP_LOGI(TAG, "释放: 0x%02X (位图)", raw);
                if (btn_callback) {
                    for (uint8_t i = 0; i < 8; i++) {
                        if (!(raw & (1 << i)) && (prev_stable & (1 << i))) {
                            btn_callback((btn_id_t)i, BTN_EVENT_RELEASE);
                        }
                    }
                }
            } else if (prev_any_pressed && curr_any_pressed) {
                s_key_fsm.state = KEY_STATE_PRESSED;  // 持续按住, 不重复触发
            } else {
                s_key_fsm.state = KEY_STATE_IDLE;
            }
        }
    }

    *pressed_mask = s_key_fsm.last_stable;
    return ESP_OK;
}

/**
 * @brief 注册按键事件回调
 */
esp_err_t pcf8574_register_callback(pcf8574_btn_cb_t cb)
{
    if (cb == NULL) return ESP_ERR_INVALID_ARG;
    btn_callback = cb;
    return ESP_OK;
}

/**
 * @brief 中断处理函数 (由 ISR 调用)
 */
static void IRAM_ATTR pcf8574_isr_handler(void *arg)
{
    // 中断只做标志位标记, 实际处理放到任务中
    (void)arg;
}

/**
 * @brief 启用中断 (GPIO 下降沿触发)
 * @note  PCF8574 INT 为开漏输出, 低电平有效. 任何按键按下/释放都会触发 INT.
 *        本函数配置 ESP32 下降沿中断, 但 ISR 内只做占位 (实际扫描仍走主循环).
 *        启用后, 主循环可读取 INT 状态作为"有按键变化"提示以减少扫描频率.
 */
esp_err_t pcf8574_enable_interrupt(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PCF8574_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,   // 下降沿触发
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) return ret;

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    ret = gpio_isr_handler_add(PCF8574_INT_GPIO, pcf8574_isr_handler, NULL);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "PCF8574 中断已启用 (GPIO=%d)", PCF8574_INT_GPIO);
    return ESP_OK;
}

/**
 * @brief 读取两个对接微动开关的状态
 */
esp_err_t pcf8574_read_dock_switches(bool *sw1, bool *sw2)
{
    if (sw1 == NULL || sw2 == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t state = 0xFF;
    if (pcf8574_read_all(&state) != ESP_OK) {
        *sw1 = false;
        *sw2 = false;
        return ESP_FAIL;
    }
    /* P5/P4 内部上拉, 压下拉低 —— 取反才是"压下" */
    *sw1 = !(state & (1 << SW_K5));
    *sw2 = !(state & (1 << SW_K6));
    return ESP_OK;
}
