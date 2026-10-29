/**
 * @file ui_encoder.c
 * @brief UI 编码电机调试页实现
 * @author ESP32 机械狗项目
 * @date 2026-10-04
 */

#include "ui_encoder.h"
#include "encoder.h"
#include "motor_driver.h"
#include "tft_display.h"
#include <stdio.h>

/* ========== 运行模式 ========== */
typedef enum {
    MODE_M1 = 0,    /* 只转电机 A */
    MODE_M2,        /* 只转电机 B */
    MODE_BOTH,      /* 两电机同转 */
    MODE_DIR,       /* 切换方向 (FORWARD / REVERSE) */
    MODE_SPEED,     /* 调速档位 */
    MODE_MAX
} motor_mode_t;

/* 模式显示名 */
static const char *MODE_NAMES[MODE_MAX] = {
    "M1",
    "M2",
    "A+B",
    "DIR",
    "SPD"
};

/* 速度档位 (SPEED 模式设定, 同时也是 M1/M2/BOTH 实际转动速度)
 * 直接引用 motor_spd_levels[] —— 全工程唯一的档位表,
 * App 的"速度档位"也用这张表, 保证屏上和 App 上的档位含义完全一致。 */
#define SPEED_LEVELS      motor_spd_levels
#define SPEED_LEVEL_COUNT MOTOR_SPD_LEVEL_COUNT

/* 当前模式 */
static motor_mode_t s_mode       = MODE_M1;
/* 当前速度档位 (0~SPEED_LEVEL_COUNT-1, 对应 20/40/60/80) */
static uint8_t      s_speed_idx  = 0;
/* 当前方向 */
static motor_dir_t  s_dir        = MOTOR_DIR_FORWARD;
/* 启动标志: 仅 M1/M2/BOTH 模式 + s_running=true 时实际转动 */
static bool         s_running    = false;

/* 上一次显示的 RPM */
static float s_last_rpm[2] = { 0.0f, 0.0f };

/* ========== 辅助函数 ========== */

/**
 * @brief 把 M1/M2/BOTH + s_running + s_dir + s_speed_idx 应用到硬件
 *        DIR/SPD 模式不会驱动电机, 调用前需确保已刹车
 */
static void apply_motor_state(void)
{
    int8_t sa = 0, sb = 0;
    int8_t sign = (s_dir == MOTOR_DIR_REVERSE) ? -1 : 1;
    int8_t spd = SPEED_LEVELS[s_speed_idx];

    if (!s_running) {
        motor_brake_all();
        return;
    }

    switch (s_mode) {
        case MODE_M1:   sa = spd * sign; sb = 0;          break;
        case MODE_M2:   sa = 0;          sb = spd * sign; break;
        case MODE_BOTH: sa = spd * sign; sb = spd * sign; break;
        default: break;
    }
    motor_set_speed_both(sa, sb);
}

/**
 * @brief 强制进入 DIR/SPD 设定模式: 停车并清运行标志
 *        用于 K3 从 M1/M2/BOTH 切到 DIR/SPD 时, 保证状态行与实际硬件同步
 */
static void enter_setting_mode(void)
{
    if (s_running) {
        s_running = false;
        motor_brake_all();
    }
}

/* ========== 绘制坐标常量 (128x160, font6x8) ========== */
#define Y_TITLE        1       /* 标题栏 */
#define Y_ENC1         14      /* E1 */
#define Y_RPM1         26      /* R1 */
#define Y_ENC2         38      /* E2 */
#define Y_RPM2         50      /* R2 */
#define Y_SEP1         62      /* 分隔线 1 */
#define Y_MODE         72      /* 模式行 */
#define Y_STAT         90      /* 状态行 */
#define Y_SEP2         108     /* 分隔线 2 */
#define Y_HINT         120     /* 单行按键提示 */

/* 模式行: 5 个模式横向排列 */
#define MODE_X_BASE    6
#define MODE_X_STEP    24

/**
 * @brief 绘制静态骨架 (标题 + 4 行数据区标签 + 模式/状态行 + 分隔线 + 单行按键提示)
 */
static void ui_encoder_draw_static(void)
{
    tft_clear(TFT_COLOR_BLACK);

    /* 顶部标题 */
    ui_fill_rect(0, 0, UI_SCREEN_W, UI_FONT_H + 2, TFT_COLOR_WHITE);
    ui_draw_string(2, Y_TITLE, "ENCODER+MOTOR", TFT_COLOR_BLACK, TFT_COLOR_WHITE);

    /* 数据区标签 (E1/R1/E2/R2) */
    ui_draw_string(2, Y_ENC1, "E1:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_RPM1, "R1:", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_ENC2, "E2:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_RPM2, "R2:", TFT_COLOR_WHITE, TFT_COLOR_BLACK);

    /* 分隔线 */
    ui_draw_hline(0, Y_SEP1, UI_SCREEN_W, TFT_COLOR_BLUE);

    /* 模式行标签 + 状态行标签 */
    ui_draw_string(2, Y_MODE,  "Mode:", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_STAT,  "Stat:", TFT_COLOR_WHITE, TFT_COLOR_BLACK);

    /* 分隔线 + 单行按键提示 */
    ui_draw_hline(0, Y_SEP2, UI_SCREEN_W, TFT_COLOR_BLUE);
    ui_draw_string(2, Y_HINT, "[K1]Back [K2]Set/Run [K3]Mode",
                   TFT_COLOR_WHITE, TFT_COLOR_BLACK);
}

/**
 * @brief 绘制编码值与 RPM (4 行, 各自一行)
 */
static void ui_encoder_draw_enc(void)
{
    char buf[20];

    /* ENC1 */
    ui_fill_rect(28, Y_ENC1, 96, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%8d", encoder_get_count(ENCODER_1));
    ui_draw_string(28, Y_ENC1, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    /* RPM1 */
    ui_fill_rect(28, Y_RPM1, 96, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%7.1f rpm", s_last_rpm[0]);
    ui_draw_string(28, Y_RPM1, buf, TFT_COLOR_GREEN, TFT_COLOR_BLACK);

    /* ENC2 */
    ui_fill_rect(28, Y_ENC2, 96, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%8d", encoder_get_count(ENCODER_2));
    ui_draw_string(28, Y_ENC2, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    /* RPM2 */
    ui_fill_rect(28, Y_RPM2, 96, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%7.1f rpm", s_last_rpm[1]);
    ui_draw_string(28, Y_RPM2, buf, TFT_COLOR_GREEN, TFT_COLOR_BLACK);
}

/**
 * @brief 完整重画模式行 (5 个模式, 选中反白)
 */
static void ui_encoder_draw_mode_full(void)
{
    for (uint8_t i = 0; i < MODE_MAX; i++) {
        uint8_t x = MODE_X_BASE + i * MODE_X_STEP;
        if (i == (uint8_t)s_mode) {
            ui_fill_rect(x, Y_MODE, MODE_X_STEP - 2, UI_FONT_H, TFT_COLOR_WHITE);
            ui_draw_string(x, Y_MODE, MODE_NAMES[i], TFT_COLOR_BLACK, TFT_COLOR_WHITE);
        } else {
            ui_fill_rect(x, Y_MODE, MODE_X_STEP - 2, UI_FONT_H, TFT_COLOR_BLACK);
            ui_draw_string(x, Y_MODE, MODE_NAMES[i], TFT_COLOR_WHITE, TFT_COLOR_BLACK);
        }
    }
}

/**
 * @brief 增量更新模式行 (旧模式恢复正常, 新模式反白)
 */
static void ui_encoder_draw_mode_delta(motor_mode_t prev_mode)
{
    /* 旧模式取消反白 */
    {
        uint8_t x = MODE_X_BASE + (uint8_t)prev_mode * MODE_X_STEP;
        ui_fill_rect(x, Y_MODE, MODE_X_STEP - 2, UI_FONT_H, TFT_COLOR_BLACK);
        ui_draw_string(x, Y_MODE, MODE_NAMES[prev_mode], TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    }
    /* 新模式反白 (如果相同则跳过, 避免重复画) */
    if (prev_mode != s_mode) {
        uint8_t x = MODE_X_BASE + (uint8_t)s_mode * MODE_X_STEP;
        ui_fill_rect(x, Y_MODE, MODE_X_STEP - 2, UI_FONT_H, TFT_COLOR_WHITE);
        ui_draw_string(x, Y_MODE, MODE_NAMES[s_mode], TFT_COLOR_BLACK, TFT_COLOR_WHITE);
    }
}

/**
 * @brief 绘制状态行 (根据当前模式显示不同内容)
 *        M1/M2/BOTH → RUN / STOP
 *        DIR        → FORWARD / REVERSE
 *        SPD        → 档位数值
 */
static void ui_encoder_draw_stat(void)
{
    char buf[20];

    ui_fill_rect(36, Y_STAT, 90, UI_FONT_H, TFT_COLOR_BLACK);
    switch (s_mode) {
        case MODE_M1:
        case MODE_M2:
        case MODE_BOTH:
            ui_draw_string(36, Y_STAT, s_running ? "RUN" : "STOP",
                           s_running ? TFT_COLOR_GREEN : TFT_COLOR_RED,
                           TFT_COLOR_BLACK);
            break;
        case MODE_DIR:
            snprintf(buf, sizeof(buf), "%s",
                     s_dir == MOTOR_DIR_FORWARD ? "FORWARD" : "REVERSE");
            ui_draw_string(36, Y_STAT, buf,
                           s_dir == MOTOR_DIR_FORWARD ? TFT_COLOR_GREEN : TFT_COLOR_YELLOW,
                           TFT_COLOR_BLACK);
            break;
        case MODE_SPEED:
            snprintf(buf, sizeof(buf), "LV %3d", (int)SPEED_LEVELS[s_speed_idx]);
            ui_draw_string(36, Y_STAT, buf, TFT_COLOR_CYAN, TFT_COLOR_BLACK);
            break;
        default: break;
    }
}

/* ========== 页面回调 ========== */

static void page_enter(void)
{
    /* 进入时复位运行标志, 保留方向/档位设定 */
    s_running    = false;
    s_last_rpm[0] = 0.0f;
    s_last_rpm[1] = 0.0f;

    /* 进入时先停车 */
    motor_brake_all();

    ui_encoder_draw_static();
    ui_encoder_draw_enc();
    ui_encoder_draw_mode_full();
    ui_encoder_draw_stat();
}

static void page_exit(void)
{
    /* 退出时务必停车 */
    motor_brake_all();
}

static void page_key(ui_key_t key)
{
    if (key == UI_KEY_K1) {
        /* 退出回菜单 (on_exit 自动停车) */
        lcd_ui_switch_to(UI_PAGE_MENU);
        return;
    }

    if (key == UI_KEY_K3) {
        /* K3: 循环切换模式
         *   - 从 M1/M2/BOTH 切到 DIR/SPD: 强制进入设定模式 (停车 + 清运行标志)
         *     保证状态行与实际硬件同步 (切到 DIR/SPD 时电机确实停了)
         *   - 从 DIR/SPD 切回 M1/M2/BOTH: 保留 s_running=false, 等用户按 K2 启动 */
        motor_mode_t prev_mode = s_mode;
        s_mode = (motor_mode_t)(((uint8_t)s_mode + 1) % MODE_MAX);

        if (prev_mode != MODE_DIR && prev_mode != MODE_SPEED &&
            (s_mode       == MODE_DIR || s_mode       == MODE_SPEED)) {
            /* M* → DIR/SPD: 强制进入设定模式 */
            enter_setting_mode();
        }

        ui_encoder_draw_mode_delta(prev_mode);
        ui_encoder_draw_stat();
        return;
    }

    if (key == UI_KEY_K2) {
        /* K2: 按当前模式分别处理 */
        switch (s_mode) {
            case MODE_M1:
            case MODE_M2:
            case MODE_BOTH:
                /* 启动/停车 (以当前方向 + 当前档位实际转动) */
                s_running = !s_running;
                apply_motor_state();
                ui_encoder_draw_stat();
                break;
            case MODE_DIR:
                /* 切换方向 (设定, 不转) */
                s_dir = (s_dir == MOTOR_DIR_FORWARD)
                        ? MOTOR_DIR_REVERSE : MOTOR_DIR_FORWARD;
                ui_encoder_draw_stat();
                break;
            case MODE_SPEED:
                /* 循环档位 (设定, 不转) */
                s_speed_idx = (s_speed_idx + 1) % SPEED_LEVEL_COUNT;
                ui_encoder_draw_stat();
                break;
            default: break;
        }
        return;
    }
}

static void page_refresh(void)
{
    /* 每 150ms 重算 RPM 并刷新 (lcd_ui_loop 内部已节流) */
    s_last_rpm[0] = encoder_get_rpm(ENCODER_1, 150);
    s_last_rpm[1] = encoder_get_rpm(ENCODER_2, 150);
    ui_encoder_draw_enc();
}

/* ========== 页面注册 ========== */

static const ui_page_t encoder_page = {
    .id         = UI_PAGE_ENCODER,
    .title      = "Encoder & Motor",
    .on_enter   = page_enter,
    .on_exit    = page_exit,
    .on_key     = page_key,
    .on_refresh = page_refresh,
};

esp_err_t ui_encoder_register(void)
{
    return lcd_ui_register_page(&encoder_page);
}