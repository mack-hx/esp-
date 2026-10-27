/**
 * @file ui_mpu9250.c
 * @brief UI MPU9250 IMU 调试页实现
 * @note  - ACC / GYR 采用两列排布 (左右对照, 每轴占一行)
 *        - 显示欧拉角 (Roll/Pitch/Yaw) - 取自 mpu9250_get_euler_angle()
 *        - 显示平衡车重点参数: 姿态角 Pitch / 角速度 gx / 温度 / 陀螺仪零偏
 *        - K1 退出回菜单
 *        - K2 触发陀螺仪校准
 *        - K3 触发软复位
 *        - 当 MPU9250 初始化失败 (模块未接), 显示 "NOT READY" 并禁用 K2/K3
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#include "ui_mpu9250.h"
#include "mpu9250.h"
#include "tft_display.h"
#include "esp_log.h"
#include <stdio.h>

/* MPU9250 是否就绪 (在 page_enter 时通过一次 ping 检测) */
static bool s_ready = false;

/* ========== 绘制坐标常量 (128x160, font6x8, 每行 21 字符) ========== */
#define Y_TITLE        1       /* 标题栏文字 */
#define Y_AXIS         14      /* ACC/GYR 轴标签行 */
#define Y_ACC1         24      /* ax */
#define Y_ACC2         34      /* ay */
#define Y_ACC3         44      /* az */
#define Y_SEP1         56      /* 分隔线 1 */
#define Y_RPY          62      /* RPY 行 */
#define Y_SEP2         74      /* 分隔线 2 */
#define Y_BAL_P        80      /* 平衡参数: Pitch 姿态角 */
#define Y_BAL_G        90      /* 平衡参数: gx 角速度 */
#define Y_BAL_T        100     /* 平衡参数: 温度 */
#define Y_BAL_OFF      110     /* 平衡参数: 陀螺仪零偏 */
#define Y_SEP3         122     /* 分隔线 3 */
#define Y_HINT         130     /* 单行按键提示 */
#define Y_HINT2        140     /* 第二行提示 (校准说明) */

/* 两列排布: 左列 ACC, 右列 GYR */
#define COL_L_X        2       /* 左列 X 起点 */
#define COL_R_X        68      /* 右列 X 起点 */
#define COL_L_VAL_X    26      /* 左列数值 X 起点 */
#define COL_R_VAL_X    92      /* 右列数值 X 起点 */

/**
 * @brief 检测 MPU9250 是否就绪 (通过读取一次数据判断)
 */
static bool check_mpu9250_ready(void)
{
    mpu9250_data_t tmp;
    return (mpu9250_read_data(&tmp) == ESP_OK);
}

/**
 * @brief 绘制静态骨架 (标题 + 两列 ACC/GYR 标签 + RPY + 平衡参数标签 + 分隔线 + 单行提示)
 */
static void ui_mpu9250_draw_static(void)
{
    tft_clear(TFT_COLOR_BLACK);

    /* 顶部反白标题栏 */
    ui_fill_rect(0, 0, UI_SCREEN_W, UI_FONT_H + 2, TFT_COLOR_WHITE);
    ui_draw_string(2, Y_TITLE, "MPU9250", TFT_COLOR_BLACK, TFT_COLOR_WHITE);

    /* 两列标签: 左 ACC (加速度 m/s^2), 右 GYR (角速度 deg/s) */
    ui_draw_string(COL_L_X, Y_AXIS, "ACC m/s2", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(COL_R_X, Y_AXIS, "GYR d/s",   TFT_COLOR_CYAN, TFT_COLOR_BLACK);

    /* 左列 x/y/z 轴标签 */
    ui_draw_string(COL_L_X, Y_ACC1, "ax", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(COL_L_X, Y_ACC2, "ay", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(COL_L_X, Y_ACC3, "az", TFT_COLOR_WHITE, TFT_COLOR_BLACK);

    /* 右列 x/y/z 轴标签 */
    ui_draw_string(COL_R_X, Y_ACC1, "gx", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(COL_R_X, Y_ACC2, "gy", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(COL_R_X, Y_ACC3, "gz", TFT_COLOR_WHITE, TFT_COLOR_BLACK);

    /* 分隔线 1: 两列与 RPY 之间 */
    ui_draw_hline(0, Y_SEP1, UI_SCREEN_W, TFT_COLOR_BLUE);

    /* RPY 标签 */
    ui_draw_string(2, Y_RPY, "RPY", TFT_COLOR_CYAN, TFT_COLOR_BLACK);

    /* 分隔线 2: RPY 与平衡参数之间 */
    ui_draw_hline(0, Y_SEP2, UI_SCREEN_W, TFT_COLOR_BLUE);

    /* 平衡车重点参数标签 */
    ui_draw_string(2,  Y_BAL_P,   "Pitch:", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2,  Y_BAL_G,   "Gyx:",   TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2,  Y_BAL_T,   "Temp:",  TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2,  Y_BAL_OFF, "Off:",   TFT_COLOR_WHITE, TFT_COLOR_BLACK);

    /* 分隔线 3 + 两行按键提示
     * 注意: 屏幕每行 21 字符 (128/6), 超出 ui_draw_string 会自动换行, 故提示串必须 <= 21 字符 */
    ui_draw_hline(0, Y_SEP3, UI_SCREEN_W, TFT_COLOR_BLUE);
    ui_draw_string(2, Y_HINT, "[K1]Bk [K2]Cal [K3]Rs",
                   TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    /* 第二行: 说明 K2 校准的用法。
     * ⚠ 2026-10-31 改: 标定逻辑改成"挑最稳的 0.25 秒窗口", 所以
     *   拿稳 0.25 秒即可, 不再要求全程静止 (手持也能标)。 */
    ui_draw_string(2, Y_HINT2, "Cal: hold 0.25s",
                   TFT_COLOR_YELLOW, TFT_COLOR_BLACK);
}

/**
 * @brief 绘制实时数据 (两列 ACC/GYR + RPY + 平衡车重点参数)
 */
static void ui_mpu9250_draw_data(void)
{
    mpu9250_data_t  data;
    euler_angle_t   angle;
    mpu9250_axis_t  offset;
    char buf[20];

    if (mpu9250_read_data(&data) != ESP_OK) {
        /* 读失败: 重画骨架 (恢复被擦掉的标签) 再提示错误 */
        ui_mpu9250_draw_static();
        ui_fill_rect(0, Y_AXIS, UI_SCREEN_W, Y_SEP3 - Y_AXIS, TFT_COLOR_BLACK);
        ui_draw_string(20, 70, "SENSOR ERR", TFT_COLOR_RED, TFT_COLOR_BLACK);
        return;
    }
    if (mpu9250_get_euler_angle(&angle) != ESP_OK) {
        ui_mpu9250_draw_static();
        ui_fill_rect(0, Y_AXIS, UI_SCREEN_W, Y_SEP3 - Y_AXIS, TFT_COLOR_BLACK);
        ui_draw_string(20, 70, "SENSOR ERR", TFT_COLOR_RED, TFT_COLOR_BLACK);
        return;
    }
    mpu9250_get_gyro_offset(&offset);

    /* 左列: ax / ay / az (黄色) */
    ui_fill_rect(COL_L_VAL_X, Y_ACC1, 40, UI_FONT_H * 3, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%5.2f", data.accel.x);
    ui_draw_string(COL_L_VAL_X, Y_ACC1, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%5.2f", data.accel.y);
    ui_draw_string(COL_L_VAL_X, Y_ACC2, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%5.2f", data.accel.z);
    ui_draw_string(COL_L_VAL_X, Y_ACC3, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    /* 右列: gx / gy / gz (绿色) */
    ui_fill_rect(COL_R_VAL_X, Y_ACC1, 36, UI_FONT_H * 3, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%5.1f", data.gyro.x);
    ui_draw_string(COL_R_VAL_X, Y_ACC1, buf, TFT_COLOR_GREEN, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%5.1f", data.gyro.y);
    ui_draw_string(COL_R_VAL_X, Y_ACC2, buf, TFT_COLOR_GREEN, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%5.1f", data.gyro.z);
    ui_draw_string(COL_R_VAL_X, Y_ACC3, buf, TFT_COLOR_GREEN, TFT_COLOR_BLACK);

    /* RPY: Roll / Pitch / Yaw 单行 (6 字符标签 + 3×5 字符数值 = 21 字符, 刚好一行) */
    ui_fill_rect(24, Y_RPY, 104, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%4.0f %4.0f %4.0f",
             angle.roll, angle.pitch, angle.yaw);
    ui_draw_string(24, Y_RPY, buf, TFT_COLOR_MAGENTA, TFT_COLOR_BLACK);

    /* 平衡车重点参数 */
    /* Pitch 姿态角: 平衡车姿态控制的核心量 */
    ui_fill_rect(40, Y_BAL_P, 84, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%6.2f deg", angle.pitch);
    ui_draw_string(40, Y_BAL_P, buf, TFT_COLOR_MAGENTA, TFT_COLOR_BLACK);

    /* Gyx 角速度: 姿态变化的速率, 阻尼/超前控制输入
     * ⚠ 这里取 gyro.x 是对的 —— 俯仰绕 MPU 的 X 轴 (车体轴向差 90° 置换,
     *   详见 mpu9250.c 的 mpu9250_fuse_euler 注释) */
    ui_fill_rect(40, Y_BAL_G, 84, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%6.2f d/s", data.gyro.x);
    ui_draw_string(40, Y_BAL_G, buf, TFT_COLOR_GREEN, TFT_COLOR_BLACK);

    /* Temp 温度: 器件自热/漂移参考 */
    ui_fill_rect(40, Y_BAL_T, 84, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%5.1f C", data.temperature);
    ui_draw_string(40, Y_BAL_T, buf, TFT_COLOR_CYAN, TFT_COLOR_BLACK);

    /* Off 陀螺仪零偏: 判断校准是否正常 (静止时应接近 0) */
    ui_fill_rect(30, Y_BAL_OFF, 96, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%4.1f/%4.1f/%4.1f",
             offset.x, offset.y, offset.z);
    ui_draw_string(30, Y_BAL_OFF, buf, TFT_COLOR_CYAN, TFT_COLOR_BLACK);
}

/* ========== 页面回调 ========== */

static void page_enter(void)
{
    s_ready = check_mpu9250_ready();
    ui_mpu9250_draw_static();

    if (!s_ready) {
        /* 模块未就绪: 显示 "NOT READY" 提示 */
        ui_fill_rect(0, Y_AXIS, UI_SCREEN_W, Y_SEP3 - Y_AXIS, TFT_COLOR_BLACK);
        ui_draw_string(20, 50, "MPU9250",          TFT_COLOR_RED,    TFT_COLOR_BLACK);
        ui_draw_string(20, 70, "NOT READY",        TFT_COLOR_RED,    TFT_COLOR_BLACK);
        ui_draw_string(2,  100, "Check I2C & module", TFT_COLOR_YELLOW, TFT_COLOR_BLACK);
        return;
    }

    ui_mpu9250_draw_data();
}

static void page_exit(void)
{
    /* MPU9250 持续运行, 无需释放 */
}

static void page_key(ui_key_t key)
{
    if (key == UI_KEY_K1) {
        lcd_ui_switch_to(UI_PAGE_MENU);
    } else if (!s_ready) {
        /* 模块未就绪, K2/K3 无效 */
        return;
    } else if (key == UI_KEY_K2) {
        /* 触发陀螺仪零点校准 (200 次采样, 约 2 秒)
         * ⚠ 2026-10-31 改动: 采样数 200 (原来 100) —— 标定逻辑改成了
         *   "从整段采样里挑最稳的 0.25 秒窗口", 素材越多越容易挑到稳的一段。
         *   拿稳 0.25 秒即可, 不要求全程静止。 */
        ESP_LOGI("UI_MPU9250", "开始陀螺仪校准: 请拿稳设备 0.25 秒 (手拿也行, 别晃)");
        if (mpu9250_calibrate_gyro(200) == ESP_OK) {
            ESP_LOGI("UI_MPU9250", "陀螺仪校准完成");
        } else {
            ESP_LOGE("UI_MPU9250", "陀螺仪校准失败 (沿用 NVS 里的旧零偏), "
                                  "试试拿得更稳一点 (0.25 秒不晃就行)");
        }
    } else if (key == UI_KEY_K3) {
        /* 软复位 (reset 后需重新调用 init) */
        mpu9250_reset();
        mpu9250_init();
        s_ready = check_mpu9250_ready();
    }
}

static void page_refresh(void)
{
    if (!s_ready) {
        /* 周期性重试检测, 模块接上后能自动恢复 */
        if (check_mpu9250_ready()) {
            s_ready = true;
            ui_mpu9250_draw_static();
            ui_mpu9250_draw_data();
        }
        return;
    }
    ui_mpu9250_draw_data();
}

/* ========== 页面注册 ========== */

static const ui_page_t mpu9250_page = {
    .id         = UI_PAGE_MPU9250,
    .title      = "MPU9250",
    .on_enter   = page_enter,
    .on_exit    = page_exit,
    .on_key     = page_key,
    .on_refresh = page_refresh,
};

esp_err_t ui_mpu9250_register(void)
{
    return lcd_ui_register_page(&mpu9250_page);
}
