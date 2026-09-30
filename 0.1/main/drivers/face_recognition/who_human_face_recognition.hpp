#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_camera.h"

/* 按键状态配置参数 */
typedef enum
{
    KEY_SHORT_PRESS = 1,        /* 短按状态 */
    KEY_LONG_PRESS,             /* 长按状态 */
    KEY_DOUBLE_CLICK,           /* 双击状态 */
} key_state_t;

/* 按键扫描状态 */
typedef struct
{
    gpio_num_t      io_num;     /* 按键IO */
    key_state_t     state;      /* 按键状态 */
} key_scan_state_t;

typedef enum
{
    IDLE = 0,                   /* 空闲状态 */
    DETECT,                     /* 检测状态 */
    ENROLL,                     /* 注册状态 */
    RECOGNIZE,                  /* 识别状态 */
    DELETE,                     /* 删除状态 */
} recognizer_state_t;

typedef enum
{
    SHOW_STATE_IDLE,            /* 空闲 */
    SHOW_STATE_DELETE,          /* 删除 */
    SHOW_STATE_RECOGNIZE,       /* 识别 */
    SHOW_STATE_ENROLL,          /* 注册 */
} show_state_t;

/* 长按或者双击时间 */
#define LONG_PRESS_THRESH       700000
#define DOUBLE_CLICK_THRESH     300000

#define RGB565_MASK_RED         0xF800
#define RGB565_MASK_GREEN       0x07E0
#define RGB565_MASK_BLUE        0x001F
#define FRAME_DELAY_NUM         16

void human_face_recognition_start(void);
