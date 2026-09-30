/*
 * @Author: chenxingxing 812150818@qq.com
 * @Date: 2025-12-19 09:31:26
 * @LastEditors: chenxingxing 812150818@qq.com
 * @LastEditTime: 2025-12-19 15:04:43
 * @FilePath: \ai-esp32s3\main\drivers\camera\camera.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include "stdio.h"
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define DEMO_UVC_XFER_BUFFER_SIZE (55 * 1024)

void camera_init(uint16_t frame_width,uint16_t frame_height);
void camera_uvc_cleanup(void);

#ifdef __cplusplus
}
#endif

