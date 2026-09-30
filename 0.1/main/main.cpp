/*
 * @Author: chenxingxing 812150818@qq.com
 * @Date: 2025-12-09 11:19:05
 * @LastEditors: chenxingxing 812150818@qq.com
 * @LastEditTime: 2026-01-04 11:53:16
 * @FilePath: \ai-esp32s3\main\main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include <stdio.h>
#include <inttypes.h>
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "spilcd.h"
#include "camera.h"
#include "who_human_face_recognition.hpp"

extern "C"  void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init()); //初始化nvs
    spilcd_init();
    camera_init(BSP_LCD_H_RES,BSP_LCD_V_RES);
    human_face_recognition_start();
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}