/*
 * @Author: chenxingxing 812150818@qq.com
 * @Date: 2025-12-08 20:20:55
 * @LastEditors: chenxingxing 812150818@qq.com
 * @LastEditTime: 2025-12-08 20:26:22
 * @FilePath: \ai-esp32s3\main\drivers\led\led.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */


#include "led.h"

/**
 * @description: led初始化
 * @param {void} 
 * @return {void}
 */
void led_init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<GPIO_NUM_10);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

/**
 * @description: led关闭
 * @param {void} 
 * @return {void}
 */
void led_off(void)
{
    gpio_set_level(GPIO_NUM_10, 1);
}

/**
 * @description: led打开
 * @param {void} 
 * @return {void}
 */
void led_on(void)
{
    gpio_set_level(GPIO_NUM_10, 0);
}