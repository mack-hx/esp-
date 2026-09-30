/*
 * @Author: chenxingxing 812150818@qq.com
 * @Date: 2025-12-15 11:19:19
 * @LastEditors: chenxingxing 812150818@qq.com
 * @LastEditTime: 2025-12-31 15:25:48
 * @FilePath: \ai-esp32s3\main\drivers\lcd\spilcd.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include <string.h>
#include "math.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_lcd_types.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#ifdef __cplusplus
extern "C"
{
#endif


#define BSP_LCD_PIXEL_CLOCK_HZ     (80 * 1000 * 1000)
#define BSP_LCD_SPI_NUM            (SPI3_HOST)
#define LCD_CMD_BITS               (8)
#define LCD_PARAM_BITS             (8)
#define BSP_LCD_BITS_PER_PIXEL     (16)

#define BSP_LCD_H_RES              (320)
#define BSP_LCD_V_RES              (240)

#define BSP_LCD_SPI_MOSI      (GPIO_NUM_40)
#define BSP_LCD_SPI_CLK       (GPIO_NUM_41)
#define BSP_LCD_SPI_CS        (GPIO_NUM_8)
#define BSP_LCD_DC            (GPIO_NUM_39)
#define BSP_LCD_RST           (GPIO_NUM_NC)
#define BSP_LCD_BACKLIGHT     (GPIO_NUM_3)  

#define LEDC0_TIMER_BIT LEDC_TIMER_13_BIT

/* 常用颜色值 */
#define WHITE               0xFFFF      /* 白色 */
#define BLACK               0x0000      /* 黑色 */
#define RED                 0xF800      /* 红色 */
#define GREEN               0x07E0      /* 绿色 */
#define BLUE                0x001F      /* 蓝色 */ 
#define MAGENTA             0XF81F      /* 洋红色 */
#define YELLOW              0XFFE0      /* 黄色 */
#define CYAN                0X07FF      /* 蓝绿色 */

/* 非常用颜色 */
#define BROWN               0XBC40      /* 棕色 */
#define BRRED               0XFC07      /* 棕红色 */
#define GRAY                0X8430      /* 灰色 */ 
#define DARKBLUE            0X01CF      /* 深蓝色 */
#define LIGHTBLUE           0X7D7C      /* 浅蓝色 */ 
#define GRAYBLUE            0X5458      /* 灰蓝色 */ 
#define LIGHTGREEN          0X841F      /* 浅绿色 */  
#define LGRAY               0XC618      /* 浅灰色(PANNEL),窗体背景色 */ 
#define LGRAYBLUE           0XA651      /* 浅灰蓝色(中间层颜色) */ 
#define LBBLUE              0X2B12      /* 浅棕蓝色(选择条目的反色) */ 

typedef struct  
{
    uint32_t pwidth;    /* 临时设定值（宽度） */
    uint32_t pheight;   /* 临时设定值（高度） */
    uint8_t  dir;       /* 屏幕方向 */
    uint16_t width;     /* 宽度 */
    uint16_t height;    /* 高度 */
} _spilcd_dev; 

extern _spilcd_dev spilcddev;;
extern esp_lcd_panel_handle_t panel_handle;
extern esp_lcd_panel_io_handle_t io_handle ;


//0°
// #define DISPLAY_MIRROR_X false
// #define DISPLAY_MIRROR_Y false
// #define DISPLAY_SWAP_XY false
 
//90°
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y true
#define DISPLAY_SWAP_XY true

//180°
// #define DISPLAY_MIRROR_X true
// #define DISPLAY_MIRROR_Y true
// #define DISPLAY_SWAP_XY false
 
// //270°
// #define DISPLAY_MIRROR_X true
// #define DISPLAY_MIRROR_Y false
// #define DISPLAY_SWAP_XY true

/* 函数声明 */
void lcd_set_color(uint16_t color);
void lcd_draw_pictrue(int x_start, int y_start, int x_end, int y_end, const unsigned char *gImage);

void spilcd_display_dir(uint8_t dir);       /* 设置屏幕方向 */
void spilcd_clear(uint16_t color);          /* 清屏 */
void spilcd_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color);           /* 在指定区域内填充单个颜色 */
void spilcd_draw_point(uint16_t x, uint16_t y, uint16_t color);                                 /* 绘画一个像素点 */
void spilcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);      /* 画线函数(直线、斜线) */
void spilcd_draw_hline(uint16_t x, uint16_t y, uint16_t len, uint16_t color);                   /* 画水平线 */
void spilcd_draw_rectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,uint16_t color);  /* 画一个矩形 */
void spilcd_draw_circle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);                  /* 画一个圆 */
void spilcd_show_char(uint16_t x, uint16_t y, uint8_t chr, uint8_t size, uint8_t mode, uint16_t color);                     /* 在指定位置显示一个字符 */
void spilcd_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint16_t color);                      /* 显示len个数字 */
void spilcd_show_xnum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t mode, uint16_t color);       /* 扩展显示len个数字(高位是0也显示) */
void spilcd_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, char *p, uint16_t color);    /* 显示字符串 */
void lcd_fill_circle(uint16_t center_x, uint16_t center_y, uint16_t radius, uint16_t color);
void lcd_draw_bline(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size, uint32_t color);

void ledc_timer_init(ledc_timer_bit_t resolution,uint32_t freq_hz,ledc_mode_t speed_mode,ledc_timer_t timer_num);
void spilcd_back_light_init(gpio_num_t pin,ledc_timer_t timer_num,ledc_timer_bit_t ledc_timer_bit,ledc_channel_t channel,bool is_output_invert) ;
void spilcd_set_brightness(uint8_t brightness);
void lcd_draw_pictrue(int x_start, int y_start, int x_end, int y_end, const unsigned char *gImage);
void lcd_draw_bitmap(int x_start, int y_start, int x_end, int y_end, const void *color_data);
void lcd_decode_and_display_mjpeg(uint8_t *mjpeg_data, uint32_t data_len);
esp_err_t spilcd_init(void);

#ifdef __cplusplus
}
#endif