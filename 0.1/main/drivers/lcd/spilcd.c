#include "spilcd.h"
#include "spilcdfont.h"

#include <jpeg_decoder.h>

#define TAG "spilcd"



ledc_channel_config_t lcd_channel_ = {0};
ledc_timer_bit_t LEDC_TIMER_BIT = LEDC_TIMER_13_BIT;


DRAM_ATTR uint8_t refresh_done_flag = 0;    
esp_lcd_panel_handle_t panel_handle = NULL;
_spilcd_dev spilcddev;

esp_lcd_panel_io_handle_t io_handle = NULL; 



static uint8_t *img_camera_buf_ = NULL;

/**
 * @brief 背光PWM初始化
 * 
 * @param resolution 分辨率
 * @param freq_hz 频率
 * @param speed_mode 模式
 * @param timer_num 定时器编号
 */
void ledc_timer_init(ledc_timer_bit_t resolution,
                            uint32_t freq_hz,
                            ledc_mode_t speed_mode,
                            ledc_timer_t timer_num)
{
    ledc_timer_config_t ledc_timer = {};
    //设置PWM的分辨率，这里设置13位，即0-8191，也就是调光的细腻度
    ledc_timer.duty_resolution = resolution;   
    //PWM的频率，频率过低会导致LED闪烁，高于200Hz的闪烁基本人眼无法察觉
    ledc_timer.freq_hz = freq_hz;

    //选择低速模式，
    ledc_timer.speed_mode = speed_mode;            
    ledc_timer.timer_num = timer_num;               
    ledc_timer.clk_cfg = LEDC_AUTO_CLK; 

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
}


/**
 * 初始化背光
 * @param pin 背光引脚
 * @param timer_num 定时器编号
 * @param ledc_timer_bit 定时器分辨率
 * @param channel 定时器通道
 * @param is_output_invert 背光输出是否翻转

 */
void spilcd_back_light_init(gpio_num_t pin,
                        ledc_timer_t timer_num,
                        ledc_timer_bit_t ledc_timer_bit,
                        ledc_channel_t channel,
                        bool is_output_invert) 
{ 
    LEDC_TIMER_BIT = ledc_timer_bit;
    lcd_channel_.channel    = channel;
    //初始化时的占空比范围0-8191（针对LEDC_TIMER_13_BIT），设置0表示为关闭状态 
    lcd_channel_.duty       = 0;
    lcd_channel_.gpio_num   = pin;
    lcd_channel_.speed_mode = LEDC_LOW_SPEED_MODE;

    // 相位偏移，我们不需要相位偏移，所以设置为0
    lcd_channel_.hpoint     = 0;
    // 使用的定时器编号 0-3
    lcd_channel_.timer_sel  = timer_num;
    lcd_channel_.flags.output_invert = is_output_invert ? 0x01 : 0x00; 
    ESP_ERROR_CHECK(ledc_channel_config(&lcd_channel_));  
}

/***
 * 设置背光亮度
 * @param brightness 背光亮度百分比
 */
void spilcd_set_brightness(uint8_t brightness)
{  
    uint32_t duty = ((1<<LEDC_TIMER_BIT)-1)*brightness/100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, lcd_channel_.channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, lcd_channel_.channel);
}



static bool notify_lcd_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
   // bool temp = (void *)user_ctx;
    refresh_done_flag = 1;
    return false;
}

// 液晶屏初始化
esp_err_t bsp_display_new(void)
{

    esp_err_t ret = ESP_OK;

    // 初始化SPI总线
    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = BSP_LCD_SPI_CLK,
        .mosi_io_num = BSP_LCD_SPI_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = BSP_LCD_H_RES * BSP_LCD_V_RES * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(BSP_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    // 液晶屏控制IO初始化 --DC CS
    ESP_LOGD(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BSP_LCD_DC,
        .cs_gpio_num = BSP_LCD_SPI_CS,
        .pclk_hz = BSP_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 2,
        .trans_queue_depth = 10,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, &io_handle), err, TAG, "New panel IO failed");
   
    // 初始化液晶屏驱动芯片ST7789
    ESP_LOGD(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle), err, TAG, "New panel failed");
    
    esp_lcd_panel_reset(panel_handle);  // 液晶屏复位
    esp_lcd_panel_init(panel_handle);  // 初始化配置寄存器
    esp_lcd_panel_invert_color(panel_handle, true); // 颜色反转
    esp_lcd_panel_swap_xy(panel_handle, true);  // 显示翻转 
    esp_lcd_panel_mirror(panel_handle, true, false); // 镜像
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lcd_flush_ready,
    };
    /* 注册屏幕刷新完成回调函数 */
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, NULL));
    spilcddev.pheight = BSP_LCD_V_RES ;  /* 高度 */
    spilcddev.pwidth  = BSP_LCD_H_RES;   /* 宽度 */
    spilcd_display_dir(1);      /* 横屏显示 */   
    spilcd_clear(WHITE);        /* 清屏 */
    return ret;

err:
    if (panel_handle) {
        esp_lcd_panel_del(panel_handle);
    }
    if (io_handle) {
        esp_lcd_panel_io_del(io_handle);
    }
    spi_bus_free(BSP_LCD_SPI_NUM);
    return ret;
}

// LCD显示初始化
esp_err_t spilcd_init(void)
{
    esp_err_t ret = ESP_OK;
    //背光初始化
    ledc_timer_init(LEDC0_TIMER_BIT,4000,LEDC_LOW_SPEED_MODE,LEDC_TIMER_0);
    spilcd_back_light_init(BSP_LCD_BACKLIGHT,LEDC_TIMER_0,LEDC0_TIMER_BIT,LEDC_CHANNEL_1,false);

    ret = bsp_display_new(); // 液晶屏驱动初始化
    lcd_set_color(0x0000); // 设置整屏背景黑色
    ret = esp_lcd_panel_disp_on_off(panel_handle, true); // 打开液晶屏显示
    spilcd_set_brightness(100);; // 打开背光显示

    // 分配RGB565缓冲区 (如 320x240x2 bytes)
    // img_camera_buf_ = (uint8_t *)heap_caps_malloc(spilcddev.width * spilcddev.height * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    // assert(img_camera_buf_ != NULL);

    return  ret;
}

// 显示图片
void lcd_draw_pictrue(int x_start, int y_start, int x_end, int y_end, const unsigned char *gImage)
{
    // 分配内存 分配了需要的字节大小 且指定在外部SPIRAM中分配
    size_t pixels_byte_size = (x_end - x_start)*(y_end - y_start) * 2;
    uint16_t *pixels = (uint16_t *)heap_caps_malloc(pixels_byte_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (NULL == pixels)
    {
        ESP_LOGE(TAG, "Memory for bitmap is not enough");
        return;
    }
    memcpy(pixels, gImage, pixels_byte_size);  // 把图片数据拷贝到内存
    esp_lcd_panel_draw_bitmap(panel_handle, x_start, y_start, x_end, y_end, (uint16_t *)pixels); // 显示整张图片数据
    heap_caps_free(pixels);  // 释放内存
}

// 液晶屏显示内容 
void lcd_draw_bitmap(int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    esp_lcd_panel_draw_bitmap(panel_handle, x_start, y_start, x_end, y_end, color_data);
}


// 设置液晶屏颜色
void lcd_set_color(uint16_t color)
{
    // 分配内存 这里分配了液晶屏一行数据需要的大小
    uint16_t *buffer = (uint16_t *)heap_caps_malloc(BSP_LCD_H_RES * sizeof(uint16_t), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    
    if (NULL == buffer)
    {
        ESP_LOGE(TAG, "Memory for bitmap is not enough");
    }
    else
    {
        for (size_t i = 0; i < BSP_LCD_H_RES; i++) // 给缓存中放入颜色数据
        {
            buffer[i] = color;
        }
        for (int y = 0; y < 240; y++) // 显示整屏颜色
        {
            esp_lcd_panel_draw_bitmap(panel_handle, 0, y, 320, y+1, buffer);
        }
        free(buffer); // 释放内存
    }
}



/**
 * @brief       设置屏幕方向
 * @param       dir: 0为竖屏，1为横屏
 * @retval      无
 */
void spilcd_display_dir(uint8_t dir)
{
    spilcddev.dir = dir;

    if (spilcddev.dir == 0)         /* 竖屏 */
    {
        spilcddev.width = spilcddev.pheight;
        spilcddev.height = spilcddev.pwidth;
    }
    else if (spilcddev.dir == 1)    /* 横屏 */
    {
        spilcddev.width = spilcddev.pwidth;
        spilcddev.height = spilcddev.pheight;
    }
    esp_lcd_panel_swap_xy(panel_handle, DISPLAY_SWAP_XY);
    esp_lcd_panel_mirror(panel_handle, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
}

/**
 * @brief       清屏
 * @param       color: 颜色值
 * @retval      无
 */
void spilcd_clear(uint16_t color)
{
    /* 以40行作为缓冲,提高速率,若出现内存不足,可以减少缓冲行数 */
    uint16_t *buffer = heap_caps_malloc(spilcddev.width * sizeof(uint16_t) * 40, MALLOC_CAP_DMA);
    uint16_t color_tmp = ((color & 0x00FF) << 8) | ((color & 0xFF00) >> 8);   /* 需要转换一下颜色值 */
    if (NULL == buffer)
    {
        ESP_LOGE("TAG", "Memory for bitmap is not enough");
    }
    else
    {
        for (uint32_t i = 0; i < spilcddev.width * 40; i++)
        {
            buffer[i] = color_tmp;
        }
        
        for (uint16_t y = 0; y < spilcddev.height; y+=40)
        {
            esp_lcd_panel_draw_bitmap(panel_handle, 0, y, spilcddev.width, y + 40, buffer);
        }
    }

    refresh_done_flag = 0;

    do
    {
        /* 等待内部缓存刷新完成 */
        vTaskDelay(1);
    }
    while (refresh_done_flag != 1);

    heap_caps_free(buffer);
}

/**
 * @brief       在指定区域内填充单个颜色
 * @param       (sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex - sx + 1) * (ey - sy + 1)
 * @param       color:要填充的颜色
 * @retval      无
 */
void spilcd_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color)
{
    /* 计算填充区域宽度 */
    uint16_t width = ex -sx;
    uint16_t height = ey - sy;

    /* 分配内存 */
    uint16_t *buffer = heap_caps_malloc(width * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
    uint16_t color_tmp = ((color & 0x00FF) << 8) | ((color & 0xFF00) >> 8);   /* 需要转换一下颜色值 */

    if (NULL == buffer)
    {
        ESP_LOGE("TAG", "Memory for bitmap is not enough");
    }
    else
    {
        /* 填充颜色 */
        for (uint16_t i = 0; i < width; i++)
        {
            buffer[i] = color_tmp;
        }

        /* 绘制填充区域 */
        for (uint16_t y = 0; y < height; y++)
        {
            esp_lcd_panel_draw_bitmap(panel_handle, sx, sy + y, ex, sy + y + 1, buffer);
        }

        /* 释放内存 */
        heap_caps_free(buffer);
    }
}

/**
 * @brief       绘画一个像素点
 * @param       x    : x轴坐标
 * @param       y    : y轴坐标
 * @param       color: 颜色值
 * @retval      无
 */
void spilcd_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
    uint16_t color_tmp = ((color & 0x00FF) << 8) | ((color & 0xFF00) >> 8);   /* 需要转换一下颜色值 */
    esp_lcd_panel_draw_bitmap(panel_handle, x, y, x + 1, y + 1, &color_tmp);
}

/**
 * @brief       画线函数(直线、斜线)
 * @param       x1,y1   起点坐标
 * @param       x2,y2   终点坐标
 * @param       color 填充颜色
 * @retval      无
 */
void spilcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, row, col;
    delta_x = x2 - x1;
    delta_y = y2 - y1;
    row = x1;
    col = y1;

    if (delta_x > 0)
    {
        incx = 1;
    }
    else if (delta_x == 0)
    {
        incx = 0;
    }
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }

    if (delta_y > 0)
    {
        incy = 1;
    }
    else if (delta_y == 0)
    {
        incy = 0;
    }
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }

    if ( delta_x > delta_y)
    {
        distance = delta_x;
    }
    else
    {
        distance = delta_y;
    }

    for (t = 0; t <= distance + 1; t++)
    {
        spilcd_draw_point(row, col, color);
        xerr += delta_x;
        yerr += delta_y;

        if (xerr > distance)
        {
            xerr -= distance;
            row += incx;
        }

        if (yerr > distance)
        {
            yerr -= distance;
            col += incy;
        }
    }
}

/**
 * @brief       画水平线
 * @param       x0,y0: 起点坐标
 * @param       len  : 线长度
 * @param       color: 矩形的颜色
 * @retval      无
 */
void spilcd_draw_hline(uint16_t x, uint16_t y, uint16_t len, uint16_t color)
{
    /* 确保坐标在LCD范围内 */
    if (len == 0 || x >= spilcddev.width || y >= spilcddev.height) return;

    uint16_t ex = fmin(spilcddev.width - 1, x + len - 1);
    uint16_t ey = y;

    /* 填充颜色区域 */
    uint32_t width = ex - x + 1;
    uint32_t h = ey - y + 1;
    uint16_t *color_buffer = malloc(width * h * sizeof(uint16_t));

    if (color_buffer == NULL) 
    {
        return; /* 检查内存分配是否成功 */
    }

    for (uint32_t i = 0; i < width * h; i++)
    {
        color_buffer[i] = color;
    }

    esp_lcd_panel_draw_bitmap(panel_handle, x, y, ex + 1, ey + 1, color_buffer);
    free(color_buffer);
}

/**
 * @brief       画一个矩形
 * @param       x1,y1   起点坐标
 * @param       x2,y2   终点坐标
 * @param       color 填充颜色
 * @retval      无
 */
void spilcd_draw_rectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,uint16_t color)
{
    spilcd_draw_line(x0, y0, x1, y0,color);
    spilcd_draw_line(x0, y0, x0, y1,color);
    spilcd_draw_line(x0, y1, x1, y1,color);
    spilcd_draw_line(x1, y0, x1, y1,color);
}

/**
 * @brief       画一个圆
 * @param       x0,y0   圆心坐标
 * @param       r   圆半径
 * @param       color 填充颜色
 * @retval      无
 */
void spilcd_draw_circle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color)
{
    int a, b;
    int di;
    a = 0;
    b = r;
    di = 3 - (r << 1);

    while (a <= b)
    {
        spilcd_draw_point(x0 - b, y0 - a, color);
        spilcd_draw_point(x0 + b, y0 - a, color);
        spilcd_draw_point(x0 - a, y0 + b, color);
        spilcd_draw_point(x0 - b, y0 - a, color);
        spilcd_draw_point(x0 - a, y0 - b, color);
        spilcd_draw_point(x0 + b, y0 + a, color);
        spilcd_draw_point(x0 + a, y0 - b, color);
        spilcd_draw_point(x0 + a, y0 + b, color);
        spilcd_draw_point(x0 - b, y0 + a, color);
        a++;

        if (di < 0)
        {
            di += 4 * a + 6;
        }
        else
        {
            di += 10 + 4 * (a - b);
            b--;
        }

        spilcd_draw_point(x0 + a, y0 + b, color);
    }
}

/**
 * @brief       在指定位置显示一个字符
 * @param       x,y  : 坐标
 * @param       chr  : 要显示的字符:" "--->"~"
 * @param       size : 字体大小 12/16/24/32
 * @param       mode : 叠加方式(1); 非叠加方式(0);
 * @param       color : 字符的颜色;
 * @retval      无
 */
void spilcd_show_char(uint16_t x, uint16_t y, uint8_t chr, uint8_t size, uint8_t mode, uint16_t color)
{
    const uint8_t *ch_code;     /* 存放chr字符对应数组的首地址 */
    uint8_t ch_width;           /* 字符的宽度 */
    uint8_t ch_height;          /* 字符的高度 */
    uint8_t ch_size;            /* 字符的大小(字节) */
    uint8_t ch_offset;          /* 字符在字库的相对位置 */
    uint8_t byte_index;         /* 字符对应数据的索引值 */
    uint8_t byte_code;          /* 字符对应数据 */
    uint8_t bit_index;          /* 字符对应字节数据的位索引 */
    uint16_t colortemp = 0;     /* 颜色数据 */
    uint16_t pix_index = 0;
    uint16_t *pcolor = NULL;

    /* 字体大小(字节) =       字体宽度占用字体大小              * 字体高度 */
    ch_size = ((size / 2) / 8 +  (((size / 2) % 8) ? 1 : 0)) * size;         /* 得到字体一个字符对应点阵集所占的字节数 */

    pcolor = heap_caps_malloc(size * (size / 2) * 2, MALLOC_CAP_DMA);        /* 申请大小 */
    uint16_t color_tmp = ((color & 0x00FF) << 8) | ((color & 0xFF00) >> 8);  /* 需要转换一下颜色值 */

    if (NULL == pcolor)
    {
        ESP_LOGE("TAG", "Memory for bitmap is not enough");
    }

    ch_offset = chr - ' ';    /* 得到偏移后的值（ASCII字库是从空格开始取模，所以-' '就是对应字符的字库） */

    switch (size)
    {
        case 12:
            ch_code = (uint8_t *)asc2_1206[ch_offset];  /* 调用1206字体 */
            ch_width = 6;
            ch_height = 12;
            break;

        case 16:
            ch_code = (uint8_t *)asc2_1608[ch_offset];  /* 调用1608字体 */
            ch_width = 8;
            ch_height = 16;
            break;

        case 24:
            ch_code = (uint8_t *)asc2_2412[ch_offset];  /* 调用2412字体 */
            ch_width = 12;
            ch_height = 24;
            break;

        case 32:
            ch_code = (uint8_t *)asc2_3216[ch_offset];  /* 调用3216字体 */
            ch_width = 16;
            ch_height = 32;
            break;

        default:
            return ;
    }

    for (byte_index = 0; byte_index < ch_size; byte_index++)
    {
        byte_code = ch_code[byte_index];    /* 获取字符的点阵数据 */

        for (bit_index = 0; bit_index < 8; bit_index++)   /* 一个字节8个点 */
        {
            if ((byte_code & 0x80) != 0)        /* 有效点,需要显示 */
            {
                colortemp = color_tmp;  
            }
            else if (mode == 0)     /* 无效点,不显示 */
            {
                colortemp = 0xFFFF;
            }

            pcolor[pix_index] = colortemp;
            pix_index++;

            if ((size == 24) && (byte_index % 2))   /* 24号字体比较特殊,奇数字节只有四位有效 */
            {
                if (bit_index == 3)
                {
                    break;
                }
            }

            byte_code <<= 1; /* 移位, 以便获取下一个位的状态 */
        }
    }

    esp_lcd_panel_draw_bitmap(panel_handle, x, y, x + ch_width, y + ch_height, (uint16_t *)pcolor);

    refresh_done_flag = 0;

    do
    {
        /* 等待内部缓存刷新完成 */
        vTaskDelay(1);
    }
    while (refresh_done_flag != 1);

    heap_caps_free(pcolor);
}

/**
 * @brief       m^n函数
 * @param       m,n: 输入参数
 * @retval      m^n次方
 */
uint32_t lcd_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;

    while(n--) result *= m;

    return result;
}

/**
 * @brief       显示len个数字
 * @param       x,y : 起始坐标
 * @param       num : 数值(0 ~ 2^32)
 * @param       len : 显示数字的位数
 * @param       size: 选择字体 12/16/24/32
 * @retval      无
 */
void spilcd_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint16_t color)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for (t = 0; t < len; t++)                                               /* 按总显示位数循环 */
    {
        temp = (num / lcd_pow(10, len - t - 1)) % 10;                       /* 获取对应位的数字 */

        if (enshow == 0 && t < (len - 1))                                   /* 没有使能显示,且还有位要显示 */
        {
            if (temp == 0)
            {
                spilcd_show_char(x + (size / 2)*t, y, ' ', size, 0, color);    /* 显示空格,占位 */
                continue;                                                   /* 继续下个一位 */
            }
            else
            {
                enshow = 1;                                                 /* 使能显示 */
            }

        }

        spilcd_show_char(x + (size / 2)*t, y, temp + '0', size, 0, color);     /* 显示字符 */
    }
}

/**
 * @brief       扩展显示len个数字(高位是0也显示)
 * @param       x,y : 起始坐标
 * @param       num : 数值(0 ~ 2^32)
 * @param       len : 显示数字的位数
 * @param       size: 选择字体 12/16/24/32
 * @param       mode: 显示模式
 *              [7]:0,不填充;1,填充0.
 *              [6:1]:保留
 *              [0]:0,非叠加显示;1,叠加显示.
 * @param       color : 数字的颜色;
 * @retval      无
 */
void spilcd_show_xnum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t mode, uint16_t color)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for (t = 0; t < len; t++)                                                           /* 按总显示位数循环 */
    {
        temp = (num / lcd_pow(10, len - t - 1)) % 10;                                   /* 获取对应位的数字 */

        if (enshow == 0 && t < (len - 1))                                               /* 没有使能显示,且还有位要显示 */
        {
            if (temp == 0)
            {
                if (mode & 0X80)                                                        /* 高位需要填充0 */
                {
                    spilcd_show_char(x + (size / 2)*t, y, '0', size, mode & 0X01, color);  /* 用0占位 */
                }
                else
                {
                    spilcd_show_char(x + (size / 2)*t, y, ' ', size, mode & 0X01, color);  /* 用空格占位 */
                }
                continue;
            }
            else
            {
                enshow = 1;                                                             /* 使能显示 */
            }
        }
        spilcd_show_char(x + (size / 2)*t, y, temp + '0', size, mode & 0X01, color);
    }
}

/**
 * @brief       显示字符串
 * @param       x,y         : 起始坐标
 * @param       width,height: 区域大小
 * @param       size        : 选择字体 12/16/24/32
 * @param       p           : 字符串首地址
 * @retval      无
 */
void spilcd_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, char *p, uint16_t color)
{
    uint8_t x0 = x;
    width += x;
    height += y;

    while ((*p <= '~') && (*p >= ' '))   /* 判断是不是非法字符! */
    {
        if (x >= width)
        {
            x = x0;
            y += size;
        }

        if (y >= height) break;  /* 退出 */

        spilcd_show_char(x, y, *p, size, 0, color);

        x += size / 2;
        p++;
    }
}

/**
 * @brief       绘制一个实体圆的函数
 * @param       center_x,center_y:写入坐标
 * @param       radius:圆的半径
 * @param       color:颜色值
 * @retval      无
 */
void lcd_fill_circle(uint16_t center_x, uint16_t center_y, uint16_t radius, uint16_t color)
{
    uint32_t i;
    uint32_t imax = ((uint32_t)radius * 707) / 1000 + 1;
    uint32_t sqmax = (uint32_t)radius * (uint32_t)radius + (uint32_t)radius / 2;
    uint32_t xr = radius;

    spilcd_draw_hline(center_x - radius, center_y, 2 * radius, color);

    for (i = 1; i <= imax; i++)
    {
        if ((i * i + xr * xr) > sqmax)
        {
            /* draw lines from outside */
            if (xr > imax)
            {
                spilcd_draw_hline (center_x - i + 1, center_y + xr, 2 * (i - 1), color);
                spilcd_draw_hline (center_x - i + 1, center_y - xr, 2 * (i - 1), color);
            }

            xr--;
        }

        /* draw lines from inside (center) */
        spilcd_draw_hline(center_x - xr, center_y + i, 2 * xr, color);
        spilcd_draw_hline(center_x - xr, center_y - i, 2 * xr, color);
    }
}


/**
 * @brief       画粗线
 * @param       x1,y1: 起点坐标
 * @param       x2,y2: 终点坐标
 * @param       size : 线条粗细程度
 * @param       color: 线的颜色
 * @retval      无
 */
void lcd_draw_bline(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size, uint32_t color)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, row, col;

    delta_x = x2 - x1;                          /* 计算坐标增量 */
    delta_y = y2 - y1;
    row = x1;
    col = y1;

    if (delta_x > 0)
    {
        incx = 1;                               /* 设置单步方向 */
    }
    else if (delta_x == 0)
    {
        incx = 0;                               /* 垂直线 */
    }
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }

    if (delta_y > 0)
    {
        incy = 1;
    }
    else if (delta_y == 0)
    {
        incy = 0;                               /* 水平线 */
    }
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }

    if (delta_x > delta_y)
    {
        distance = delta_x;                     /* 选取基本增量坐标轴 */
    }
    else
    {
        distance = delta_y;
    }

    for (t = 0; t <= distance + 1; t++)         /* 画线输出 */
    {
        lcd_fill_circle(row, col, size, color);

        xerr += delta_x;
        yerr += delta_y;

        if (xerr > distance)
        {
            xerr -= distance;
            row += incx;
        }

        if (yerr > distance)
        {
            yerr -= distance;
            col += incy;
        }
    }
}


void lcd_decode_and_display_mjpeg(uint8_t *mjpeg_data, uint32_t data_len)
{
    if(img_camera_buf_ == NULL)
    {
        return;
    }
    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = mjpeg_data,
        .indata_size = data_len,
        .outbuf = (uint8_t *)img_camera_buf_,  // 输出到缓冲区
        .outbuf_size = (uint32_t)(spilcddev.width) * (uint32_t)(spilcddev.height) * sizeof(uint16_t),  // RGB565: 2 bytes per pixel
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = JPEG_IMAGE_SCALE_0,  // 不缩放，保持原尺寸
        .flags.swap_color_bytes = 1,    //交换颜色字节
    }; 
    esp_jpeg_image_output_t outimg;
    esp_err_t ret = esp_jpeg_decode(&jpeg_cfg, &outimg);
    if (ret == ESP_OK)
    {
        //lcd_draw_pictrue(0, 0, spilcddev.width, spilcddev.height, (uint8_t *)img_camera_buf_);

        lcd_draw_bitmap(0, 0,outimg.width, outimg.height, (uint8_t *)img_camera_buf_);
    }
}

