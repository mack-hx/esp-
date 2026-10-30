/**
 * @file font6x8.h
 * @brief 6x8 ASCII 字模 (96 个字符: 空格 ~ ~ )
 * @note  每个字符 6 列 x 8 行, 数据按列存储 (列优先)
 *        每列 1 字节, 高位在上, 低位在下
 *        实际显示宽度 5 列 (最后一列通常为空白, 留作字间距)
 * @author ESP32 机械狗项目
 */

#ifndef FONT6X8_H
#define FONT6X8_H

#include <stdint.h>

/* 字模尺寸 */
#define FONT6X8_WIDTH   6       // 每字符列数

/* ASCII 范围 */
#define FONT6X8_FIRST   32      // ' ' (空格)
#define FONT6X8_LAST    127     // DEL 之前 (含 '~')
#define FONT6X8_COUNT   (FONT6X8_LAST - FONT6X8_FIRST + 1)  // 96 个

/**
 * @brief 获取指定 ASCII 字符的字模数据
 * @param ch ASCII 字符 (32-127), 超出范围返回空格字模
 * @return 6 字节字模数据 (每列 1 字节, 高位在上)
 */
const uint8_t *font6x8_get(uint8_t ch);

#endif // FONT6X8_H