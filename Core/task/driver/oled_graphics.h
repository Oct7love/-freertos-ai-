#ifndef __OLED_GRAPHICS_H
#define __OLED_GRAPHICS_H

#include <stdint.h>

// 屏幕尺寸
#define OLED_WIDTH  128
#define OLED_HEIGHT 64

// 颜色定义
#define OLED_COLOR_BLACK  0
#define OLED_COLOR_WHITE  1

// 基础图形函数
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void OLED_DrawHLine(uint8_t x, uint8_t y, uint8_t w);
void OLED_DrawVLine(uint8_t x, uint8_t y, uint8_t h);
void OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
void OLED_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

// 数据可视化函数
void OLED_DrawProgressBar(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t value, uint8_t max);
void OLED_DrawProgressBarWithPercent(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t value, uint8_t max);  // 带百分比显示
void OLED_DrawRoundedProgressBar(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t value, uint8_t max);    // 圆角进度条

#endif
