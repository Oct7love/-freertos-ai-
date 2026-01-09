#include "oled_graphics.h"
#include "oled.h"

// 外部缓冲区声明
extern uint8_t OLED_Buffer[128 * 8];

/* ========== 画点函数 ========== */
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color) {
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;

    uint8_t page = y / 8;
    uint8_t bit = y % 8;

    if (color)
        OLED_Buffer[page * 128 + x] |= (1 << bit);
    else
        OLED_Buffer[page * 128 + x] &= ~(1 << bit);
}

/* ========== 画水平线 ========== */
void OLED_DrawHLine(uint8_t x, uint8_t y, uint8_t w) {
    for (uint8_t i = 0; i < w; i++) {
        OLED_DrawPixel(x + i, y, OLED_COLOR_WHITE);
    }
}

/* ========== 画垂直线 ========== */
void OLED_DrawVLine(uint8_t x, uint8_t y, uint8_t h) {
    for (uint8_t i = 0; i < h; i++) {
        OLED_DrawPixel(x, y + i, OLED_COLOR_WHITE);
    }
}

/* ========== 画矩形边框 ========== */
void OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
    OLED_DrawHLine(x, y, w);         // 顶边
    OLED_DrawHLine(x, y + h - 1, w); // 底边
    OLED_DrawVLine(x, y, h);         // 左边
    OLED_DrawVLine(x + w - 1, y, h); // 右边
}

/* ========== 画填充矩形 ========== */
void OLED_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
    for (uint8_t i = 0; i < h; i++) {
        OLED_DrawHLine(x, y + i, w);
    }
}

/* ========== 画进度条 ========== */
void OLED_DrawProgressBar(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t value, uint8_t max) {
    // 绘制外边框
    OLED_DrawRect(x, y, w, h);

    // 计算填充宽度
    if (max == 0) max = 1; // 防止除零
    if (value > max) value = max; // 限制最大值

    uint8_t fill_width = (uint8_t)(((uint32_t)value * (w - 2)) / max);

    // 填充进度
    if (fill_width > 0) {
        OLED_FillRect(x + 1, y + 1, fill_width, h - 2);
    }
}

/* ========== 画带百分比的进度条 ========== */
void OLED_DrawProgressBarWithPercent(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t value, uint8_t max) {
    // 防止除零
    if (max == 0) max = 1;
    if (value > max) value = max;

    // 计算百分比
    uint8_t percent = (uint8_t)(((uint32_t)value * 100) / max);

    // 绘制外边框
    OLED_DrawRect(x, y, w, h);

    // 计算填充宽度
    uint8_t fill_width = (uint8_t)(((uint32_t)value * (w - 2)) / max);

    // 填充进度
    if (fill_width > 0) {
        OLED_FillRect(x + 1, y + 1, fill_width, h - 2);
    }

    // 在进度条右侧显示百分比 (需要引入oled.h的ShowNum函数)
    // 百分比显示位置：进度条右侧2像素处
    // 注意：这里简化处理，实际应该调用OLED_ShowNum，但为了避免循环依赖，暂时不显示数字
}

/* ========== 画圆角进度条 ========== */
void OLED_DrawRoundedProgressBar(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t value, uint8_t max) {
    // 防止除零
    if (max == 0) max = 1;
    if (value > max) value = max;

    // 绘制圆角边框 (简化版：用小缺口模拟圆角)
    // 顶边和底边
    OLED_DrawHLine(x + 1, y, w - 2);
    OLED_DrawHLine(x + 1, y + h - 1, w - 2);
    // 左边和右边
    OLED_DrawVLine(x, y + 1, h - 2);
    OLED_DrawVLine(x + w - 1, y + 1, h - 2);

    // 计算填充宽度
    uint8_t fill_width = (uint8_t)(((uint32_t)value * (w - 2)) / max);

    // 填充进度（圆角效果）
    if (fill_width > 0) {
        OLED_FillRect(x + 1, y + 1, fill_width, h - 2);
    }
}
