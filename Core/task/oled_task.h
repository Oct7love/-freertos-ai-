//
// Created by 13615 on 2026/1/2.
//

#ifndef FREERTOS_TEST_OLED_TASK_H
#define FREERTOS_TEST_OLED_TASK_H
#include "APP/bsp_system.h"
// OLED显示命令类型
typedef enum {
    OLED_CMD_SHOW_TEXT = 0,   // 显示文本
    OLED_CMD_SHOW_NUM,        // 显示无符号数字
    OLED_CMD_SHOW_SIGNED,     // 显示有符号数字
    OLED_CMD_SHOW_FLOAT,      // 显示浮点数
    OLED_CMD_SHOW_HEX,        // 显示十六进制
    OLED_CMD_CLEAR,           // 清屏
    OLED_CMD_FLUSH,           // 立即刷新
} oled_cmd_type_t;

// OLED显示消息结构体
typedef struct {
    oled_cmd_type_t cmd;
    uint8_t line;             // 行号：1-4
    uint8_t column;           // 列号：1-16
    union {
        char text[32];        // 文本内容
        struct {
            uint32_t value;
            uint8_t length;
        } num;                // 数字显示
        struct {
            int32_t value;
            uint8_t length;
        } signed_num;         // 有符号数字
        struct {
            float value;
            uint8_t int_len;
            uint8_t dec_len;
        } flt;                // 浮点数
        struct {
            uint32_t value;
            uint8_t length;
        } hex;                // 十六进制
    } data;
} oled_msg_t;

// 初始化OLED任务
void oled_task_init(void);

// 对外接口：显示文本（非阻塞）
void oled_show_text(uint8_t line, uint8_t column, const char *text);

// 对外接口：显示无符号数字
void oled_show_num(uint8_t line, uint8_t column, uint32_t num, uint8_t len);

// 对外接口：显示有符号数字
void oled_show_signed_num(uint8_t line, uint8_t column, int32_t num, uint8_t len);

// 对外接口：显示浮点数
void oled_show_float(uint8_t line, uint8_t column, float num, uint8_t int_len, uint8_t dec_len);

// 对外接口：显示十六进制数字
void oled_show_hex(uint8_t line, uint8_t column, uint32_t num, uint8_t len);

// 对外接口：清屏
void oled_clear_screen(void);

// 对外接口：立即刷新屏幕
void oled_flush_now(void);
#endif //FREERTOS_TEST_OLED_TASK_H