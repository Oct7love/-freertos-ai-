//
// Created by 13615 on 2026/1/2.
//

#ifndef FREERTOS_TEST_OLED_TASK_H
#define FREERTOS_TEST_OLED_TASK_H
#include "APP/bsp_system.h"

// 界面枚举
typedef enum {
    UI_PAGE_WELCOME = 0,   // 欢迎界面：Smart Hat
    UI_PAGE_DHT11,         // 温湿度界面
    UI_PAGE_MQ2,           // 气体浓度界面
    UI_PAGE_MPU6050,       // 姿态传感器界面
    UI_PAGE_MAX30102,      // 心率血氧界面
    UI_PAGE_GPS,           // GPS 定位界面
    UI_PAGE_MAX
} ui_page_t;

// OLED显示命令类型
typedef enum {
    OLED_CMD_SHOW_TEXT = 0,   // 显示文本
    OLED_CMD_SHOW_NUM,        // 显示无符号数字
    OLED_CMD_SHOW_SIGNED,     // 显示有符号数字
    OLED_CMD_SHOW_FLOAT,      // 显示浮点数
    OLED_CMD_SHOW_HEX,        // 显示十六进制
    OLED_CMD_CLEAR,           // 清屏
    OLED_CMD_FLUSH,           // 立即刷新
    OLED_CMD_SWITCH_PAGE,     // 切换界面
    OLED_CMD_UPDATE_DHT11,    // 更新温湿度显示
    OLED_CMD_UPDATE_MQ2,      // 更新气体浓度显示
    OLED_CMD_UPDATE_MPU6050,  // 更新姿态显示
    OLED_CMD_UPDATE_MAX30102, // 更新心率血氧显示
    OLED_CMD_UPDATE_GPS,      // 更新GPS显示
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
        ui_page_t page;       // 界面切换目标
        struct {
            uint8_t temp;
            uint8_t humi;
            uint8_t valid;
        } dht11;              // 温湿度数据
        struct {
            float ppm;
            uint8_t alarm;
            uint8_t valid;
        } mq2;                // 气体浓度数据
        struct {
            float pitch;
            float roll;
            float yaw;
            uint32_t steps;
            uint8_t valid;
        } mpu6050;            // 姿态数据
        struct {
            uint16_t heart_rate;
            uint8_t spo2;
            uint8_t finger;
            uint8_t valid;
        } max30102;           // 心率血氧数据
        struct {
            float latitude;
            float longitude;
            uint8_t satellites;
            uint8_t is_fixed;
        } gps;                // GPS数据
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

// 界面控制接口
void oled_switch_page(ui_page_t page);    // 切换到指定界面
void oled_next_page(void);                 // 切换到下一个界面
ui_page_t oled_get_current_page(void);     // 获取当前界面
void oled_update_dht11(uint8_t temp, uint8_t humi, uint8_t valid);  // 更新温湿度显示
void oled_update_mq2(float ppm, uint8_t alarm, uint8_t valid);      // 更新气体浓度显示
void oled_update_mpu6050(float pitch, float roll, float yaw, uint32_t steps, uint8_t valid);  // 更新姿态显示
void oled_update_max30102(uint16_t heart_rate, uint8_t spo2, uint8_t finger, uint8_t valid);  // 更新心率血氧显示
void oled_update_gps(float lat, float lon, uint8_t satellites, uint8_t is_fixed);  // 更新GPS显示

#endif //FREERTOS_TEST_OLED_TASK_H