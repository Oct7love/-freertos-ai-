//
// Created by Claude on 2026/1/4.
// MAX30102 心率血氧传感器 FreeRTOS 任务接口
//

#ifndef FREERTOS_TEST_MAX30102_TASK_H
#define FREERTOS_TEST_MAX30102_TASK_H

#include "APP/bsp_system.h"

// ==================== 硬件配置 ====================
#define MAX30102_ADDR               0xAE            // I2C 地址 (7位地址0x57左移1位)
#define MAX30102_I2C_TIMEOUT        100             // I2C 超时 (ms)

// ==================== 算法参数 ====================
#define MAX30102_BUFFER_SIZE        100             // 采样缓冲区大小
#define MAX30102_SAMPLE_RATE        100             // 采样率 (Hz)
#define MAX30102_MIN_HEART_RATE     40              // 最小心率 (BPM)
#define MAX30102_MAX_HEART_RATE     200             // 最大心率 (BPM)
#define MAX30102_MIN_SPO2           70              // 最小血氧 (%)
#define MAX30102_FINGER_THRESHOLD   25000           // 手指检测阈值（降低，避免误判）

// ==================== 寄存器地址 ====================
#define MAX30102_REG_INTR_STATUS_1  0x00
#define MAX30102_REG_INTR_STATUS_2  0x01
#define MAX30102_REG_INTR_ENABLE_1  0x02
#define MAX30102_REG_INTR_ENABLE_2  0x03
#define MAX30102_REG_FIFO_WR_PTR    0x04
#define MAX30102_REG_OVF_COUNTER    0x05
#define MAX30102_REG_FIFO_RD_PTR    0x06
#define MAX30102_REG_FIFO_DATA      0x07
#define MAX30102_REG_FIFO_CONFIG    0x08
#define MAX30102_REG_MODE_CONFIG    0x09
#define MAX30102_REG_SPO2_CONFIG    0x0A
#define MAX30102_REG_LED1_PA        0x0C            // 红光 LED
#define MAX30102_REG_LED2_PA        0x0D            // 红外 LED
#define MAX30102_REG_PILOT_PA       0x10
#define MAX30102_REG_MULTI_LED_CTRL1 0x11
#define MAX30102_REG_MULTI_LED_CTRL2 0x12
#define MAX30102_REG_TEMP_INT       0x1F
#define MAX30102_REG_TEMP_FRAC      0x20
#define MAX30102_REG_TEMP_CONFIG    0x21
#define MAX30102_REG_REV_ID         0xFE
#define MAX30102_REG_PART_ID        0xFF            // 应返回 0x15

// ==================== 状态枚举 ====================
typedef enum {
    MAX30102_OK = 0,
    MAX30102_ERROR,
    MAX30102_TIMEOUT,
    MAX30102_NOT_FOUND,
    MAX30102_NO_FINGER
} MAX30102_Status_t;

// 心率状态
typedef enum {
    HR_STATUS_INVALID = 0,      // 无效
    HR_STATUS_CALCULATING,      // 计算中
    HR_STATUS_VALID,            // 有效
    HR_STATUS_NO_FINGER         // 未检测到手指
} MAX30102_HR_Status_t;

// ==================== 数据结构 ====================
typedef struct {
    // 心率数据
    uint16_t heart_rate;        // 心率 (BPM)
    uint8_t hr_valid;           // 心率有效标志
    MAX30102_HR_Status_t hr_status;  // 心率状态

    // 血氧数据
    uint8_t spo2;               // 血氧饱和度 (%)
    uint8_t spo2_valid;         // 血氧有效标志

    // 传感器状态
    uint8_t finger_detected;    // 手指检测标志
    uint8_t signal_quality;     // 信号质量 (0-100)

    // 原始数据（调试用）
    uint32_t red_raw;           // 红光原始值
    uint32_t ir_raw;            // 红外原始值

    // 温度（传感器内置）
    float temperature;          // 芯片温度 (°C)

    // 整体状态
    uint8_t is_valid;           // 数据整体有效标志
} MAX30102_Data_t;

// ==================== 任务接口 ====================
// 初始化 MAX30102 任务（默认挂起状态）
void max30102_task_init(void);

// 启动/停止任务
void max30102_task_start(void);
void max30102_task_stop(void);
uint8_t max30102_is_running(void);

// 获取数据（线程安全）
void max30102_get_data(uint16_t *heart_rate, uint8_t *spo2, uint8_t *valid);
void max30102_get_full_data(MAX30102_Data_t *data);
uint8_t max30102_is_finger_detected(void);
uint8_t max30102_get_signal_quality(void);
MAX30102_HR_Status_t max30102_get_hr_status(void);

// 传感器控制
MAX30102_Status_t max30102_reset(void);
float max30102_read_temperature(void);

#endif //FREERTOS_TEST_MAX30102_TASK_H
