//
// Created by Claude on 2026/1/5.
// ESP32 通信模块 FreeRTOS 任务头文件
//

#ifndef FREERTOS_TEST_ESP32_TASK_H
#define FREERTOS_TEST_ESP32_TASK_H

#include "APP/bsp_system.h"

// ==================== 命令定义 ====================
#define CMD_HR_START        "{CMD:HR_START}"
#define CMD_HR_STOP         "{CMD:HR_STOP}"
#define CMD_TEMP_START      "{CMD:TEMP_START}"
#define CMD_TEMP_STOP       "{CMD:TEMP_STOP}"
#define CMD_MPU_START       "{CMD:MPU_START}"
#define CMD_MPU_STOP        "{CMD:MPU_STOP}"
#define CMD_GPS_START       "{CMD:GPS_START}"
#define CMD_GPS_STOP        "{CMD:GPS_STOP}"
#define CMD_MQ2_START       "{CMD:MQ2_START}"
#define CMD_MQ2_STOP        "{CMD:MQ2_STOP}"

// ==================== 配置参数 ====================
#define ESP32_RX_BUFFER_SIZE    256
#define ESP32_TX_BUFFER_SIZE    128
#define ESP32_DMA_BUFFER_SIZE   128

// ==================== 公共接口 ====================
void esp32_task_init(void);
void esp32_task_start(void);
void esp32_task_stop(void);
uint8_t esp32_is_running(void);

// 数据发送接口（DMA 非阻塞）
void esp32_send_heartrate(uint8_t hr, uint8_t spo2);
void esp32_send_temperature(float temp, float humi);
void esp32_send_mpu6050(float pitch, float roll, float yaw);
void esp32_send_gps(float lat, float lon, uint8_t satellites);
void esp32_send_mq2(uint16_t adc_value, uint8_t alarm);
void esp32_send_status(const char *status);
void esp32_send_event(const char *event);
void esp32_send_sensor_state(const char *sensor, uint8_t on);  // 传感器状态通知

// UART 回调（由 usart.c 调用）
void esp32_uart_rx_callback(uint8_t *data, uint16_t len);
void esp32_uart_tx_callback(void);

#endif //FREERTOS_TEST_ESP32_TASK_H
