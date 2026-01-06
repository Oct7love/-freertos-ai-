//
// Created by Claude on 2026/1/5.
// GPS 模块 FreeRTOS 任务接口
//

#ifndef FREERTOS_TEST_GPS_TASK_H
#define FREERTOS_TEST_GPS_TASK_H

#include "APP/bsp_system.h"

// ==================== 配置参数 ====================
#define GPS_UART_HANDLE         huart2          // GPS 使用 UART2
#define GPS_RX_BUFFER_SIZE      256             // 接收缓冲区大小
#define GPS_PARSE_INTERVAL      100             // 解析间隔 (ms)
#define GPS_PRINT_INTERVAL      2000            // 打印间隔 (ms)

// ==================== GPS 数据结构 ====================
typedef struct {
    // 位置信息
    float latitude;             // 纬度（十进制度数，正=北，负=南）
    float longitude;            // 经度（十进制度数，正=东，负=西）
    char lat_dir;               // 纬度方向 'N'/'S'
    char lon_dir;               // 经度方向 'E'/'W'
    float altitude;             // 海拔高度 (m)

    // 时间信息
    char utc_time[12];          // UTC 时间 "HH:MM:SS.SS"

    // 定位状态
    uint8_t fix_quality;        // 定位质量 (0=无效, 1=GPS, 2=DGPS)
    uint8_t satellites;         // 可见卫星数
    float hdop;                 // 水平精度因子

    // 有效标志
    uint8_t is_valid;           // 数据有效标志
    uint8_t is_fixed;           // 是否已定位
} GPS_Data_t;

// ==================== 状态枚举 ====================
typedef enum {
    GPS_STATUS_INIT = 0,        // 初始化中
    GPS_STATUS_SEARCHING,       // 搜索卫星
    GPS_STATUS_FIXED,           // 已定位
    GPS_STATUS_ERROR            // 错误
} GPS_Status_t;

// ==================== 任务接口 ====================
// 初始化 GPS 任务（默认挂起状态）
void gps_task_init(void);

// 启动/停止任务
void gps_task_start(void);
void gps_task_stop(void);
uint8_t gps_is_running(void);

// 获取数据（线程安全）
void gps_get_data(GPS_Data_t *data);
void gps_get_position(float *lat, float *lon, uint8_t *valid);
uint8_t gps_get_satellites(void);
GPS_Status_t gps_get_status(void);
uint8_t gps_is_fixed(void);

// UART2 数据接收回调（由中断调用）
void gps_uart_rx_callback(uint8_t *data, uint16_t len);

#endif //FREERTOS_TEST_GPS_TASK_H
