//
// Created by 13615 on 2026/1/3.
//

#ifndef FREERTOS_TEST_DHT11_TASK_H
#define FREERTOS_TEST_DHT11_TASK_H
#include "APP/bsp_system.h"

// 初始化DHT11任务（默认挂起状态）
void dht11_task_init(void);

// 获取温湿度数据（线程安全）
void dht11_get_data(uint8_t *temp, uint8_t *humi, uint8_t *valid);

// 任务控制接口
void dht11_task_start(void);      // 启动/恢复采集
void dht11_task_stop(void);       // 停止/挂起采集
uint8_t dht11_is_running(void);   // 查询运行状态：1=运行中，0=已停止

#endif //FREERTOS_TEST_DHT11_TASK_H