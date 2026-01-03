//
// Created by 13615 on 2026/1/3.
// MQ2 FreeRTOS 任务接口
//

#ifndef FREERTOS_TEST_MQ2_TASK_H
#define FREERTOS_TEST_MQ2_TASK_H

// 只包含必要头文件，避免循环依赖
#include "APP/bsp_system.h"

// 初始化MQ2任务（默认挂起状态）
void mq2_task_init(void);

// 获取MQ2数据（线程安全）
void mq2_get_data(float *ppm, MQ2_AlarmLevel_t *alarm, uint8_t *valid);

// 任务控制接口
void mq2_task_start(void);      // 启动/恢复采集
void mq2_task_stop(void);       // 停止/挂起采集
uint8_t mq2_is_running(void);   // 查询运行状态：1=运行中，0=已停止

#endif //FREERTOS_TEST_MQ2_TASK_H
