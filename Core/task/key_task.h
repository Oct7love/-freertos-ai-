//
// Created by 13615 on 2026/1/2.
//

#ifndef FREERTOS_TEST_KEY_TASK_H
#define FREERTOS_TEST_KEY_TASK_H
#include "APP/bsp_system.h"
// 按键事件定义
#define KEY_EVENT_KEY1_PRESS   (1 << 0)  // KEY1按下事件
#define KEY_EVENT_KEY2_PRESS   (1 << 1)  // KEY2按下事件
// 初始化按键任务
 void key_task_init(void);

// 获取按键事件组句柄（供其他模块使用）
EventGroupHandle_t key_get_event_group(void);
void key_exti_callback(uint16_t GPIO_Pin);
#endif //FREERTOS_TEST_KEY_TASK_H