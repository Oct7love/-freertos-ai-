//
// Created by 13615 on 2026/1/1.
//

#ifndef FREERTOS_TEST_LED_TASK_H
#define FREERTOS_TEST_LED_TASK_H
#include "APP/bsp_system.h"
// LED事件定义（事件组的位定义）
 #define LED_EVENT_HEARTBEAT   (1 << 0)  // 正常心跳
 #define LED_EVENT_ERROR       (1 << 1)  // 错误指示
 #define LED_EVENT_BUSY        (1 << 2)  // 忙碌指示
/**
  * @brief 初始化LED任务（创建任务、事件组、定时器）
  */
 void led_task_init(void);

/**
 * @brief 设置LED模式（通过事件组）
 * @param event LED事件（LED_EVENT_HEARTBEAT/ERROR/BUSY）
 * @note  其他任务调用此函数来改变LED状态
 */
void led_set_mode(uint32_t event);
#endif //FREERTOS_TEST_LED_TASK_H