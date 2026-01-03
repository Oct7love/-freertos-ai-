//
// Created by 13615 on 2026/1/2.
//

#ifndef FREERTOS_TEST_DELAY_H
#define FREERTOS_TEST_DELAY_H
#include "APP/bsp_system.h"

/**
 * @brief  初始化微秒延时（基于TIM3硬件定时器）
 * @note   必须在main()中，osKernelStart()之前调用
 * @note   CubeMX配置：TIM3 PSC=71, ARR=65535
 *         计算：72MHz/(71+1) = 1MHz → 计数器每+1 = 1us
 */
void delay_init(void);

/**
 * @brief  精确微秒延时（阻塞式）
 * @param  us 延时微秒数（1-65535us，最大65.5ms）
 * @note   完全阻塞当前任务，100% CPU占用
 * @note   仅用于短时间硬件通信（DHT11、超声波等）
 * @warning 不要用于长延时！超过10ms建议用vTaskDelay()
 */
void delay_us(uint32_t us);

/**
 * @brief  毫秒延时（阻塞式）
 * @param  ms 延时毫秒数
 * @note   FreeRTOS环境强烈不推荐！应使用vTaskDelay()
 */
void delay_ms(uint32_t ms);
#endif //FREERTOS_TEST_DELAY_H