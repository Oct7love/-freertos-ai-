//
// Created by 13615 on 2026/1/2.
//

#include "delay.h"
// TIM3句柄（由CubeMX生成）
 extern TIM_HandleTypeDef htim3;
void delay_init(void)
{
    HAL_TIM_Base_Start(&htim3);
}

/**
 * @brief  微秒级精确延时
 * @param  us 延时微秒数
 * @note   基于TIM3硬件定时器，精度1us
 */
void delay_us(uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);           // 复位计数器为0
    while(__HAL_TIM_GET_COUNTER(&htim3) < us);  // 等待计数到us
}
/**
   * @brief  毫秒延时（不推荐在FreeRTOS中使用）
   * @param  ms 延时毫秒数
   */
void delay_ms(uint32_t ms)
{
    while(ms--)
    {
        delay_us(1000);
    }
}