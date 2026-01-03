//
// Created by 13615 on 2026/1/3.
//

#ifndef FREERTOS_TEST_DHT11_H
#define FREERTOS_TEST_DHT11_H
#include "APP/bsp_system.h"
// DHT11引脚配置
 #define DHT11_GPIO_PORT         GPIOA
 #define DHT11_GPIO_PIN          GPIO_PIN_8
 #define DHT11_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

// 引脚操作宏
 #define DHT11_PIN_HIGH()    HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_SET)
 #define DHT11_PIN_LOW()     HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_RESET)
 #define DHT11_PIN_READ()    HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN)

// 数据范围
 #define DHT11_TEMP_MIN      0
 #define DHT11_TEMP_MAX      50
 #define DHT11_HUMI_MIN      20
 #define DHT11_HUMI_MAX      90

// 数据验证
#define DHT11_IS_TEMP_VALID(t)  ((t) >= DHT11_TEMP_MIN && (t) <= DHT11_TEMP_MAX)
#define DHT11_IS_HUMI_VALID(h)  ((h) >= DHT11_HUMI_MIN && (h) <= DHT11_HUMI_MAX)

// DHT11数据结构
typedef struct {
    uint8_t temperature;    // 温度（°C）
    uint8_t humidity;       // 湿度（%）
    uint8_t is_valid;       // 数据有效性
} DHT11_Data_t;

// 底层驱动函数
void DHT11_Init(void);
uint8_t DHT11_Read(uint8_t *temp, uint8_t *humi);
#endif //FREERTOS_TEST_DHT11_H