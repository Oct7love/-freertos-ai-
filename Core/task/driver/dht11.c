//
// Created by 13615 on 2026/1/3.
//

#include "dht11.h"
#include "FreeRTOS.h"
#include "task.h"

// 设置引脚模式
static void DHT11_SetPinMode(uint32_t mode) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_GPIO_PIN;
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pull = (mode == GPIO_MODE_INPUT) ? GPIO_PULLUP : GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);
}

// 发送起始信号（不在临界区内，20ms太长）
static void DHT11_Start(void) {
    DHT11_SetPinMode(GPIO_MODE_OUTPUT_PP);
    DHT11_PIN_LOW();
    delay_ms(20);  // 拉低20ms
    DHT11_PIN_HIGH();
    delay_us(30);  // 拉高30us
    DHT11_SetPinMode(GPIO_MODE_INPUT);
}

// 等待DHT11响应（需要在临界区内调用）
static uint8_t DHT11_CheckResponse(void) {
    uint16_t timeout = 0;

    // 等待DHT11拉低（80us）
    while (DHT11_PIN_READ() && timeout < 100) {
        delay_us(1);
        timeout++;
    }
    if (timeout >= 100) return 1;

    timeout = 0;
    // 等待DHT11拉高（80us）
    while (!DHT11_PIN_READ() && timeout < 100) {
        delay_us(1);
        timeout++;
    }
    if (timeout >= 100) return 1;

    return 0;
}

// 读取一个位（需要在临界区内调用）
static uint8_t DHT11_ReadBit(void) {
    uint16_t timeout = 0;

    // 等待高电平结束
    while (DHT11_PIN_READ() && timeout < 100) {
        delay_us(1);
        timeout++;
    }

    timeout = 0;
    // 等待低电平结束
    while (!DHT11_PIN_READ() && timeout < 100) {
        delay_us(1);
        timeout++;
    }

    delay_us(40);  // 延时40us采样

    return (DHT11_PIN_READ() == GPIO_PIN_SET) ? 1 : 0;
}

// 读取一个字节（需要在临界区内调用）
static uint8_t DHT11_ReadByte(void) {
    uint8_t byte = 0;

    for (uint8_t i = 0; i < 8; i++) {
        byte <<= 1;
        byte |= DHT11_ReadBit();
    }

    return byte;
}

// 初始化DHT11
void DHT11_Init(void) {
    DHT11_GPIO_CLK_ENABLE();
    DHT11_SetPinMode(GPIO_MODE_OUTPUT_PP);
    DHT11_PIN_HIGH();
}

// 读取DHT11温湿度（带临界区保护）
uint8_t DHT11_Read(uint8_t *temp, uint8_t *humi) {
    uint8_t data[5];

    // 发送起始信号（不禁用中断，20ms太长）
    DHT11_Start();

    // 进入临界区：禁用中断，保护时序敏感操作
    taskENTER_CRITICAL();

    // 检测DHT11响应
    if (DHT11_CheckResponse() != 0) {
        taskEXIT_CRITICAL();
        return 1;  // 无响应
    }

    // 读取40位数据（约4ms）
    for (uint8_t i = 0; i < 5; i++) {
        data[i] = DHT11_ReadByte();
    }

    // 退出临界区
    taskEXIT_CRITICAL();

    // 校验和验证（不需要临界区保护）
    uint8_t checksum = data[0] + data[1] + data[2] + data[3];
    if (checksum != data[4]) {
        return 1;  // 校验失败
    }

    // 数据范围验证
    if (!DHT11_IS_TEMP_VALID(data[2]) || !DHT11_IS_HUMI_VALID(data[0])) {
        return 1;  // 数据异常
    }

    *humi = data[0];
    *temp = data[2];

    return 0;  // 成功
}
