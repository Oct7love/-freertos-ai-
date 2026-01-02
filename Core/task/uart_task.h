//
// Created by 13615 on 2025/12/30.
//
// FreeRTOS串口DMA非阻塞收发驱动头文件

#ifndef FREERTOS_TEST_UART_TASK_H
#define FREERTOS_TEST_UART_TASK_H

#include "APP/bsp_system.h"

/**
 * @brief 初始化UART DMA收发任务
 * @note  必须在调度器启动前调用（MX_FREERTOS_Init中）
 * @note  会创建接收任务、信号量并启动DMA接收
 */
void uart_task_init(void);

/**
 * @brief 格式化DMA发送函数
 * @param huart UART句柄（如 &huart1）
 * @param format printf风格的格式化字符串
 * @param ... 可变参数
 * @return 发送的字节数
 * @note  支持在调度器启动前后使用
 * @note  调度器运行时使用信号量等待（不占CPU）
 * @note  多任务安全（内置互斥锁保护）
 */
int uart_printf_dma(UART_HandleTypeDef *huart, const char *format, ...);

/**
 * @brief UART接收中断回调（由usart.c中的HAL回调调用）
 * @param data 接收到的数据指针
 * @param size 数据长度
 * @note  写入RingBuffer并发送信号量通知任务
 */
void uart_rx_event_callback(uint8_t *data, uint16_t size);

/**
 * @brief DMA发送完成中断回调（由usart.c中的HAL回调调用）
 * @note  释放信号量通知发送完成
 */
void uart_tx_complete_callback(void);

/**
 * @brief DMA接收缓冲区（硬件循环写入）
 */
extern uint8_t uart_rx_dma_buffer[128];

#endif //FREERTOS_TEST_UART_TASK_H
