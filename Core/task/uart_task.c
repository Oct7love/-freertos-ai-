//
// Created by 13615 on 2025/12/30.
//
// FreeRTOS串口DMA驱动（非阻塞收发）

#include "uart_task.h"

// ==================== 数据结构 ====================
static ringbuffer_t uart_rx_rb;              // 接收环形缓冲区（2KB）
uint8_t uart_rx_dma_buffer[128];             // DMA硬件接收缓冲区
static uint8_t uart_tx_dma_buffer[256];      // DMA发送缓冲区

// ==================== IPC对象 ====================
static SemaphoreHandle_t uart_rx_semaphore;  // 接收数据通知信号量
static SemaphoreHandle_t uart_tx_mutex;      // 发送互斥锁（多任务保护）
static SemaphoreHandle_t uart_tx_done;       // DMA发送完成信号量

// ==================== UTF-8 辅助函数 ====================
/**
 * @brief 查找 UTF-8 字符边界，确保不截断多字节字符
 * @param data 数据缓冲区
 * @param len 缓冲区长度
 * @return 最后一个完整 UTF-8 字符的结束位置（可安全截断的位置）
 */
static uint32_t find_utf8_boundary(uint8_t *data, uint32_t len) {
    if (len == 0) return 0;

    // 从末尾向前扫描，最多检查 4 个字节（UTF-8 最长 4 字节）
    for (int i = len - 1; i >= 0 && i >= (int)len - 4; i--) {
        uint8_t byte = data[i];

        // 单字节字符 (0xxxxxxx)
        if ((byte & 0x80) == 0x00) {
            return i + 1;
        }

        // 多字节字符首字节
        if ((byte & 0xE0) == 0xC0) {  // 110xxxxx - 2字节字符
            if (len - i >= 2) return len;  // 完整
            return i;  // 不完整，截断到首字节前
        }
        if ((byte & 0xF0) == 0xE0) {  // 1110xxxx - 3字节字符
            if (len - i >= 3) return len;  // 完整
            return i;  // 不完整
        }
        if ((byte & 0xF8) == 0xF0) {  // 11110xxx - 4字节字符
            if (len - i >= 4) return len;  // 完整
            return i;  // 不完整
        }

        // 后续字节 (10xxxxxx) - 继续向前扫描
    }

    // 未找到有效首字节，返回原长度（可能全是 ASCII 或数据异常）
    return len;
}

// ==================== 接收任务 ====================
/**
 * @brief 串口接收任务（阻塞在信号量，不占CPU）
 * @note  从RingBuffer读取数据并回显（按行缓冲 + UTF-8 边界检测）
 */
static void uart_rx_task(void *arg)
{
    static uint8_t line_buffer[512];      // 行缓冲区
    static uint32_t line_len = 0;         // 当前行长度
    static uint8_t incomplete_buffer[4];  // 保存不完整的 UTF-8 字节
    static uint32_t incomplete_len = 0;
    uint8_t data_buffer[256];

    uart_printf_dma(&huart1, "[RX Task] Running!\r\n");

    for (;;)
    {
        if (xSemaphoreTake(uart_rx_semaphore, portMAX_DELAY) == pdTRUE) {
            while (!ringbuffer_is_empty(&uart_rx_rb)) {
                uint32_t offset = 0;

                // 先复制上次不完整的字节
                if (incomplete_len > 0) {
                    memcpy(data_buffer, incomplete_buffer, incomplete_len);
                    offset = incomplete_len;
                    incomplete_len = 0;
                }

                // 计算可读取的数据量
                uint32_t available = (uart_rx_rb.w - uart_rx_rb.r + RINGBUFFER_SIZE) % RINGBUFFER_SIZE;
                uint32_t read_size = sizeof(data_buffer) - offset;
                if (available < read_size) {
                    read_size = available;
                }

                // 从 ringbuffer 读取数据
                if (ringbuffer_read(&uart_rx_rb, data_buffer + offset, read_size) != 0) {
                    break;
                }

                uint32_t total_len = offset + read_size;

                // UTF-8 边界检测
                uint32_t safe_len = find_utf8_boundary(data_buffer, total_len);

                // 保存不完整的字节到下次处理
                if (safe_len < total_len) {
                    incomplete_len = total_len - safe_len;
                    memcpy(incomplete_buffer, data_buffer + safe_len, incomplete_len);
                }

                // 将安全的数据逐字节处理（按行缓冲）
                for (uint32_t i = 0; i < safe_len; i++) {
                    uint8_t byte = data_buffer[i];

                    if (byte == '\r' || byte == '\n') {
                        if (line_len > 0) {
                            line_buffer[line_len] = '\0';
                            uart_printf_dma(&huart1, "[RX] %s\r\n", (char*)line_buffer);

                            // if (strstr((char*)line_buffer, "error")) {
                            //     led_set_mode(LED_EVENT_ERROR);
                            // } else if (strstr((char*)line_buffer, "normal")) {
                            //     led_set_mode(LED_EVENT_HEARTBEAT);
                            // }

                            line_len = 0;
                        }
                    } else {
                        if (line_len < sizeof(line_buffer) - 1) {
                            line_buffer[line_len++] = byte;
                        }
                    }
                }
            }
        }
    }
}

// ==================== 中断回调 ====================
/**
 * @brief DMA发送完成中断回调（从usart.c的HAL_UART_TxCpltCallback调用）
 * @note  释放信号量通知发送完成
 */
void uart_tx_complete_callback(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (uart_tx_done) {
        xSemaphoreGiveFromISR(uart_tx_done, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief UART空闲中断回调（从usart.c的HAL_UARTEx_RxEventCallback调用）
 * @param data 接收到的数据指针
 * @param size 数据长度
 * @note  写入RingBuffer并释放信号量通知任务
 */
void uart_rx_event_callback(uint8_t *data, uint16_t size) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // 写入 RingBuffer，检查是否成功
    if (ringbuffer_write(&uart_rx_rb, data, size) != 0) {
        // RingBuffer 满了，数据丢失！（可选：添加错误计数）
        // 注意：不要在 ISR 中调用 printf，这里只是标记
    }

    xSemaphoreGiveFromISR(uart_rx_semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ==================== DMA发送函数 ====================
/**
 * @brief 格式化DMA发送（支持调度器启动前后使用）
 * @param huart UART句柄
 * @param format 格式化字符串
 * @return 发送字节数
 * @note  调度器未启动时使用轮询等待，启动后使用信号量等待（不占CPU）
 */
int uart_printf_dma(UART_HandleTypeDef *huart, const char *format, ...) {
    va_list arg;
    int len;
    uint32_t timeout;

    // 格式化字符串
    va_start(arg, format);
    len = vsnprintf((char *)uart_tx_dma_buffer, sizeof(uart_tx_dma_buffer), format, arg);
    va_end(arg);
    if (len <= 0) return 0;

    // 检查调度器状态
    BaseType_t os_running = (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING);

    // 多任务保护：获取互斥锁
    if (os_running && uart_tx_mutex) {
        xSemaphoreTake(uart_tx_mutex, portMAX_DELAY);
    }

    // 等待上次DMA完成（带超时保护）
    timeout = 0;
    while (huart->gState != HAL_UART_STATE_READY) {
        if (os_running) {
            vTaskDelay(1);
            if (++timeout > 1000) break;  // 1秒超时
        } else {
            if (++timeout > 100000) break;  // 轮询超时
        }
    }

    // 清空残留信号量
    if (os_running && uart_tx_done) {
        xSemaphoreTake(uart_tx_done, 0);
    }

    // 启动DMA发送
    if (HAL_UART_Transmit_DMA(huart, uart_tx_dma_buffer, len) != HAL_OK) {
        if (os_running && uart_tx_mutex) {
            xSemaphoreGive(uart_tx_mutex);
        }
        return 0;
    }

    // 等待DMA完成
    if (os_running && uart_tx_done) {
        // 调度器运行：信号量等待（不占CPU）
        xSemaphoreTake(uart_tx_done, pdMS_TO_TICKS(1000));
    } else {
        // 调度器未启动：轮询等待（带超时保护）
        timeout = 0;
        while (huart->gState != HAL_UART_STATE_READY) {
            if (++timeout > 100000) break;
        }
    }

    // 释放互斥锁
    if (os_running && uart_tx_mutex) {
        xSemaphoreGive(uart_tx_mutex);
    }

    return len;
}

// ==================== 初始化函数 ====================
/**
 * @brief 初始化UART DMA收发任务
 * @note  必须在FreeRTOS调度器启动前调用
 */
void uart_task_init(void) {
    // 初始化RingBuffer
    ringbuffer_init(&uart_rx_rb);

    // 创建同步对象
    uart_rx_semaphore = xSemaphoreCreateBinary();
    uart_tx_done = xSemaphoreCreateBinary();
    uart_tx_mutex = xSemaphoreCreateMutex();

    if (!uart_rx_semaphore || !uart_tx_done || !uart_tx_mutex) {
        for(;;);  // 创建失败，死循环等待看门狗复位
    }

    // 创建接收任务（优先级3，栈512字，增大栈以容纳512字节缓冲区）
    if (xTaskCreate(uart_rx_task, "uart_rx", 512, NULL, 3, NULL) != pdPASS) {
        for(;;);  // 任务创建失败
    }

    // 启动DMA循环接收（空闲中断触发回调）
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_dma_buffer, sizeof(uart_rx_dma_buffer));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);  // 禁用半满中断

    // 发送初始化完成消息（调度器未启动，使用轮询等待）
    uart_printf_dma(&huart1, "\r\n[UART] DMA TX/RX Init OK!\r\n");
}
