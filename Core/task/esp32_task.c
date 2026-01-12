//
// Created by Claude on 2026/1/5.
// ESP32 通信模块 FreeRTOS 任务实现
//

#include "esp32_task.h"
    

// ==================== 外部声明 ====================
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

// ==================== 私有变量 ====================
static TaskHandle_t esp32_task_handle = NULL;
static SemaphoreHandle_t esp32_tx_mutex = NULL;
static SemaphoreHandle_t esp32_tx_done = NULL;
static volatile uint8_t esp32_running = 0;

// DMA 缓冲区
uint8_t esp32_dma_rx_buffer[ESP32_DMA_BUFFER_SIZE];
static uint8_t esp32_tx_buffer[ESP32_TX_BUFFER_SIZE];

// 接收环形缓冲区
static uint8_t esp32_rx_buffer[ESP32_RX_BUFFER_SIZE];
static volatile uint16_t rx_write_pos = 0;
static volatile uint16_t rx_read_pos = 0;

// 解析缓冲区
static char parse_buffer[128];
static uint16_t parse_len = 0;

// ==================== 环形缓冲区操作 ====================
static uint16_t ringbuf_available(void) {
    return (rx_write_pos - rx_read_pos + ESP32_RX_BUFFER_SIZE) % ESP32_RX_BUFFER_SIZE;
}

static uint8_t ringbuf_read_byte(void) {
    uint8_t byte = esp32_rx_buffer[rx_read_pos];
    rx_read_pos = (rx_read_pos + 1) % ESP32_RX_BUFFER_SIZE;
    return byte;
}

// ==================== 命令解析 ====================
static void process_command(const char *cmd) {
    uart_printf_dma(&huart1, "[ESP32] RX: %s\r\n", cmd);

    // 心率命令
    if (strstr(cmd, CMD_HR_STOP)) {
        if (!max30102_is_running()) {
            esp32_send_status("ALREADY_OFF");
        } else {
            max30102_task_stop();
            esp32_send_status("OK");
        }
    }
    else if (strstr(cmd, CMD_HR_START)) {
        if (max30102_is_running()) {
            esp32_send_status("ALREADY_ON");
        } else {
            max30102_task_start();
            esp32_send_status("OK");
        }
    }
    // 温湿度命令
    else if (strstr(cmd, CMD_TEMP_STOP)) {
        if (!dht11_is_running()) {
            esp32_send_status("ALREADY_OFF");
        } else {
            dht11_task_stop();
            esp32_send_status("OK");
        }
    }
    else if (strstr(cmd, CMD_TEMP_START)) {
        if (dht11_is_running()) {
            esp32_send_status("ALREADY_ON");
        } else {
            dht11_task_start();
            esp32_send_status("OK");
        }
    }
    // MPU6050 命令
    else if (strstr(cmd, CMD_MPU_STOP)) {
        if (!mpu6050_is_running()) {
            esp32_send_status("ALREADY_OFF");
        } else {
            mpu6050_task_stop();
            esp32_send_status("OK");
        }
    }
    else if (strstr(cmd, CMD_MPU_START)) {
        if (mpu6050_is_running()) {
            esp32_send_status("ALREADY_ON");
        } else {
            mpu6050_task_start();
            esp32_send_status("OK");
        }
    }
    // GPS 命令
    else if (strstr(cmd, CMD_GPS_STOP)) {
        if (!gps_is_running()) {
            esp32_send_status("ALREADY_OFF");
        } else {
            gps_task_stop();
            esp32_send_status("OK");
        }
    }
    else if (strstr(cmd, CMD_GPS_START)) {
        if (gps_is_running()) {
            esp32_send_status("ALREADY_ON");
        } else {
            gps_task_start();
            esp32_send_status("OK");
        }
    }
    // MQ2 命令
    else if (strstr(cmd, CMD_MQ2_STOP)) {
        if (!mq2_is_running()) {
            esp32_send_status("ALREADY_OFF");
        } else {
            mq2_task_stop();
            esp32_send_status("OK");
        }
    }
    else if (strstr(cmd, CMD_MQ2_START)) {
        if (mq2_is_running()) {
            esp32_send_status("ALREADY_ON");
        } else {
            mq2_task_start();
            esp32_send_status("OK");
        }
    }
    else {
        esp32_send_status("ERROR");
        uart_printf_dma(&huart1, "[ESP32] Unknown cmd: %s\r\n", cmd);
    }
}

// 解析接收数据（查找 {xxx}\r\n 格式）
static void parse_rx_data(void) {
    while (ringbuf_available() > 0) {
        uint8_t byte = ringbuf_read_byte();

        if (byte == '{') {
            parse_len = 0;
            parse_buffer[parse_len++] = byte;
        }
        else if (parse_len > 0) {
            if (parse_len < sizeof(parse_buffer) - 1) {
                parse_buffer[parse_len++] = byte;
            }

            if (byte == '}' || byte == '\n') {
                parse_buffer[parse_len] = '\0';
                // 去除尾部 \r\n
                char *end = strchr(parse_buffer, '\r');
                if (end) *end = '\0';
                end = strchr(parse_buffer, '\n');
                if (end) *end = '\0';

                if (strlen(parse_buffer) > 2) {
                    process_command(parse_buffer);
                }
                parse_len = 0;
            }
        }
    }
}

// ==================== 任务主函数 ====================
static void esp32_task(void *arg) {
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(600));
    uart_printf_dma(&huart1, "[ESP32] Task Started\r\n");

    // 自动启动 DMA 接收
    esp32_running = 1;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, esp32_dma_rx_buffer, ESP32_DMA_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

    // 发送就绪消息
    esp32_send_status("STM32_READY");

    while (1) {
        // 解析接收数据
        parse_rx_data();

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ==================== UART 回调 ====================
void esp32_uart_rx_callback(uint8_t *data, uint16_t len) {
    // 带溢出保护的环形缓冲区写入
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next_pos = (rx_write_pos + 1) % ESP32_RX_BUFFER_SIZE;
        
        // 缓冲区满时丢弃新数据，避免覆盖未读数据
        if (next_pos == rx_read_pos) {
            break;
        }
        
        esp32_rx_buffer[rx_write_pos] = data[i];
        rx_write_pos = next_pos;
    }
}

void esp32_uart_tx_callback(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (esp32_tx_done) {
        xSemaphoreGiveFromISR(esp32_tx_done, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// ==================== DMA 发送函数 ====================
static int esp32_send_dma(const char *data, uint16_t len) {
    if (!esp32_running || len == 0 || len > ESP32_TX_BUFFER_SIZE) return 0;

    xSemaphoreTake(esp32_tx_mutex, portMAX_DELAY);

    // 等待上次发送完成
    uint32_t timeout = 0;
    while (huart3.gState != HAL_UART_STATE_READY) {
        vTaskDelay(1);
        if (++timeout > 100) break;
    }

    // 清空残留信号量
    xSemaphoreTake(esp32_tx_done, 0);

    // 复制数据到 DMA 缓冲区
    memcpy(esp32_tx_buffer, data, len);

    // 启动 DMA 发送
    if (HAL_UART_Transmit_DMA(&huart3, esp32_tx_buffer, len) != HAL_OK) {
        xSemaphoreGive(esp32_tx_mutex);
        return 0;
    }

    // 等待发送完成
    xSemaphoreTake(esp32_tx_done, pdMS_TO_TICKS(500));

    xSemaphoreGive(esp32_tx_mutex);
    return len;
}

// ==================== 公共发送接口 ====================
void esp32_send_heartrate(uint8_t hr, uint8_t spo2) {
    char buf[48];
    int len = snprintf(buf, sizeof(buf), "{DATA:HR:%d,SPO2:%d}\r\n", hr, spo2);
    esp32_send_dma(buf, len);
}

void esp32_send_temperature(float temp, float humi) {
    char buf[48];
    // ESP32 解析格式: {DATA:TEMP:%d,%d}
    int len = snprintf(buf, sizeof(buf), "{DATA:TEMP:%d,%d}\r\n", (int)temp, (int)humi);
    esp32_send_dma(buf, len);
}

void esp32_send_mpu6050(float pitch, float roll, float yaw) {
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "{DATA:MPU:%.1f,%.1f,%.1f}\r\n", pitch, roll, yaw);
    esp32_send_dma(buf, len);
}

void esp32_send_gps(float lat, float lon, uint8_t satellites) {
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "{DATA:GPS:%.6f,%.6f,%d}\r\n", lat, lon, satellites);
    esp32_send_dma(buf, len);
}

void esp32_send_mq2(uint16_t adc_value, uint8_t alarm) {
    char buf[48];
    // ESP32 解析格式: {DATA:MQ2:%f,%d}
    int len = snprintf(buf, sizeof(buf), "{DATA:MQ2:%.1f,%d}\r\n", (float)adc_value, alarm);
    esp32_send_dma(buf, len);
}

void esp32_send_status(const char *status) {
    char buf[32];
    int len = snprintf(buf, sizeof(buf), "{STATUS:%s}\r\n", status);
    esp32_send_dma(buf, len);
}

void esp32_send_event(const char *event) {
    char buf[32];
    int len = snprintf(buf, sizeof(buf), "{EVENT:%s}\r\n", event);
    esp32_send_dma(buf, len);
}

void esp32_send_sensor_state(const char *sensor, uint8_t on) {
    char buf[32];
    int len = snprintf(buf, sizeof(buf), "{SENSOR:%s:%s}\r\n", sensor, on ? "ON" : "OFF");
    esp32_send_dma(buf, len);
}

// ==================== 公共接口 ====================
void esp32_task_init(void) {
    // 创建同步对象
    esp32_tx_mutex = xSemaphoreCreateMutex();
    esp32_tx_done = xSemaphoreCreateBinary();

    if (!esp32_tx_mutex || !esp32_tx_done) {
        uart_printf_dma(&huart1, "[ERROR] ESP32 semaphore failed!\r\n");
        return;
    }

    // 创建任务
    BaseType_t ret = xTaskCreate(
        esp32_task,
        "esp32",
        512,  // 增大栈避免溢出
        NULL,
        2,
        &esp32_task_handle
    );

    if (ret != pdPASS) {
        uart_printf_dma(&huart1, "[ERROR] ESP32 task create failed!\r\n");
        return;
    }

    uart_printf_dma(&huart1, "[ESP32] Init OK\r\n");
}

void esp32_task_start(void) {
    if (esp32_task_handle && !esp32_running) {
        esp32_running = 1;
        rx_write_pos = 0;
        rx_read_pos = 0;
        parse_len = 0;

        // 启动 USART3 DMA 接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, esp32_dma_rx_buffer, ESP32_DMA_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

        vTaskResume(esp32_task_handle);
        uart_printf_dma(&huart1, "[ESP32] Task Started, DMA RX/TX Enabled\r\n");

        // 发送就绪消息
        esp32_send_status("STM32_READY");
    }
}

void esp32_task_stop(void) {
    if (esp32_task_handle && esp32_running) {
        esp32_running = 0;
        HAL_UART_DMAStop(&huart3);
        uart_printf_dma(&huart1, "[ESP32] Task Stopped\r\n");
    }
}

uint8_t esp32_is_running(void) {
    return esp32_running;
}
