//
// Created by 13615 on 2026/1/3.
//

#include "dht11_task.h"
#include "oled_task.h"
#include "esp32_task.h"

// 共享数据（互斥锁保护）
static struct {
    uint8_t temperature;
    uint8_t humidity;
    uint8_t is_valid;
} dht11_shared_data = {0, 0, 0};

// IPC对象
static SemaphoreHandle_t dht11_mutex = NULL;
static TaskHandle_t dht11_task_handle = NULL;  // 任务句柄
static volatile uint8_t dht11_running = 0;     // 运行状态标志

// DHT11任务
static void dht11_task(void *arg) {
    uint8_t temp = 0, humi = 0;
    uint32_t read_count = 0;
    uint32_t error_count = 0;

    // 错峰启动
    vTaskDelay(pdMS_TO_TICKS(250));

    // 初始化DHT11硬件
    DHT11_Init();
    vTaskDelay(pdMS_TO_TICKS(1000));
    uart_printf_dma(&huart1, "[DHT11] Hardware init OK, waiting for start...\r\n");

    for (;;) {
        // 检查是否处于运行状态
        if (!dht11_running) {
            // 挂起自己，等待被唤醒
            vTaskSuspend(NULL);
            // 被唤醒后继续
            uart_printf_dma(&huart1, "[DHT11] Task resumed!\r\n");
            continue;
        }

        // 读取DHT11（阻塞约18ms）
        if (DHT11_Read(&temp, &humi) == 0) {
            read_count++;

            // 更新共享数据（互斥锁保护）
            xSemaphoreTake(dht11_mutex, portMAX_DELAY);
            dht11_shared_data.temperature = temp;
            dht11_shared_data.humidity = humi;
            dht11_shared_data.is_valid = 1;
            xSemaphoreGive(dht11_mutex);

            // 串口打印
            uart_printf_dma(&huart1, "[DHT11] T=%d C H=%d%% [OK:%lu]\r\n",
                           temp, humi, read_count);

            // 通知OLED更新温湿度显示
            oled_update_dht11(temp, humi, 1);

            // 发送数据给ESP32
            esp32_send_temperature((float)temp, (float)humi);

        } else {
            error_count++;
            uart_printf_dma(&huart1, "[DHT11] Read failed! [ERR:%lu]\r\n", error_count);

            xSemaphoreTake(dht11_mutex, portMAX_DELAY);
            dht11_shared_data.is_valid = 0;
            xSemaphoreGive(dht11_mutex);

            // 通知OLED数据无效
            oled_update_dht11(0, 0, 0);
        }

        // 延时2秒（释放CPU）
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// 初始化DHT11任务（默认挂起状态）
void dht11_task_init(void) {
    // 创建互斥锁
    dht11_mutex = xSemaphoreCreateMutex();
    if (!dht11_mutex) {
        uart_printf_dma(&huart1, "[ERROR] DHT11 mutex failed!\r\n");
        for(;;);
    }

    // 默认停止状态
    dht11_running = 0;

    // 创建DHT11任务（优先级2，栈256字节），保存句柄
    if (xTaskCreate(dht11_task, "dht11", 256, NULL, 2, &dht11_task_handle) != pdPASS) {
        uart_printf_dma(&huart1, "[ERROR] DHT11 task failed!\r\n");
        for(;;);
    }

    uart_printf_dma(&huart1, "[DHT11] Task created (suspended)!\r\n");
}

// 获取温湿度数据（线程安全）
void dht11_get_data(uint8_t *temp, uint8_t *humi, uint8_t *valid) {
    if (!dht11_mutex || !temp || !humi || !valid) return;

    xSemaphoreTake(dht11_mutex, portMAX_DELAY);
    *temp = dht11_shared_data.temperature;
    *humi = dht11_shared_data.humidity;
    *valid = dht11_shared_data.is_valid;
    xSemaphoreGive(dht11_mutex);
}

// 启动DHT11采集
void dht11_task_start(void) {
    if (dht11_task_handle && !dht11_running) {
        dht11_running = 1;
        vTaskResume(dht11_task_handle);
        uart_printf_dma(&huart1, "[DHT11] Started!\r\n");
    }
}

// 停止DHT11采集
void dht11_task_stop(void) {
    if (dht11_running) {
        dht11_running = 0;
        uart_printf_dma(&huart1, "[DHT11] Stopped!\r\n");
    }
}

// 查询运行状态
uint8_t dht11_is_running(void) {
    return dht11_running;
}