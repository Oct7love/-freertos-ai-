//
// Created by 13615 on 2026/1/3.
// MQ2 FreeRTOS 任务实现
//

#include "mq2_task.h"

// ==================== 共享数据（Mutex保护） ====================
static struct {
    float ppm;
    MQ2_AlarmLevel_t alarm;
    uint8_t is_valid;
} mq2_shared_data = {0.0f, MQ2_ALARM_SAFE, 0};

// ==================== IPC对象 ====================
static SemaphoreHandle_t mq2_mutex = NULL;
static TaskHandle_t mq2_task_handle = NULL;
static volatile uint8_t mq2_running = 0;

// ==================== MQ2任务 ====================
static void mq2_task(void *arg) {
    MQ2_Data_t data;
    uint32_t read_count = 0;

    // 错峰启动
    vTaskDelay(pdMS_TO_TICKS(300));

    // 初始化MQ2硬件（启动ADC DMA）
    MQ2_Init();
    vTaskDelay(pdMS_TO_TICKS(500));  // ADC稳定
    uart_printf_dma(&huart1, "[MQ2] Hardware init OK, waiting for start...\r\n");

    for (;;) {
        // 检查运行状态
        if (!mq2_running) {
            vTaskSuspend(NULL);
            uart_printf_dma(&huart1, "[MQ2] Task resumed!\r\n");
            continue;
        }

        // 更新MQ2数据
        MQ2_Update(&data);
        read_count++;

        // 更新共享数据（Mutex保护）
        xSemaphoreTake(mq2_mutex, portMAX_DELAY);
        mq2_shared_data.ppm = data.ppm;
        mq2_shared_data.alarm = data.alarm;
        mq2_shared_data.is_valid = 1;
        xSemaphoreGive(mq2_mutex);

        // 串口打印
        uart_printf_dma(&huart1, "[MQ2] PPM=%.1f Alarm=%d [OK:%lu]\r\n",
                       data.ppm, data.alarm, read_count);

        // 通知OLED更新MQ2显示
        oled_update_mq2(data.ppm, data.alarm, 1);

        // 报警处理：危险时LED快闪
        if (data.alarm >= MQ2_ALARM_HIGH) {
            led_set_mode(LED_EVENT_ERROR);
        }

        // 延时1秒（释放CPU）
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ==================== 初始化 ====================
void mq2_task_init(void) {
    // 创建互斥锁
    mq2_mutex = xSemaphoreCreateMutex();
    if (!mq2_mutex) {
        uart_printf_dma(&huart1, "[ERROR] MQ2 mutex failed!\r\n");
        for(;;);
    }

    // 默认停止状态
    mq2_running = 0;

    // 创建MQ2任务（优先级2，栈384字节：浮点运算需要更大栈）
    if (xTaskCreate(mq2_task, "mq2", 384, NULL, 2, &mq2_task_handle) != pdPASS) {
        uart_printf_dma(&huart1, "[ERROR] MQ2 task failed!\r\n");
        for(;;);
    }

    uart_printf_dma(&huart1, "[MQ2] Task created (suspended)!\r\n");
}

// ==================== 数据获取（线程安全） ====================
void mq2_get_data(float *ppm, MQ2_AlarmLevel_t *alarm, uint8_t *valid) {
    if (!mq2_mutex || !ppm || !alarm || !valid) return;

    xSemaphoreTake(mq2_mutex, portMAX_DELAY);
    *ppm = mq2_shared_data.ppm;
    *alarm = mq2_shared_data.alarm;
    *valid = mq2_shared_data.is_valid;
    xSemaphoreGive(mq2_mutex);
}

// ==================== 任务控制 ====================
void mq2_task_start(void) {
    if (mq2_task_handle && !mq2_running) {
        mq2_running = 1;
        vTaskResume(mq2_task_handle);
        uart_printf_dma(&huart1, "[MQ2] Started!\r\n");
    }
}

void mq2_task_stop(void) {
    if (mq2_running) {
        mq2_running = 0;
        uart_printf_dma(&huart1, "[MQ2] Stopped!\r\n");
    }
}

uint8_t mq2_is_running(void) {
    return mq2_running;
}
