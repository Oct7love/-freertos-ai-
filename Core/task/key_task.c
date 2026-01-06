//
// Created by 13615 on 2026/1/2.
//

#include "key_task.h"
#include "oled_task.h"
#include "dht11_task.h"
#include "mq2_task.h"
#include "mpu6050_task.h"
#include "max30102_task.h"
#include "gps_task.h"
#include "esp32_task.h"

// IPC对象
static EventGroupHandle_t key_event_group = NULL;
static TimerHandle_t key1_debounce_timer = NULL;
static TimerHandle_t key2_debounce_timer = NULL;

// 去抖时间
#define KEY_DEBOUNCE_MS  30

// 软件定时器回调：KEY1去抖确认
static void key1_debounce_callback(TimerHandle_t xTimer) {
    (void)xTimer;
    // 再次读取按键状态，确认是否仍然按下
    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
        // 确认按下，设置事件位
        xEventGroupSetBits(key_event_group, KEY_EVENT_KEY1_PRESS);
    }
}

// 软件定时器回调：KEY2去抖确认
static void key2_debounce_callback(TimerHandle_t xTimer) {
    (void)xTimer;

    if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {
        xEventGroupSetBits(key_event_group, KEY_EVENT_KEY2_PRESS);
    }
}
// GPIO中断回调（需要在stm32f1xx_it.c中调用）
void key_exti_callback(uint16_t GPIO_Pin) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (GPIO_Pin == KEY1_Pin) {
        // 启动KEY1去抖定时器（30ms单次）
        if (key1_debounce_timer) {
            xTimerStartFromISR(key1_debounce_timer, &xHigherPriorityTaskWoken);
        }
    } else if (GPIO_Pin == KEY2_Pin) {
        // 启动KEY2去抖定时器（30ms单次）
        if (key2_debounce_timer) {
            xTimerStartFromISR(key2_debounce_timer, &xHigherPriorityTaskWoken);
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
// 按键处理任务
static void key_task(void *arg) {
    EventBits_t events;

    // 错峰启动
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_printf_dma(&huart1, "[KEY Task] Running!\r\n");

    for (;;) {
        // 阻塞等待按键事件
        events = xEventGroupWaitBits(
            key_event_group,
            KEY_EVENT_KEY1_PRESS | KEY_EVENT_KEY2_PRESS,
            pdTRUE,   // 清除事件位
            pdFALSE,  // 任意事件触发
            portMAX_DELAY
        );

        // KEY1：根据当前界面启停对应传感器
        if (events & KEY_EVENT_KEY1_PRESS) {
            uart_printf_dma(&huart1, "[KEY] KEY1 Pressed!\r\n");

            // 获取当前界面（原子读取，线程安全）
            ui_page_t page = oled_get_current_page();

            if (page == UI_PAGE_DHT11) {
                // DHT11 界面：只控制 DHT11
                if (dht11_is_running()) {
                    dht11_task_stop();
                    esp32_send_sensor_state("TEMP", 0);
                } else {
                    dht11_task_start();
                    esp32_send_sensor_state("TEMP", 1);
                }
                uart_printf_dma(&huart1, "[KEY] DHT11 %s\r\n",
                               dht11_is_running() ? "started" : "stopped");
            }
            else if (page == UI_PAGE_MQ2) {
                // MQ2 界面：只控制 MQ2
                if (mq2_is_running()) {
                    mq2_task_stop();
                    esp32_send_sensor_state("MQ2", 0);
                } else {
                    mq2_task_start();
                    esp32_send_sensor_state("MQ2", 1);
                }
                uart_printf_dma(&huart1, "[KEY] MQ2 %s\r\n",
                               mq2_is_running() ? "started" : "stopped");
            }
            else if (page == UI_PAGE_MPU6050) {
                // MPU6050 界面：只控制 MPU6050
                if (mpu6050_is_running()) {
                    mpu6050_task_stop();
                    esp32_send_sensor_state("MPU", 0);
                } else {
                    mpu6050_task_start();
                    esp32_send_sensor_state("MPU", 1);
                }
                uart_printf_dma(&huart1, "[KEY] MPU6050 %s\r\n",
                               mpu6050_is_running() ? "started" : "stopped");
            }
            else if (page == UI_PAGE_MAX30102) {
                // MAX30102 界面：只控制 MAX30102
                if (max30102_is_running()) {
                    max30102_task_stop();
                    esp32_send_sensor_state("HR", 0);
                } else {
                    max30102_task_start();
                    esp32_send_sensor_state("HR", 1);
                }
                uart_printf_dma(&huart1, "[KEY] MAX30102 %s\r\n",
                               max30102_is_running() ? "started" : "stopped");
            }
            else if (page == UI_PAGE_GPS) {
                // GPS 界面：只控制 GPS
                if (gps_is_running()) {
                    gps_task_stop();
                    esp32_send_sensor_state("GPS", 0);
                } else {
                    gps_task_start();
                    esp32_send_sensor_state("GPS", 1);
                }
                uart_printf_dma(&huart1, "[KEY] GPS %s\r\n",
                               gps_is_running() ? "started" : "stopped");
            }
            else {
                // 欢迎界面：提示用户先切换界面
                uart_printf_dma(&huart1, "[KEY] Switch to sensor page first!\r\n");
            }

            // 刷新当前界面（更新状态显示）
            oled_switch_page(page);
        }

        // KEY2：切换界面
        if (events & KEY_EVENT_KEY2_PRESS) {
            uart_printf_dma(&huart1, "[KEY] KEY2 Pressed!\r\n");
            oled_next_page();
            uart_printf_dma(&huart1, "[KEY] Page switched\r\n");
        }
    }
}
// 初始化按键任务
void key_task_init(void) {
    // 创建事件组
    key_event_group = xEventGroupCreate();
    if (!key_event_group) {
        uart_printf_dma(&huart1, "[ERROR] KEY event group failed!\r\n");
        for(;;);
    }

    // 创建KEY1去抖定时器（30ms单次触发）
    key1_debounce_timer = xTimerCreate(
        "key1_db",
        pdMS_TO_TICKS(KEY_DEBOUNCE_MS),
        pdFALSE,  // 单次触发
        NULL,
        key1_debounce_callback
    );
    if (!key1_debounce_timer) {
        uart_printf_dma(&huart1, "[ERROR] KEY1 timer failed!\r\n");
        for(;;);
    }

    // 创建KEY2去抖定时器
    key2_debounce_timer = xTimerCreate(
        "key2_db",
        pdMS_TO_TICKS(KEY_DEBOUNCE_MS),
        pdFALSE,
        NULL,
        key2_debounce_callback
    );
    if (!key2_debounce_timer) {
        uart_printf_dma(&huart1, "[ERROR] KEY2 timer failed!\r\n");
        for(;;);
    }

    // 创建按键任务（优先级3，栈256字节）
    if (xTaskCreate(key_task, "key", 256, NULL, 3, NULL) != pdPASS) {
        uart_printf_dma(&huart1, "[ERROR] KEY task creation failed!\r\n");
        for(;;);
    }

    HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    uart_printf_dma(&huart1, "[KEY] Init OK!\r\n");
}

// 获取按键事件组句柄
EventGroupHandle_t key_get_event_group(void) {
    return key_event_group;
}