//
// Created by 13615 on 2026/1/2.
//

#include "oled_task.h"
#include "dht11_task.h"
#include "mq2_task.h"

// IPC对象
static QueueHandle_t oled_queue = NULL;      // 显示命令队列
static SemaphoreHandle_t oled_mutex = NULL;  // 互斥锁保护I2C和缓冲区

// 界面状态
static volatile ui_page_t current_page = UI_PAGE_WELCOME;

// 温湿度缓存（用于界面刷新）
static uint8_t cached_temp = 0;
static uint8_t cached_humi = 0;
static uint8_t cached_valid = 0;

// MQ2缓存（用于界面刷新）
static float cached_ppm = 0.0f;
static uint8_t cached_alarm = 0;
static uint8_t cached_mq2_valid = 0;

// ==================== 界面绘制函数 ====================

// 绘制欢迎界面
static void draw_welcome_page(void) {
    OLED_Clear();
    OLED_ShowString(1, 3, "Smart Hat");
    OLED_ShowString(2, 2, "FreeRTOS V1");
    OLED_ShowString(3, 1, "----------------");
    OLED_ShowString(4, 1, "KEY1:Start");
}

// 绘制温湿度界面
static void draw_dht11_page(void) {
    OLED_Clear();
    OLED_ShowString(1, 1, "=DHT11 Monitor=");

    // 状态行
    if (dht11_is_running()) {
        OLED_ShowString(2, 1, "Status: RUN ");
    } else {
        OLED_ShowString(2, 1, "Status: STOP");
    }

    // 温湿度数据
    if (cached_valid) {
        OLED_ShowString(3, 1, "T:");
        OLED_ShowNum(3, 3, cached_temp, 2);
        OLED_ShowString(3, 5, "C  H:");
        OLED_ShowNum(3, 10, cached_humi, 2);
        OLED_ShowString(3, 12, "%");
    } else {
        OLED_ShowString(3, 1, "T:-- C  H:-- %");
    }

    // 提示行
    OLED_ShowString(4, 1, "K1:ON/OFF K2:--");
}

// 绘制MQ2气体浓度界面
static void draw_mq2_page(void) {
    OLED_Clear();
    OLED_ShowString(1, 1, "==MQ2 Monitor==");

    // 状态行
    if (mq2_is_running()) {
        OLED_ShowString(2, 1, "Status: RUN ");
    } else {
        OLED_ShowString(2, 1, "Status: STOP");
    }

    // PPM数据和报警等级
    if (cached_mq2_valid) {
        OLED_ShowString(3, 1, "PPM:");
        OLED_ShowFloat(3, 5, cached_ppm, 4, 1);
        OLED_ShowString(4, 1, "Alarm: Lv");
        OLED_ShowNum(4, 10, cached_alarm, 1);
    } else {
        OLED_ShowString(3, 1, "PPM: ----.-");
        OLED_ShowString(4, 1, "Alarm: --");
    }
}

// OLED任务主函数
static void oled_task(void *arg) {
    oled_msg_t msg;
    TickType_t last_flush = 0;

    // 错峰启动，避免打印冲突
    vTaskDelay(pdMS_TO_TICKS(150));
    uart_printf_dma(&huart1, "[OLED Task] Running!\r\n");

    // OLED硬件初始化
    uart_printf_dma(&huart1, "[OLED] Initializing...\r\n");
    OLED_Init();

    // 显示欢迎界面
    draw_welcome_page();
    OLED_Flush();
    uart_printf_dma(&huart1, "[OLED] Init OK! Welcome page shown.\r\n");

    for (;;) {
        // 1. 非阻塞接收显示命令（100ms超时）
        if (xQueueReceive(oled_queue, &msg, pdMS_TO_TICKS(100)) == pdPASS) {

            // 2. 获取互斥锁（保护缓冲区和I2C）
            if (xSemaphoreTake(oled_mutex, pdMS_TO_TICKS(200)) == pdPASS) {

                // 3. 根据命令类型操作
                switch (msg.cmd) {
                    case OLED_CMD_SHOW_TEXT:
                        OLED_ShowString(msg.line, msg.column, msg.data.text);
                        break;

                    case OLED_CMD_SHOW_NUM:
                        OLED_ShowNum(msg.line, msg.column,
                                     msg.data.num.value, msg.data.num.length);
                        break;

                    case OLED_CMD_SHOW_SIGNED:
                        OLED_ShowSignedNum(msg.line, msg.column,
                                          msg.data.signed_num.value,
                                          msg.data.signed_num.length);
                        break;

                    case OLED_CMD_SHOW_FLOAT:
                        OLED_ShowFloat(msg.line, msg.column,
                                      msg.data.flt.value,
                                      msg.data.flt.int_len,
                                      msg.data.flt.dec_len);
                        break;

                    case OLED_CMD_SHOW_HEX:
                        OLED_ShowHexNum(msg.line, msg.column,
                                       msg.data.hex.value, msg.data.hex.length);
                        break;

                    case OLED_CMD_CLEAR:
                        OLED_Clear();
                        break;

                    case OLED_CMD_FLUSH:
                        OLED_Flush();
                        last_flush = xTaskGetTickCount();
                        break;

                    case OLED_CMD_SWITCH_PAGE:
                        // 切换界面
                        current_page = msg.data.page;
                        if (current_page == UI_PAGE_WELCOME) {
                            draw_welcome_page();
                        } else if (current_page == UI_PAGE_DHT11) {
                            draw_dht11_page();
                        } else if (current_page == UI_PAGE_MQ2) {
                            draw_mq2_page();
                        }
                        OLED_Flush();
                        last_flush = xTaskGetTickCount();
                        uart_printf_dma(&huart1, "[OLED] Page switched to %d\r\n", current_page);
                        break;

                    case OLED_CMD_UPDATE_DHT11:
                        // 更新温湿度缓存
                        cached_temp = msg.data.dht11.temp;
                        cached_humi = msg.data.dht11.humi;
                        cached_valid = msg.data.dht11.valid;

                        // 如果当前在温湿度界面，刷新显示
                        if (current_page == UI_PAGE_DHT11) {
                            draw_dht11_page();
                        }
                        break;

                    case OLED_CMD_UPDATE_MQ2:
                        // 更新MQ2缓存
                        cached_ppm = msg.data.mq2.ppm;
                        cached_alarm = msg.data.mq2.alarm;
                        cached_mq2_valid = msg.data.mq2.valid;

                        // 如果当前在MQ2界面，刷新显示
                        if (current_page == UI_PAGE_MQ2) {
                            draw_mq2_page();
                        }
                        break;

                    default:
                        break;
                }

                // 4. 释放互斥锁
                xSemaphoreGive(oled_mutex);
            }
        }

        // 5. 定期自动刷新（每200ms刷新一次屏幕）
        if (xTaskGetTickCount() - last_flush > pdMS_TO_TICKS(200)) {
            if (xSemaphoreTake(oled_mutex, pdMS_TO_TICKS(50)) == pdPASS) {
                OLED_Flush();
                xSemaphoreGive(oled_mutex);
                last_flush = xTaskGetTickCount();
            }
        }
    }
}


// 初始化OLED任务
void oled_task_init(void) {
      // 创建消息队列（10条消息）
      oled_queue = xQueueCreate(10, sizeof(oled_msg_t));
      if (!oled_queue) {
          uart_printf_dma(&huart1, "[ERROR] OLED queue failed!\r\n");
          for(;;);
      }

      // 创建互斥锁
      oled_mutex = xSemaphoreCreateMutex();
      if (!oled_mutex) {
          uart_printf_dma(&huart1, "[ERROR] OLED mutex failed!\r\n");
          for(;;);
      }

      // 创建OLED任务（优先级2，栈256字节）
      if (xTaskCreate(oled_task, "oled", 256, NULL, 2, NULL) != pdPASS) {
          uart_printf_dma(&huart1, "[ERROR] OLED task creation failed!\r\n");
          for(;;);
      }

      uart_printf_dma(&huart1, "[OLED] Init scheduled!\r\n");
  }

  // ==================== 对外接口实现 ====================

  void oled_show_text(uint8_t line, uint8_t column, const char *text) {
      if (!oled_queue) return;

      oled_msg_t msg;
      msg.cmd = OLED_CMD_SHOW_TEXT;
      msg.line = line;
      msg.column = column;
      strncpy(msg.data.text, text, sizeof(msg.data.text) - 1);
      msg.data.text[sizeof(msg.data.text) - 1] = '\0';

      xQueueSend(oled_queue, &msg, 0);  // 不等待，立即返回
  }

  void oled_show_num(uint8_t line, uint8_t column, uint32_t num, uint8_t len) {
      if (!oled_queue) return;

      oled_msg_t msg;
      msg.cmd = OLED_CMD_SHOW_NUM;
      msg.line = line;
      msg.column = column;
      msg.data.num.value = num;
      msg.data.num.length = len;

      xQueueSend(oled_queue, &msg, 0);
  }

  void oled_show_signed_num(uint8_t line, uint8_t column, int32_t num, uint8_t len) {
      if (!oled_queue) return;

      oled_msg_t msg;
      msg.cmd = OLED_CMD_SHOW_SIGNED;
      msg.line = line;
      msg.column = column;
      msg.data.signed_num.value = num;
      msg.data.signed_num.length = len;

      xQueueSend(oled_queue, &msg, 0);
  }

  void oled_show_float(uint8_t line, uint8_t column, float num, uint8_t int_len, uint8_t dec_len) {
      if (!oled_queue) return;

      oled_msg_t msg;
      msg.cmd = OLED_CMD_SHOW_FLOAT;
      msg.line = line;
      msg.column = column;
      msg.data.flt.value = num;
      msg.data.flt.int_len = int_len;
      msg.data.flt.dec_len = dec_len;

      xQueueSend(oled_queue, &msg, 0);
  }

  void oled_show_hex(uint8_t line, uint8_t column, uint32_t num, uint8_t len) {
      if (!oled_queue) return;

      oled_msg_t msg;
      msg.cmd = OLED_CMD_SHOW_HEX;
      msg.line = line;
      msg.column = column;
      msg.data.hex.value = num;
      msg.data.hex.length = len;

      xQueueSend(oled_queue, &msg, 0);
  }

  void oled_clear_screen(void) {
      if (!oled_queue) return;

      oled_msg_t msg;
      msg.cmd = OLED_CMD_CLEAR;

      xQueueSend(oled_queue, &msg, 0);
  }

  void oled_flush_now(void) {
      if (!oled_queue) return;

      oled_msg_t msg;
      msg.cmd = OLED_CMD_FLUSH;

      xQueueSend(oled_queue, &msg, 0);
  }

// ==================== 界面控制接口 ====================

// 切换到指定界面
void oled_switch_page(ui_page_t page) {
    if (!oled_queue || page >= UI_PAGE_MAX) return;

    oled_msg_t msg;
    msg.cmd = OLED_CMD_SWITCH_PAGE;
    msg.data.page = page;

    xQueueSend(oled_queue, &msg, 0);
}

// 切换到下一个界面
void oled_next_page(void) {
    ui_page_t next = (current_page + 1) % UI_PAGE_MAX;
    oled_switch_page(next);
}

// 获取当前界面
ui_page_t oled_get_current_page(void) {
    return current_page;
}

// 更新温湿度显示
void oled_update_dht11(uint8_t temp, uint8_t humi, uint8_t valid) {
    if (!oled_queue) return;

    oled_msg_t msg;
    msg.cmd = OLED_CMD_UPDATE_DHT11;
    msg.data.dht11.temp = temp;
    msg.data.dht11.humi = humi;
    msg.data.dht11.valid = valid;

    xQueueSend(oled_queue, &msg, 0);
}

// 更新MQ2气体浓度显示
void oled_update_mq2(float ppm, uint8_t alarm, uint8_t valid) {
    if (!oled_queue) return;

    oled_msg_t msg;
    msg.cmd = OLED_CMD_UPDATE_MQ2;
    msg.data.mq2.ppm = ppm;
    msg.data.mq2.alarm = alarm;
    msg.data.mq2.valid = valid;

    xQueueSend(oled_queue, &msg, 0);
}