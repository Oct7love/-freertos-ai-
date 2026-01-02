//
// Created by 13615 on 2026/1/2.
//

#include "oled_task.h"
// IPC对象
static QueueHandle_t oled_queue = NULL;      // 显示命令队列
static SemaphoreHandle_t oled_mutex = NULL;  // 互斥锁保护I2C和缓冲区

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
      OLED_Clear();
      OLED_ShowString(1, 1, "FreeRTOS");
      OLED_ShowString(2, 1, "System Ready");
      OLED_Flush();
      uart_printf_dma(&huart1, "[OLED] Init OK!\r\n");

      for (;;) {
          // 1. 非阻塞接收显示命令（100ms超时）
          if (xQueueReceive(oled_queue, &msg, pdMS_TO_TICKS(100)) == pdPASS) {

              // 2. 获取互斥锁（保护缓冲区和I2C）
              if (xSemaphoreTake(oled_mutex, pdMS_TO_TICKS(200)) == pdPASS) {

                  // 3. 根据命令类型操作缓冲区
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
                          OLED_Flush();  // 立即刷新
                          last_flush = xTaskGetTickCount();
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
                  OLED_Flush();  // 耗时50-100ms，但不影响其他任务
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