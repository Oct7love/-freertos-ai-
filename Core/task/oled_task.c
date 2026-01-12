//
// Created by 13615 on 2026/1/2.
//

#include "oled_task.h"
#include "dht11_task.h"
#include "mq2_task.h"
#include "mpu6050_task.h"
#include "max30102_task.h"
#include "gps_task.h"
#include "driver/oled_graphics.h"
#include "driver/oled_icons.h"

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

// MPU6050缓存（用于界面刷新）
static float cached_pitch = 0.0f;
static float cached_roll = 0.0f;
static float cached_yaw = 0.0f;
static uint32_t cached_steps = 0;
static uint8_t cached_mpu_valid = 0;

// MAX30102缓存（用于界面刷新）
static uint16_t cached_heart_rate = 0;
static uint8_t cached_spo2 = 0;
static uint8_t cached_finger = 0;
static uint8_t cached_max30102_valid = 0;
static uint8_t heart_beat_state = 0;  // 心跳动画状态 (0=小, 1=大)

// GPS缓存（用于界面刷新）
static float cached_lat = 0.0f;
static float cached_lon = 0.0f;
static uint8_t cached_satellites = 0;
static uint8_t cached_gps_fixed = 0;

// ==================== 界面绘制函数 ====================

// 绘制欢迎界面
static void draw_welcome_page(void) {
    OLED_Clear();

    // 绘制边框
    OLED_DrawRect(0, 0, 127, 63);

    // 绘制顶部装饰线
    OLED_DrawHLine(0, 16, 127);

    // 绘制房屋图标
    OLED_DrawIcon8x8(2, 4, icon_home);

    // 绘制标题
    OLED_ShowString(1, 3, "Smart Hat");

    // 绘制版本号
    OLED_ShowString(2, 2, "FreeRTOS V1");

    // 绘制提示
    OLED_ShowString(4, 1, "KEY2:Switch");
}

// 绘制温湿度界面
static void draw_dht11_page(void) {
    OLED_Clear();

    // 绘制边框
    OLED_DrawRect(0, 0, 127, 63);

    // 绘制温度计图标
    OLED_DrawIcon8x8(2, 4, icon_temp);

    // 绘制标题
    OLED_ShowString(1, 3, "DHT11");

    // 右上角显示运行状态
    if (dht11_is_running()) {
        OLED_ShowString(1, 13, "RUN");
    } else {
        OLED_ShowString(1, 12, "STOP");
    }

    // 绘制分隔线
    OLED_DrawHLine(0, 16, 127);

    // 温湿度数据
    if (cached_valid) {
        // 温度值 + 进度条 (行2)
        OLED_ShowString(2, 1, "T:");
        OLED_ShowNum(2, 3, cached_temp, 2);
        OLED_ShowString(2, 5, "C");
        uint8_t temp_percent = (cached_temp > 50) ? 100 : cached_temp * 2;
        OLED_DrawRoundedProgressBar(50, 20, 70, 6, temp_percent, 100);

        // 湿度值 + 进度条 (行4)
        OLED_ShowString(4, 1, "H:");
        OLED_ShowNum(4, 3, cached_humi, 2);
        OLED_ShowString(4, 5, "%");
        OLED_DrawRoundedProgressBar(50, 52, 70, 6, cached_humi, 100);
    } else {
        OLED_ShowString(2, 1, "T: -- C");
        OLED_ShowString(4, 1, "H: -- %");
    }

    // 行3显示采集状态
    if (dht11_is_running()) {
        if (cached_valid) {
            OLED_ShowString(3, 1, "Sampling OK");
        } else {
            OLED_ShowString(3, 1, "Waiting...");
        }
    } else {
        OLED_ShowString(3, 1, "Stopped");
    }
}

// 绘制MQ2气体浓度界面
static void draw_mq2_page(void) {
    OLED_Clear();

    // 绘制边框
    OLED_DrawRect(0, 0, 127, 63);

    // 绘制气体图标
    OLED_DrawIcon8x8(2, 4, icon_gas);

    // 绘制标题
    OLED_ShowString(1, 3, "MQ2");

    // 绘制状态
    if (mq2_is_running()) {
        OLED_ShowString(1, 13, "RUN");
    } else {
        OLED_ShowString(1, 12, "STOP");
    }

    // 绘制分隔线
    OLED_DrawHLine(0, 16, 127);

    // PPM数据和报警等级
    if (cached_mq2_valid) {
        // 行2: PPM数值
        OLED_ShowString(2, 1, "PPM:");
        OLED_ShowFloat(2, 5, cached_ppm, 4, 1);

        // 行3: 报警等级 + 状态 (同一行显示)
        OLED_ShowString(3, 1, "Lv");
        OLED_ShowNum(3, 3, cached_alarm, 1);
        if (cached_alarm == 0) {
            OLED_ShowString(3, 5, "SAFE");
        } else if (cached_alarm == 1) {
            OLED_ShowString(3, 5, "LOW");
        } else if (cached_alarm == 2) {
            OLED_ShowString(3, 5, "MED!");
        } else {
            OLED_ShowString(3, 5, "HIGH!");
        }

        // 行4: 进度条 (Y=48, 高度6, 与PPM分开)
        uint16_t ppm_value = (cached_ppm > 5000) ? 5000 : (uint16_t)cached_ppm;
        OLED_DrawRoundedProgressBar(8, 52, 112, 6, ppm_value / 50, 100);
    } else {
        OLED_ShowString(2, 1, "PPM: ----.-");
        OLED_ShowString(3, 1, "Lv: --");
    }
}

// 绘制MPU6050姿态界面
static void draw_mpu6050_page(void) {
    OLED_Clear();

    // 绘制边框
    OLED_DrawRect(0, 0, 127, 63);

    // 绘制陀螺仪图标
    OLED_DrawIcon8x8(2, 4, icon_gyro);

    // 绘制标题
    OLED_ShowString(1, 3, "MPU6050");

    // 绘制状态
    if (mpu6050_is_running()) {
        OLED_ShowString(1, 13, "RUN");
    } else {
        OLED_ShowString(1, 12, "STOP");
    }

    // 绘制分隔线
    OLED_DrawHLine(0, 16, 127);

    // 姿态数据
    if (cached_mpu_valid) {
        // 第2行：P和R中间空一格
        // P:-99  R:-99
        OLED_ShowString(2, 1, "P:");
        OLED_ShowSignedNum(2, 3, (int32_t)cached_pitch, 3);
        OLED_ShowString(2, 8, "R:");
        OLED_ShowSignedNum(2, 10, (int32_t)cached_roll, 3);

        // 第3行：Y和S中间空一格
        // Y:-180  S:00000
        OLED_ShowString(3, 1, "Y:");
        OLED_ShowSignedNum(3, 3, (int32_t)cached_yaw, 4);
        OLED_ShowString(3, 9, "S:");
        OLED_ShowNum(3, 11, cached_steps, 5);

        // 第4行：Fall和Coll中间空一格
        // Fall:N  Coll:N
        uint8_t fall = mpu6050_is_fall_detected();
        uint8_t coll = mpu6050_is_collision_detected();
        OLED_ShowString(4, 1, "Fall:");
        OLED_ShowString(4, 6, fall ? "Y" : "N");
        OLED_ShowString(4, 9, "Coll:");
        OLED_ShowString(4, 14, coll ? "Y" : "N");
    } else {
        OLED_ShowString(2, 1, "P:---  R:---");
        OLED_ShowString(3, 1, "Y:----  S:-----");
        OLED_ShowString(4, 1, "Fall:-  Coll:-");
    }
}

// 绘制MAX30102心率血氧界面
static void draw_max30102_page(void) {
    OLED_Clear();

    // 绘制边框
    OLED_DrawRect(0, 0, 127, 63);

    // 绘制心跳图标
    OLED_DrawIcon8x8(2, 4, icon_heart);

    // 绘制标题
    OLED_ShowString(1, 3, "MAX30102");

    // 绘制状态
    if (max30102_is_running()) {
        OLED_ShowString(1, 13, "RUN");
    } else {
        OLED_ShowString(1, 12, "STOP");
    }

    // 绘制分隔线
    OLED_DrawHLine(0, 16, 127);

    // 心率血氧数据
    if (cached_max30102_valid && cached_finger) {
        // 心率
        OLED_ShowString(2, 1, "HR :");
        OLED_ShowNum(2, 5, cached_heart_rate, 3);
        OLED_ShowString(2, 8, "BPM");

        // 血氧
        OLED_ShowString(3, 1, "SpO2:");
        OLED_ShowNum(3, 6, cached_spo2, 3);
        OLED_ShowString(3, 9, "%");

        // 血氧进度条 (90-100%映射到0-100%)
        uint8_t spo2_percent = (cached_spo2 < 90) ? 0 : ((cached_spo2 - 90) * 10);
        OLED_DrawRoundedProgressBar(8, 52, 90, 6, spo2_percent, 100);

        // 右下角跳动爱心
        heart_beat_state = !heart_beat_state;  // 切换状态
        if (heart_beat_state) {
            // 大爱心
            OLED_DrawIcon8x8(115, 51, icon_heart_big);
        } else {
            // 小爱心 (偏移一点使其居中)
            for (uint8_t col = 0; col < 6; col++) {
                uint8_t column_data = icon_heart_small[col];
                for (uint8_t bit = 0; bit < 6; bit++) {
                    if (column_data & (1 << bit)) {
                        OLED_DrawPixel(116 + col, 53 + bit, 1);
                    }
                }
            }
        }
    } else if (!cached_finger) {
        OLED_ShowString(2, 1, " No Finger!");
        OLED_ShowString(3, 1, "Place finger");
    } else {
        OLED_ShowString(2, 1, "HR :--- BPM");
        OLED_ShowString(3, 1, "SpO2:--- %");
    }
}

// 绘制GPS定位界面
static void draw_gps_page(void) {
    OLED_Clear();

    // 绘制边框
    OLED_DrawRect(0, 0, 127, 63);

    // 绘制GPS图标
    OLED_DrawIcon8x8(2, 4, icon_gps);

    // 绘制标题
    OLED_ShowString(1, 3, "GPS");

    // 右上角显示运行状态
    if (gps_is_running()) {
        OLED_ShowString(1, 13, "RUN");
    } else {
        OLED_ShowString(1, 12, "STOP");
    }

    // 绘制分隔线
    OLED_DrawHLine(0, 16, 127);

    // GPS未运行时显示提示
    if (!gps_is_running()) {
        OLED_ShowString(2, 1, "GPS Stopped");
        OLED_ShowString(3, 1, "KEY1 to start");
        return;
    }

    // GPS数据
    if (cached_gps_fixed) {
        // 定位成功
        OLED_ShowString(2, 1, "FIXED Sat:");
        OLED_ShowNum(2, 11, cached_satellites, 2);

        // 纬度
        OLED_ShowString(3, 1, "La:");
        OLED_ShowFloat(3, 4, cached_lat, 2, 4);

        // 经度
        OLED_ShowString(4, 1, "Lo:");
        OLED_ShowFloat(4, 4, cached_lon, 3, 4);
    } else {
        // 搜索中
        OLED_ShowString(2, 1, "Searching...");

        // 卫星数
        OLED_ShowString(3, 1, "Satellites:");
        OLED_ShowNum(3, 12, cached_satellites, 2);

        // 信号强度条
        uint8_t sat_value = (cached_satellites > 32) ? 32 : cached_satellites;
        OLED_DrawRoundedProgressBar(8, 52, 112, 6, sat_value * 3, 100);
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
                        } else if (current_page == UI_PAGE_MPU6050) {
                            draw_mpu6050_page();
                        } else if (current_page == UI_PAGE_MAX30102) {
                            draw_max30102_page();
                        } else if (current_page == UI_PAGE_GPS) {
                            draw_gps_page();
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

                    case OLED_CMD_UPDATE_MPU6050:
                        // 更新MPU6050缓存
                        cached_pitch = msg.data.mpu6050.pitch;
                        cached_roll = msg.data.mpu6050.roll;
                        cached_yaw = msg.data.mpu6050.yaw;
                        cached_steps = msg.data.mpu6050.steps;
                        cached_mpu_valid = msg.data.mpu6050.valid;

                        // 如果当前在MPU6050界面，刷新显示
                        if (current_page == UI_PAGE_MPU6050) {
                            draw_mpu6050_page();
                        }
                        break;

                    case OLED_CMD_UPDATE_MAX30102:
                        // 更新MAX30102缓存
                        cached_heart_rate = msg.data.max30102.heart_rate;
                        cached_spo2 = msg.data.max30102.spo2;
                        cached_finger = msg.data.max30102.finger;
                        cached_max30102_valid = msg.data.max30102.valid;

                        // 如果当前在MAX30102界面，刷新显示
                        if (current_page == UI_PAGE_MAX30102) {
                            draw_max30102_page();
                        }
                        break;

                    case OLED_CMD_UPDATE_GPS:
                        // 更新GPS缓存
                        cached_lat = msg.data.gps.latitude;
                        cached_lon = msg.data.gps.longitude;
                        cached_satellites = msg.data.gps.satellites;
                        cached_gps_fixed = msg.data.gps.is_fixed;

                        // 如果当前在GPS界面，刷新显示
                        if (current_page == UI_PAGE_GPS) {
                            draw_gps_page();
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

// 更新MPU6050姿态显示
void oled_update_mpu6050(float pitch, float roll, float yaw, uint32_t steps, uint8_t valid) {
    if (!oled_queue) return;

    oled_msg_t msg;
    msg.cmd = OLED_CMD_UPDATE_MPU6050;
    msg.data.mpu6050.pitch = pitch;
    msg.data.mpu6050.roll = roll;
    msg.data.mpu6050.yaw = yaw;
    msg.data.mpu6050.steps = steps;
    msg.data.mpu6050.valid = valid;

    xQueueSend(oled_queue, &msg, 0);
}

// 更新MAX30102心率血氧显示
void oled_update_max30102(uint16_t heart_rate, uint8_t spo2, uint8_t finger, uint8_t valid) {
    if (!oled_queue) return;

    oled_msg_t msg;
    msg.cmd = OLED_CMD_UPDATE_MAX30102;
    msg.data.max30102.heart_rate = heart_rate;
    msg.data.max30102.spo2 = spo2;
    msg.data.max30102.finger = finger;
    msg.data.max30102.valid = valid;

    xQueueSend(oled_queue, &msg, 0);
}

// 更新GPS显示
void oled_update_gps(float lat, float lon, uint8_t satellites, uint8_t is_fixed) {
    if (!oled_queue) return;

    oled_msg_t msg;
    msg.cmd = OLED_CMD_UPDATE_GPS;
    msg.data.gps.latitude = lat;
    msg.data.gps.longitude = lon;
    msg.data.gps.satellites = satellites;
    msg.data.gps.is_fixed = is_fixed;

    xQueueSend(oled_queue, &msg, 0);
}