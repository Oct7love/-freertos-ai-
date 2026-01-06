#ifndef __STM32_CONTROLLER_H__
#define __STM32_CONTROLLER_H__

#include "mcp_server.h"
#include "display.h"
#include "board.h"
#include "application.h"
#include <esp_log.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>

#define STM32_UART_NUM UART_NUM_1
#define STM32_TXD_PIN  (GPIO_NUM_17)
#define STM32_RXD_PIN  (GPIO_NUM_18)
#define STM32_UART_BAUD_RATE 115200
#define BUF_SIZE (1024)
#define LINE_BUF_SIZE (256)  // 行缓冲区大小

class STM32Controller {
private:
    bool heart_rate_active_ = false;
    bool temperature_active_ = false;
    bool mpu6050_active_ = false;
    bool gps_active_ = false;
    bool mq2_active_ = false;

    int latest_heart_rate_ = 0;
    int latest_spo2_ = 0;
    float latest_temperature_ = 0.0f;
    int latest_humidity_ = 0;           // ← 新添加
    float latest_pitch_ = 0.0f;
    float latest_roll_ = 0.0f;
    float latest_yaw_ = 0.0f;
    float latest_mq2_ppm_ = 0.0f;      // ← 新添加的第35行
    int latest_mq2_alarm_ = 0;          // ← 新添加的第36行

        // GPS数据
      float latest_latitude_ = 0.0f;
      float latest_longitude_ = 0.0f;
      int latest_satellites_ = 0;

    // 行缓冲区（处理分片接收）
    char line_buffer_[LINE_BUF_SIZE];
    int line_pos_ = 0;


    static constexpr const char* TAG = "STM32Controller";

    void InitializeUART() {
        uart_config_t uart_config = {
            .baud_rate = STM32_UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
            .source_clk = UART_SCLK_DEFAULT,
        };

        ESP_ERROR_CHECK(uart_param_config(STM32_UART_NUM, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(STM32_UART_NUM, STM32_TXD_PIN, STM32_RXD_PIN,
                                      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        ESP_ERROR_CHECK(uart_driver_install(STM32_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));

        ESP_LOGI(TAG, "UART initialized: TX=%d, RX=%d, Baud=%d",
                 STM32_TXD_PIN, STM32_RXD_PIN, STM32_UART_BAUD_RATE);

        xTaskCreate(uart_rx_task_static, "stm32_uart_rx", 4096, this, 10, NULL);
    }

    static void uart_rx_task_static(void* arg) {
        STM32Controller* controller = (STM32Controller*)arg;
        controller->uart_rx_task();
    }

    // 处理接收到的原始数据，按行分割后调用ProcessResponse
    void ProcessRawData(const uint8_t* data, int len) {
        for (int i = 0; i < len; i++) {
            char c = (char)data[i];

            if (c == '\n' || c == '\r') {
                // 行结束，处理缓冲区中的数据
                if (line_pos_ > 0) {
                    line_buffer_[line_pos_] = '\0';
                    ProcessResponse(line_buffer_);
                    line_pos_ = 0;
                }
            } else if (c == '{') {
                // 新消息开始，重置缓冲区
                line_pos_ = 0;
                line_buffer_[line_pos_++] = c;
            } else if (line_pos_ > 0 && line_pos_ < LINE_BUF_SIZE - 1) {
                // 继续填充缓冲区
                line_buffer_[line_pos_++] = c;

                // 检测消息结束（遇到 }）
                if (c == '}') {
                    line_buffer_[line_pos_] = '\0';
                    ProcessResponse(line_buffer_);
                    line_pos_ = 0;
                }
            }
        }
    }

    void uart_rx_task() {
        uint8_t data[BUF_SIZE];
        line_pos_ = 0;  // 初始化行缓冲区位置

        while (1) {
            int len = uart_read_bytes(STM32_UART_NUM, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
            if (len > 0) {
                ProcessRawData(data, len);
            }
        }
    }

    void ProcessResponse(const char* response) {
        ESP_LOGI(TAG, "收到STM32数据: %s", response);

        auto& board = Board::GetInstance();
        auto display = board.GetDisplay();
        char notify_msg[128];
        const char* pos = NULL;

        // 关键修复：使用strstr返回的位置进行sscanf解析
        if ((pos = strstr(response, "{DATA:HR:")) != NULL) {
            int hr = 0, spo2 = 0;
            if (sscanf(pos, "{DATA:HR:%d,SPO2:%d}", &hr, &spo2) == 2) {
                latest_heart_rate_ = hr;
                latest_spo2_ = spo2;
                ESP_LOGI(TAG, "心率: %d bpm, 血氧: %d%%", hr, spo2);
                snprintf(notify_msg, sizeof(notify_msg), "心率:%d 血氧:%d%%", hr, spo2);
                if (display) display->ShowNotification(notify_msg);
            }
        }
        else if ((pos = strstr(response, "{DATA:TEMP:")) != NULL) {
              int temp = 0, humi = 0;
              if (sscanf(pos, "{DATA:TEMP:%d,%d}", &temp, &humi) == 2) {
                  latest_temperature_ = (float)temp;
                  latest_humidity_ = humi;
                  ESP_LOGI(TAG, "温度: %d°C, 湿度: %d%%", temp, humi);
                  snprintf(notify_msg, sizeof(notify_msg), "温度:%d°C 湿度:%d%%", temp, humi);
                  if (display) display->ShowNotification(notify_msg);
              }
          }
        else if ((pos = strstr(response, "{DATA:MPU:")) != NULL) {
            float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
            if (sscanf(pos, "{DATA:MPU:%f,%f,%f}", &pitch, &roll, &yaw) == 3) {
                latest_pitch_ = pitch;
                latest_roll_ = roll;
                latest_yaw_ = yaw;
                ESP_LOGI(TAG, "姿态: Pitch=%.1f° Roll=%.1f° Yaw=%.1f°", pitch, roll, yaw);
                if (display) display->ShowNotification("姿态数据已更新");
            }
        }
         else if ((pos = strstr(response, "{DATA:MQ2:")) != NULL) {
              float ppm = 0.0f;
              int alarm = 0;
              if (sscanf(pos, "{DATA:MQ2:%f,%d}", &ppm, &alarm) == 2) {
                  latest_mq2_ppm_ = ppm;
                  latest_mq2_alarm_ = alarm;
                  ESP_LOGI(TAG, "烟雾: PPM=%.1f, 报警级别=%d", ppm, alarm);

                  // 显示数据
                  char notify_msg[128];
                  const char* level_str[] = {"安全", "低", "中", "高"};
                  snprintf(notify_msg, sizeof(notify_msg), "烟雾:%.1fppm [%s]",
                          ppm, alarm < 4 ? level_str[alarm] : "危险");
                  if (display) display->ShowNotification(notify_msg);
              }
          }
           else if ((pos = strstr(response, "{DATA:GPS:")) != NULL) {
              float lat = 0.0f, lon = 0.0f;
              int satellites = 0;
              if (sscanf(pos, "{DATA:GPS:%f,%f,%d}", &lat, &lon, &satellites) == 3) {
                  latest_latitude_ = lat;
                  latest_longitude_ = lon;
                  latest_satellites_ = satellites;
                  ESP_LOGI(TAG, "GPS: 纬度=%.6f, 经度=%.6f, 卫星=%d", lat, lon, satellites);
                  snprintf(notify_msg, sizeof(notify_msg), "GPS:%.5f,%.5f", lat, lon);
                  if (display) display->ShowNotification(notify_msg);
              }
          }                                
        else if (strstr(response, "{STATUS:")) {
            if (strstr(response, "OK")) {
                ESP_LOGI(TAG, "命令执行成功");
                if (display) display->ShowNotification("STM32: OK");
            } else if (strstr(response, "ALREADY_ON")) {
                ESP_LOGI(TAG, "传感器已经打开");
                if (display) display->ShowNotification("传感器已经打开");
            } else if (strstr(response, "ALREADY_OFF")) {
                ESP_LOGI(TAG, "传感器已经关闭");
                if (display) display->ShowNotification("传感器已经关闭");
            } else if (strstr(response, "ERROR")) {
                ESP_LOGW(TAG, "命令执行失败");
                if (display) display->ShowNotification("STM32: ERROR");
            }
        }
        // 传感器状态同步（按键控制时 STM32 主动通知）
        else if (strstr(response, "{SENSOR:")) {
            if (strstr(response, "HR:ON")) {
                heart_rate_active_ = true;
                ESP_LOGI(TAG, "心率传感器已被按键打开");
            } else if (strstr(response, "HR:OFF")) {
                heart_rate_active_ = false;
                ESP_LOGI(TAG, "心率传感器已被按键关闭");
            } else if (strstr(response, "TEMP:ON")) {
                temperature_active_ = true;
                ESP_LOGI(TAG, "温度传感器已被按键打开");
            } else if (strstr(response, "TEMP:OFF")) {
                temperature_active_ = false;
                ESP_LOGI(TAG, "温度传感器已被按键关闭");
            } else if (strstr(response, "MPU:ON")) {
                mpu6050_active_ = true;
                ESP_LOGI(TAG, "姿态传感器已被按键打开");
            } else if (strstr(response, "MPU:OFF")) {
                mpu6050_active_ = false;
                ESP_LOGI(TAG, "姿态传感器已被按键关闭");
            } else if (strstr(response, "GPS:ON")) {
                gps_active_ = true;
                ESP_LOGI(TAG, "GPS已被按键打开");
            } else if (strstr(response, "GPS:OFF")) {
                gps_active_ = false;
                ESP_LOGI(TAG, "GPS已被按键关闭");
            } else if (strstr(response, "MQ2:ON")) {
                mq2_active_ = true;
                ESP_LOGI(TAG, "烟雾传感器已被按键打开");
            } else if (strstr(response, "MQ2:OFF")) {
                mq2_active_ = false;
                ESP_LOGI(TAG, "烟雾传感器已被按键关闭");
            }
        }
        else if (strstr(response, "{EVENT:FALL}")) {
            ESP_LOGW(TAG, "⚠️ 检测到摔倒事件！");

            auto& board = Board::GetInstance();
            auto display = board.GetDisplay();
            if (display) {
                display->ShowNotification("⚠️ 摔倒警告！");
            }

            // 播放警告音效
            auto& app = Application::GetInstance();
            app.PlaySound("alert");

            ESP_LOGW(TAG, "警告：检测到摔倒，请注意安全！");
        }
        else if (strstr(response, "{EVENT:COLLISION}")) {
            ESP_LOGW(TAG, "⚠️ 检测到碰撞事件！");

            auto& board = Board::GetInstance();
            auto display = board.GetDisplay();
            if (display) {
                display->ShowNotification("⚠️ 碰撞警告！");
            }

            // 播放警告音效
            auto& app = Application::GetInstance();
            app.PlaySound("alert");

            ESP_LOGW(TAG, "警告：检测到碰撞，请小心！");
        }else if (strstr(response, "{EVENT:SMOKE}")) {    // ← 从这里开始添加
              ESP_LOGW(TAG, "⚠️ 检测到烟雾事件！");

              auto& board = Board::GetInstance();
              auto display = board.GetDisplay();
              if (display) {
                  display->ShowNotification("⚠️ 烟雾警告！");
              }

              // 播放警告音效
              auto& app = Application::GetInstance();
              app.PlaySound("alert");

              ESP_LOGW(TAG, "警告：检测到烟雾，请远离火源！");
          }      

    }

    void SendCommand(const char* cmd) {
        int len = strlen(cmd);
        int written = uart_write_bytes(STM32_UART_NUM, cmd, len);
        if (written == len) {
            ESP_LOGI(TAG, "发送命令: %s", cmd);
        } else {
            ESP_LOGE(TAG, "发送命令失败");
        }
    }

public:
    STM32Controller() {
        ESP_LOGI(TAG, "Initializing STM32Controller");
        InitializeUART();

        auto& mcp_server = McpServer::GetInstance();

        // ============ 心率检测 ============
        mcp_server.AddTool("self.stm32.heart_rate.get_state",
            "Get real-time heart rate/SpO2 data AND sensor status from helmet MAX30102.\n"
            "MUST call this tool when user asks:\n"
            "- Heart rate/SpO2 values: '心率多少', '血氧', 'heart rate', 'SpO2', 'pulse'\n"
            "- Sensor status: '心率传感器开了吗', '打开了吗', 'is it on', 'is it running'\n"
            "- ANY question about heart rate sensor state or data\n"
            "NEVER use memory or cached values - ALWAYS call this tool for real-time data.\n"
            "Returns: {active: true/false, heart_rate: bpm, spo2: %}",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                char json[256];
                snprintf(json, sizeof(json),
                    "{\"active\": %s, \"heart_rate\": %d, \"spo2\": %d}",
                    heart_rate_active_ ? "true" : "false",
                    latest_heart_rate_,
                    latest_spo2_);
                return std::string(json);
            });

        mcp_server.AddTool("self.stm32.heart_rate.start",
            "Start the helmet's heart rate sensor (MAX30102).\n"
            "MUST call this tool when user says: '打开心率', '开启心率检测', '测心率', 'start heart rate'\n"
            "This controls REAL hardware - always call this tool, never just reply without action.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                if (heart_rate_active_) {
                    ESP_LOGI(TAG, "心率检测已在运行中");
                    return std::string("Heart rate sensor is already running");
                }
                heart_rate_active_ = true;
                SendCommand("{CMD:HR_START}\r\n");
                ESP_LOGI(TAG, "心率检测已启动");
                auto display = Board::GetInstance().GetDisplay();
                if (display) display->ShowNotification("心率检测已启动");
                return true;
            });

        mcp_server.AddTool("self.stm32.heart_rate.stop",
            "Stop the helmet's heart rate sensor (MAX30102).\n"
            "MUST call this tool when user says: '关闭心率', '停止心率检测', 'stop heart rate'\n"
            "This controls REAL hardware - always call this tool, never just reply without action.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                if (!heart_rate_active_) {
                    ESP_LOGI(TAG, "心率检测已停止");
                    return std::string("Heart rate sensor is already stopped");
                }
                heart_rate_active_ = false;
                SendCommand("{CMD:HR_STOP}\r\n");
                ESP_LOGI(TAG, "心率检测已停止");
                auto display = Board::GetInstance().GetDisplay();
                if (display) display->ShowNotification("心率检测已停止");
                return true;
            });

        // ============ 温度检测 ============
       mcp_server.AddTool("self.stm32.temperature.get_state",
              "Get real-time temperature/humidity data AND sensor status from helmet DHT11.\n"
              "MUST call this tool when user asks:\n"
              "- Temperature/humidity values: '温度多少', '湿度', 'temperature', 'humidity'\n"
              "- Sensor status: '传感器开了吗', '打开了吗', 'is it on', 'is it running'\n"
              "- ANY question about temperature sensor state or data\n"
              "NEVER use memory or cached values - ALWAYS call this tool for real-time data.\n"
              "Returns: {active: true/false, temperature: °C, humidity: %}",
              PropertyList(),
              [this](const PropertyList& properties) -> ReturnValue {
                  char json[128];
                  snprintf(json, sizeof(json),
                      "{\"active\": %s, \"temperature\": %.1f, \"humidity\": %d}",
                      temperature_active_ ? "true" : "false",
                      latest_temperature_,
                      latest_humidity_);
                  return std::string(json);
              });

        mcp_server.AddTool("self.stm32.temperature.start",
            "Start the helmet's temperature/humidity sensor (DHT11).\n"
            "MUST call this tool when user says: '打开温度', '打开温湿度', 'start temperature'\n"
            "WARNING: Sensor state may change by physical buttons. Call get_state first if unsure.\n"
            "This controls REAL hardware - always call this tool, never just reply.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                if (temperature_active_) {
                    ESP_LOGI(TAG, "温度检测已在运行中");
                    return std::string("Temperature sensor is already running");
                }
                temperature_active_ = true;
                SendCommand("{CMD:TEMP_START}\r\n");
                ESP_LOGI(TAG, "温度检测已启动");
                auto display = Board::GetInstance().GetDisplay();
                if (display) display->ShowNotification("温度检测已启动");
                return true;
            });

        mcp_server.AddTool("self.stm32.temperature.stop",
            "Stop the helmet's temperature/humidity sensor (DHT11).\n"
            "MUST call this tool when user says: '关闭温度', '关闭温湿度', 'stop temperature'\n"
            "WARNING: Sensor state may change by physical buttons. Call get_state first if unsure.\n"
            "This controls REAL hardware - always call this tool, never just reply.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                if (!temperature_active_) {
                    ESP_LOGI(TAG, "温度检测已停止");
                    return std::string("Temperature sensor is already stopped");
                }
                temperature_active_ = false;
                SendCommand("{CMD:TEMP_STOP}\r\n");
                ESP_LOGI(TAG, "温度检测已停止");
                auto display = Board::GetInstance().GetDisplay();
                if (display) display->ShowNotification("温度检测已停止");
                return true;
            });

        // ============ 姿态检测 ============
        mcp_server.AddTool("self.stm32.mpu6050.get_state",
            "Get real-time attitude data AND sensor status from helmet MPU6050.\n"
            "MUST call this tool when user asks:\n"
            "- Attitude values: '姿态多少', '角度', 'pitch', 'roll', 'yaw', 'tilt'\n"
            "- Sensor status: '姿态传感器开了吗', '打开了吗', 'is it on'\n"
            "- ANY question about MPU6050 sensor state or data\n"
            "NEVER use memory - ALWAYS call this tool for real-time data.\n"
            "Returns: {active: true/false, pitch: °, roll: °, yaw: °}",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                char json[256];
                snprintf(json, sizeof(json),
                    "{\"active\": %s, \"pitch\": %.1f, \"roll\": %.1f, \"yaw\": %.1f}",
                    mpu6050_active_ ? "true" : "false",
                    latest_pitch_,
                    latest_roll_,
                    latest_yaw_);
                return std::string(json);
            });

        mcp_server.AddTool("self.stm32.mpu6050.start",
            "Start the helmet's attitude sensor (MPU6050).\n"
            "MUST call this tool when user says: '打开姿态', '开启姿态检测', '打开陀螺仪', 'start MPU'\n"
            "This controls REAL hardware - always call this tool, never just reply without action.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                if (mpu6050_active_) {
                    ESP_LOGI(TAG, "姿态检测已在运行中");
                    return std::string("MPU6050 sensor is already running");
                }
                mpu6050_active_ = true;
                SendCommand("{CMD:MPU_START}\r\n");
                ESP_LOGI(TAG, "姿态检测已启动");
                auto display = Board::GetInstance().GetDisplay();
                if (display) display->ShowNotification("姿态检测已启动");
                return true;
            });

        mcp_server.AddTool("self.stm32.mpu6050.stop",
            "Stop the helmet's attitude sensor (MPU6050).\n"
            "MUST call this tool when user says: '关闭姿态', '停止姿态检测', '关闭陀螺仪', 'stop MPU'\n"
            "This controls REAL hardware - always call this tool, never just reply without action.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                if (!mpu6050_active_) {
                    ESP_LOGI(TAG, "姿态检测已停止");
                    return std::string("MPU6050 sensor is already stopped");
                }
                mpu6050_active_ = false;
                SendCommand("{CMD:MPU_STOP}\r\n");
                ESP_LOGI(TAG, "姿态检测已停止");
                auto display = Board::GetInstance().GetDisplay();
                if (display) display->ShowNotification("姿态检测已停止");
                return true;
            });
  // ============ 烟雾检测 ============          // ← 从这里开始添加
    mcp_server.AddTool("self.stm32.mq2.get_state",
                "Get real-time smoke/gas data AND sensor status from helmet MQ2.\n"
                "MUST call this tool when user asks:\n"
                "- Smoke/gas values: '烟雾浓度', '气体', 'smoke', 'gas', 'air quality'\n"
                "- Sensor status: '烟雾传感器开了吗', '打开了吗', 'is it on'\n"
                "- ANY question about MQ2 sensor state or data\n"
                "NEVER use memory - ALWAYS call this tool for real-time data.\n"
                "Returns: {active: true/false, ppm: concentration, alarm_level: 0-4}",
                PropertyList(),
                [this](const PropertyList& properties) -> ReturnValue {
                    char json[256];
                    snprintf(json, sizeof(json),
                        "{\"active\": %s, \"ppm\": %.1f, \"alarm_level\": %d}",
                        mq2_active_ ? "true" : "false",
                        latest_mq2_ppm_,
                        latest_mq2_alarm_);
                    return std::string(json);
                });     
          mcp_server.AddTool("self.stm32.mq2.start",
              "Start the helmet's smoke/gas sensor (MQ2).\n"
              "MUST call this tool when user says: '打开烟雾', '开启烟雾检测', '打开气体传感器', 'start smoke'\n"
              "This controls REAL hardware - always call this tool, never just reply without action.",
              PropertyList(),
              [this](const PropertyList& properties) -> ReturnValue {
                  if (mq2_active_) {
                      ESP_LOGI(TAG, "烟雾检测已在运行中");
                      return std::string("MQ2 sensor is already running");
                  }
                  mq2_active_ = true;
                  SendCommand("{CMD:MQ2_START}\r\n");
                  ESP_LOGI(TAG, "烟雾检测已启动");
                  auto display = Board::GetInstance().GetDisplay();
                  if (display) display->ShowNotification("烟雾检测已启动");
                  return true;
              });

          mcp_server.AddTool("self.stm32.mq2.stop",
              "Stop the helmet's smoke/gas sensor (MQ2).\n"
              "MUST call this tool when user says: '关闭烟雾', '停止烟雾检测', '关闭气体传感器', 'stop smoke'\n"
              "This controls REAL hardware - always call this tool, never just reply without action.",
              PropertyList(),
              [this](const PropertyList& properties) -> ReturnValue {
                  if (!mq2_active_) {
                      ESP_LOGI(TAG, "烟雾检测已停止");
                      return std::string("MQ2 sensor is already stopped");
                  }
                  mq2_active_ = false;
                  SendCommand("{CMD:MQ2_STOP}\r\n");
                  ESP_LOGI(TAG, "烟雾检测已停止");
                  auto display = Board::GetInstance().GetDisplay();
                  if (display) display->ShowNotification("烟雾检测已停止");
                  return true;
              });
               // ============ GPS定位 ============
          mcp_server.AddTool("self.stm32.gps.get_state",
              "Get real-time GPS location data AND sensor status from helmet GPS module.\n"
              "MUST call this tool when user asks:\n"
              "- Location values: '位置', '坐标', '经纬度', 'GPS', 'location', 'where am I'\n"
              "- Sensor status: 'GPS开了吗', '定位打开了吗', 'is GPS on'\n"
              "- ANY question about GPS sensor state or data\n"
              "NEVER use memory - ALWAYS call this tool for real-time data.\n"
              "Returns: {active: true/false, latitude, longitude, satellites}",
              PropertyList(),
              [this](const PropertyList& properties) -> ReturnValue {
                  char json[256];
                  snprintf(json, sizeof(json),
                      "{\"active\": %s, \"latitude\": %.6f, \"longitude\": %.6f, \"satellites\": %d}",
                      gps_active_ ? "true" : "false",
                      latest_latitude_,
                      latest_longitude_,
                      latest_satellites_);
                  return std::string(json);
              });

          mcp_server.AddTool("self.stm32.gps.start",
             "Start the helmet's GPS module for location tracking.\n"
             "MUST call this tool when user says: '打开GPS', '开启定位', '打开导航', 'start GPS'\n"
             "This controls REAL hardware - always call this tool, never just reply without action.",
              PropertyList(),
              [this](const PropertyList& properties) -> ReturnValue {
                  if (gps_active_) {
                      ESP_LOGI(TAG, "GPS定位已在运行中");
                      return std::string("GPS is already running");
                  }
                  gps_active_ = true;
                  SendCommand("{CMD:GPS_START}\r\n");
                  ESP_LOGI(TAG, "GPS定位已启动");
                  auto display = Board::GetInstance().GetDisplay();
                  if (display) display->ShowNotification("GPS定位已启动");
                  return true;
              });

          mcp_server.AddTool("self.stm32.gps.stop",
              "Stop the helmet's GPS module.\n"
              "MUST call this tool when user says: '关闭GPS', '停止定位', '关闭导航', 'stop GPS'\n"
              "This controls REAL hardware - always call this tool, never just reply without action.",
              PropertyList(),
              [this](const PropertyList& properties) -> ReturnValue {
                  if (!gps_active_) {
                      ESP_LOGI(TAG, "GPS定位已停止");
                      return std::string("GPS is already stopped");
                  }
                  gps_active_ = false;
                  SendCommand("{CMD:GPS_STOP}\r\n");
                  ESP_LOGI(TAG, "GPS定位已停止");
                  auto display = Board::GetInstance().GetDisplay();
                  if (display) display->ShowNotification("GPS定位已停止");
                  return true;
              });            
        ESP_LOGI(TAG, "STM32Controller initialized successfully");
    }
};

#endif // __STM32_CONTROLLER_H__
