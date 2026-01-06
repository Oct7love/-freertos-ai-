# ESP32通信模块FreeRTOS移植技术文档

> 更新：2026-01-06 | 模块：ESP32-S3（小智AI） | 通信：UART3 DMA

---

## 一、系统架构概述

### 1.1 整体架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                        智能头盔系统架构                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────┐         UART3          ┌─────────────────────┐    │
│  │   STM32     │ ◄─────────────────────► │      ESP32-S3       │    │
│  │  (传感器)   │    115200bps DMA        │    (小智AI语音)     │    │
│  └─────────────┘                         └─────────────────────┘    │
│        │                                          │                 │
│        │ 采集                                     │ 云端            │
│        ▼                                          ▼                 │
│  ┌─────────────┐                         ┌─────────────────────┐    │
│  │ DHT11/MQ2   │                         │   MQTT服务器        │    │
│  │ MPU6050     │                         │   (xiaozhi.me)      │    │
│  │ MAX30102    │                         └─────────────────────┘    │
│  │ GPS         │                                                    │
│  └─────────────┘                                                    │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 1.2 通信协议设计

采用简单的JSON-like文本协议，便于调试和扩展：

| 方向 | 格式 | 示例 |
|------|------|------|
| STM32→ESP32 | `{DATA:类型:数据}\r\n` | `{DATA:TEMP:25,45}\r\n` |
| STM32→ESP32 | `{STATUS:状态}\r\n` | `{STATUS:OK}\r\n` |
| STM32→ESP32 | `{EVENT:事件}\r\n` | `{EVENT:FALL}\r\n` |
| STM32→ESP32 | `{SENSOR:名称:状态}\r\n` | `{SENSOR:TEMP:ON}\r\n` |
| ESP32→STM32 | `{CMD:命令}\r\n` | `{CMD:TEMP_START}\r\n` |

### 1.3 硬件连接

```
STM32F103          ESP32-S3
  PC10 (TX) ──────> GPIO18 (RX)
  PC11 (RX) <────── GPIO17 (TX)
  GND       ──────> GND
```

> **注意**：USART3使用部分重映射（PC10/PC11），需要在CubeMX中配置`__HAL_AFIO_REMAP_USART3_PARTIAL()`

---

## 二、STM32端实现

### 2.1 任务参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 任务名 | "esp32" | |
| 栈大小 | 512字 (2KB) | 字符串处理需要较大栈 |
| 优先级 | 2 | 与传感器任务相同 |
| 解析周期 | 50ms | 快速响应命令 |
| DMA缓冲区 | 128字节 | 接收缓冲 |
| 环形缓冲区 | 256字节 | 累积数据 |

### 2.2 命令定义

```c
// esp32_task.h
#define CMD_HR_START        "{CMD:HR_START}"
#define CMD_HR_STOP         "{CMD:HR_STOP}"
#define CMD_TEMP_START      "{CMD:TEMP_START}"
#define CMD_TEMP_STOP       "{CMD:TEMP_STOP}"
#define CMD_MPU_START       "{CMD:MPU_START}"
#define CMD_MPU_STOP        "{CMD:MPU_STOP}"
#define CMD_GPS_START       "{CMD:GPS_START}"
#define CMD_GPS_STOP        "{CMD:GPS_STOP}"
#define CMD_MQ2_START       "{CMD:MQ2_START}"
#define CMD_MQ2_STOP        "{CMD:MQ2_STOP}"
```

### 2.3 命令处理流程

```c
static void process_command(const char *cmd) {
    // 心率命令
    if (strstr(cmd, CMD_HR_START)) {
        if (max30102_is_running()) {
            esp32_send_status("ALREADY_ON");  // 已经在运行
        } else {
            max30102_task_start();
            esp32_send_status("OK");
        }
    }
    else if (strstr(cmd, CMD_HR_STOP)) {
        if (!max30102_is_running()) {
            esp32_send_status("ALREADY_OFF");  // 已经停止
        } else {
            max30102_task_stop();
            esp32_send_status("OK");
        }
    }
    // ... 其他传感器类似
}
```

### 2.4 数据发送接口

```c
// 心率血氧数据
void esp32_send_heartrate(uint8_t hr, uint8_t spo2);
// 格式: {DATA:HR:72,SPO2:98}\r\n

// 温湿度数据
void esp32_send_temperature(float temp, float humi);
// 格式: {DATA:TEMP:25,45}\r\n

// 姿态数据
void esp32_send_mpu6050(float pitch, float roll, float yaw);
// 格式: {DATA:MPU:-5.2,3.1,120.5}\r\n

// GPS数据
void esp32_send_gps(float lat, float lon, uint8_t satellites);
// 格式: {DATA:GPS:30.242796,120.205760,8}\r\n

// 烟雾数据
void esp32_send_mq2(uint16_t adc_value, uint8_t alarm);
// 格式: {DATA:MQ2:12.5,0}\r\n

// 状态响应
void esp32_send_status(const char *status);
// 格式: {STATUS:OK}\r\n 或 {STATUS:ALREADY_ON}\r\n

// 事件通知
void esp32_send_event(const char *event);
// 格式: {EVENT:FALL}\r\n 或 {EVENT:COLLISION}\r\n

// 传感器状态通知（按键控制时）
void esp32_send_sensor_state(const char *sensor, uint8_t on);
// 格式: {SENSOR:TEMP:ON}\r\n 或 {SENSOR:TEMP:OFF}\r\n
```

### 2.5 DMA发送实现

```c
static int esp32_send_dma(const char *data, uint16_t len) {
    if (!esp32_running || len == 0) return 0;

    xSemaphoreTake(esp32_tx_mutex, portMAX_DELAY);

    // 等待上次发送完成
    while (huart3.gState != HAL_UART_STATE_READY) {
        vTaskDelay(1);
    }

    // 复制数据到DMA缓冲区
    memcpy(esp32_tx_buffer, data, len);

    // 启动DMA发送
    HAL_UART_Transmit_DMA(&huart3, esp32_tx_buffer, len);

    // 等待发送完成信号量
    xSemaphoreTake(esp32_tx_done, pdMS_TO_TICKS(500));

    xSemaphoreGive(esp32_tx_mutex);
    return len;
}
```

---

## 三、ESP32端实现

### 3.1 STM32Controller类

ESP32端使用C++实现，核心是`STM32Controller`类：

```cpp
// stm32_controller.h
class STM32Controller {
private:
    // 传感器状态
    bool heart_rate_active_ = false;
    bool temperature_active_ = false;
    bool mpu6050_active_ = false;
    bool gps_active_ = false;
    bool mq2_active_ = false;

    // 最新数据
    int latest_heart_rate_ = 0;
    int latest_spo2_ = 0;
    float latest_temperature_ = 0.0f;
    int latest_humidity_ = 0;
    float latest_pitch_ = 0.0f;
    float latest_roll_ = 0.0f;
    float latest_yaw_ = 0.0f;
    float latest_mq2_ppm_ = 0.0f;
    int latest_mq2_alarm_ = 0;
    float latest_latitude_ = 0.0f;
    float latest_longitude_ = 0.0f;
    int latest_satellites_ = 0;

    // 行缓冲区（处理分片接收）
    char line_buffer_[256];
    int line_pos_ = 0;
};
```

### 3.2 数据解析（关键修复）

**问题**：ESP32 UART接收数据可能分片到达，导致解析失败

**解决**：添加行缓冲区，按`{...}`完整消息解析

```cpp
void ProcessRawData(const uint8_t* data, int len) {
    for (int i = 0; i < len; i++) {
        char c = (char)data[i];

        if (c == '\n' || c == '\r') {
            // 行结束，处理缓冲区
            if (line_pos_ > 0) {
                line_buffer_[line_pos_] = '\0';
                ProcessResponse(line_buffer_);
                line_pos_ = 0;
            }
        } else if (c == '{') {
            // 新消息开始
            line_pos_ = 0;
            line_buffer_[line_pos_++] = c;
        } else if (line_pos_ > 0 && line_pos_ < LINE_BUF_SIZE - 1) {
            line_buffer_[line_pos_++] = c;

            // 检测消息结束
            if (c == '}') {
                line_buffer_[line_pos_] = '\0';
                ProcessResponse(line_buffer_);
                line_pos_ = 0;
            }
        }
    }
}
```

### 3.3 响应处理（关键修复）

**问题**：`sscanf`从字符串开头解析，而不是从`strstr`找到的位置

**解决**：使用`strstr`返回的指针作为`sscanf`的输入

```cpp
void ProcessResponse(const char* response) {
    const char* pos = NULL;

    // 关键修复：使用strstr返回的位置进行sscanf解析
    if ((pos = strstr(response, "{DATA:HR:")) != NULL) {
        int hr = 0, spo2 = 0;
        if (sscanf(pos, "{DATA:HR:%d,SPO2:%d}", &hr, &spo2) == 2) {
            latest_heart_rate_ = hr;
            latest_spo2_ = spo2;
        }
    }
    else if ((pos = strstr(response, "{DATA:TEMP:")) != NULL) {
        int temp = 0, humi = 0;
        if (sscanf(pos, "{DATA:TEMP:%d,%d}", &temp, &humi) == 2) {
            latest_temperature_ = (float)temp;
            latest_humidity_ = humi;
        }
    }
    // ... 其他数据类型类似
}
```

### 3.4 MCP工具注册

ESP32通过MCP（Model Context Protocol）向小智AI暴露传感器控制接口：

```cpp
// 获取状态
mcp_server.AddTool("self.stm32.temperature.get_state",
    "Get the current temperature and humidity values",
    PropertyList(),
    [this](const PropertyList& properties) -> ReturnValue {
        char json[128];
        snprintf(json, sizeof(json),
            "{\"active\": %s, \"temperature\": %.1f, \"humidity\": %d}",
            temperature_active_ ? "true" : "false",
            latest_temperature_, latest_humidity_);
        return std::string(json);
    });

// 启动传感器（带状态检查）
mcp_server.AddTool("self.stm32.temperature.start",
    "Start temperature monitoring",
    PropertyList(),
    [this](const PropertyList& properties) -> ReturnValue {
        if (temperature_active_) {
            return std::string("Temperature sensor is already running");
        }
        temperature_active_ = true;
        SendCommand("{CMD:TEMP_START}\r\n");
        return true;
    });

// 停止传感器（带状态检查）
mcp_server.AddTool("self.stm32.temperature.stop",
    "Stop temperature monitoring",
    PropertyList(),
    [this](const PropertyList& properties) -> ReturnValue {
        if (!temperature_active_) {
            return std::string("Temperature sensor is already stopped");
        }
        temperature_active_ = false;
        SendCommand("{CMD:TEMP_STOP}\r\n");
        return true;
    });
```

---

## 四、遇到的困难与解决方案

### 4.1 问题1：ESP32显示错误数据

**现象**：
- 温度显示23.8°C，实际应为25°C
- MQ2显示300ppm，实际约10ppm
- 心率显示72bpm，但还未测量

**原因**：
1. `sscanf`从字符串开头解析，而不是从`strstr`找到的位置
2. 多条消息粘连时，解析位置错误

**解决**：使用`strstr`返回的指针作为`sscanf`输入（见3.3节）

### 4.2 问题2：数据分片导致解析失败

**现象**：部分数据解析失败，日志显示不完整的消息

**原因**：UART接收数据可能分多次到达，如`{DATA:TE`和`MP:25,45}\r\n`

**解决**：添加行缓冲区，累积数据直到收到完整的`{...}`消息（见3.2节）

### 4.3 问题3：状态同步问题（未完全解决）

**现象**：
1. 用户通过按键打开温度传感器
2. 让AI关闭 → AI发送关闭命令 → STM32返回`{STATUS:ALREADY_OFF}`
3. 原因：用户已经通过按键关闭了，但AI不知道

**部分解决**：
1. STM32按键控制时发送`{SENSOR:TEMP:ON/OFF}`通知ESP32
2. ESP32更新本地状态变量
3. MCP工具在发送命令前检查状态

**未解决部分**：
- AI助手有时不调用MCP工具就直接回复"已关闭"
- 这是AI行为问题，需要在AI提示词中强调"每次都必须调用工具"

### 4.4 问题4：USART3 DMA配置

**现象**：UART3无法收发数据

**原因**：
1. USART3需要部分重映射到PC10/PC11
2. DMA通道配置错误

**解决**：
```c
// 在HAL_UART_MspInit中添加
__HAL_AFIO_REMAP_USART3_PARTIAL();

// DMA配置
hdma_usart3_rx.Instance = DMA1_Channel3;  // RX
hdma_usart3_tx.Instance = DMA1_Channel2;  // TX
```

### 4.5 问题5：DMA发送完成同步

**现象**：连续发送数据时出现数据丢失或覆盖

**原因**：未等待上次DMA发送完成就开始下次发送

**解决**：
1. 使用二值信号量`esp32_tx_done`同步
2. DMA发送完成回调中释放信号量
3. 发送前等待信号量

```c
void esp32_uart_tx_callback(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(esp32_tx_done, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```

---

## 五、当前未解决的问题

### 5.1 AI助手不调用MCP工具

**问题描述**：
当用户说"关闭温度传感器"时，AI有时直接回复"已关闭"而不调用`self.stm32.temperature.stop`工具。

**影响**：
- 传感器实际未关闭，继续发送数据
- 用户以为已关闭，造成困惑

**可能原因**：
- AI记住了之前的状态，认为不需要再次调用
- AI提示词中未强调必须调用工具

**建议解决方案**：
在小智AI的系统提示词中添加：
```
当用户要求打开或关闭传感器时，必须调用对应的MCP工具（如 self.stm32.temperature.start/stop），
不要依赖之前的记忆来判断传感器状态。每次都要调用工具来确保状态正确。
```

### 5.2 语音识别准确率

**问题描述**：
用户说"温湿度传感器"，AI识别为"winN湿度传感器"或"win湿度升改器"

**影响**：
- 命令无法正确执行
- 用户体验差

**可能原因**：
- 语音识别模型对专业术语识别不准
- 环境噪音干扰

**建议解决方案**：
1. 在AI提示词中添加同义词映射
2. 使用更清晰的命令词（如"打开温度"而非"打开温湿度传感器"）

---

## 六、API接口汇总

### 6.1 STM32端接口

```c
// 初始化与控制
void esp32_task_init(void);
void esp32_task_start(void);
void esp32_task_stop(void);
uint8_t esp32_is_running(void);

// 数据发送
void esp32_send_heartrate(uint8_t hr, uint8_t spo2);
void esp32_send_temperature(float temp, float humi);
void esp32_send_mpu6050(float pitch, float roll, float yaw);
void esp32_send_gps(float lat, float lon, uint8_t satellites);
void esp32_send_mq2(uint16_t adc_value, uint8_t alarm);
void esp32_send_status(const char *status);
void esp32_send_event(const char *event);
void esp32_send_sensor_state(const char *sensor, uint8_t on);

// 回调函数
void esp32_uart_rx_callback(uint8_t *data, uint16_t len);
void esp32_uart_tx_callback(void);
```

### 6.2 ESP32端MCP工具

| 工具名 | 功能 |
|--------|------|
| `self.stm32.heart_rate.get_state` | 获取心率血氧状态和数据 |
| `self.stm32.heart_rate.start` | 启动心率检测 |
| `self.stm32.heart_rate.stop` | 停止心率检测 |
| `self.stm32.temperature.get_state` | 获取温湿度状态和数据 |
| `self.stm32.temperature.start` | 启动温度检测 |
| `self.stm32.temperature.stop` | 停止温度检测 |
| `self.stm32.mpu6050.get_state` | 获取姿态状态和数据 |
| `self.stm32.mpu6050.start` | 启动姿态检测 |
| `self.stm32.mpu6050.stop` | 停止姿态检测 |
| `self.stm32.mq2.get_state` | 获取烟雾状态和数据 |
| `self.stm32.mq2.start` | 启动烟雾检测 |
| `self.stm32.mq2.stop` | 停止烟雾检测 |
| `self.stm32.gps.get_state` | 获取GPS状态和数据 |
| `self.stm32.gps.start` | 启动GPS定位 |
| `self.stm32.gps.stop` | 停止GPS定位 |

---

## 七、文件清单

### STM32端
```
Core/task/
├── esp32_task.c        # ESP32通信任务实现
├── esp32_task.h        # ESP32通信任务接口
Core/Src/
├── usart.c             # UART3 DMA配置及回调
```

### ESP32端
```
ESP32_XiaoZhi/main/boards/common/
├── stm32_controller.h  # STM32控制器类（含MCP工具）
```

---

## 八、调试技巧

### 8.1 STM32端调试

```c
// 查看接收到的命令
[ESP32] RX: {CMD:TEMP_START}

// 查看发送的数据
uart_printf_dma(&huart1, "[ESP32] TX: %s", buf);
```

### 8.2 ESP32端调试

```
// 查看接收到的数据
I (12293) STM32Controller: 收到STM32数据: {DATA:TEMP:24,39}
I (12293) STM32Controller: 温度: 24°C, 湿度: 39%

// 查看MCP工具调用
I (19533) Application: << % self.stm32.temperature.start...
I (19533) STM32Controller: 温度检测已在运行中
```

### 8.3 通信协议测试

使用串口助手直接发送命令测试：
```
发送: {CMD:TEMP_START}\r\n
期望响应: {STATUS:OK}\r\n 或 {STATUS:ALREADY_ON}\r\n
```

---

## 九、参考资料

1. ESP-IDF UART编程指南
2. STM32 HAL库UART DMA使用
3. 小智AI MCP协议文档
4. FreeRTOS任务间通信机制
