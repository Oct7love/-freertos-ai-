# 小智AI项目架构分析

> 本文档深入分析 ESP32_XiaoZhi 项目的架构设计、核心框架和实现原理

---

## 目录

1. [项目概述](#1-项目概述)
2. [整体架构](#2-整体架构)
3. [核心模块分析](#3-核心模块分析)
   - 3.1 Application 应用层
   - 3.2 Board 硬件抽象层
   - 3.3 Protocol 通信协议层
   - 3.4 AudioService 音频服务层
4. [MCP工具系统](#4-mcp工具系统)
5. [音频处理流水线](#5-音频处理流水线)
6. [多开发板支持机制](#6-多开发板支持机制)
7. [网络通信详解](#7-网络通信详解)
8. [与STM32协同工作](#8-与stm32协同工作)

---

## 1. 项目概述

### 1.1 项目定位

小智AI是一个运行在ESP32-S3上的智能语音助手固件，具备以下核心能力：

| 能力 | 描述 |
|------|------|
| 语音唤醒 | 本地唤醒词检测，无需联网 |
| 语音识别 | 云端ASR，实时转文字 |
| AI对话 | 接入大语言模型，智能问答 |
| 语音合成 | 云端TTS，自然语音输出 |
| 硬件控制 | 通过MCP协议控制外设 |

### 1.2 技术栈

```
┌─────────────────────────────────────────────────┐
│                   应用层                         │
│  Application · McpServer · AudioService         │
├─────────────────────────────────────────────────┤
│                   协议层                         │
│  WebSocket Protocol · MQTT Protocol             │
├─────────────────────────────────────────────────┤
│                  硬件抽象层                      │
│  Board · AudioCodec · Display · Network         │
├─────────────────────────────────────────────────┤
│                   系统层                         │
│  ESP-IDF · FreeRTOS · WiFi · I2S · UART         │
└─────────────────────────────────────────────────┘
```

### 1.3 目录结构

```
ESP32_XiaoZhi/
├── main/
│   ├── application.h/cc      # 应用主类
│   ├── mcp_server.h          # MCP工具系统
│   ├── audio/                # 音频处理模块
│   │   ├── audio_service.h   # 音频服务
│   │   ├── audio_processor.h # 音频处理器
│   │   └── wake_word/        # 唤醒词检测
│   ├── protocols/            # 通信协议
│   │   ├── protocol.h        # 协议基类
│   │   ├── mqtt_protocol.h   # MQTT协议实现
│   │   └── websocket_protocol.h # WebSocket实现
│   ├── boards/               # 开发板适配
│   │   ├── common/           # 公共组件
│   │   │   ├── board.h       # Board基类
│   │   │   ├── lamp_controller.h
│   │   │   ├── fan_controller.h
│   │   │   └── stm32_controller.h
│   │   └── bread-compact-wifi/ # 具体板型
│   ├── display/              # 显示模块
│   └── codecs/               # 音频编解码
└── components/               # 第三方组件
```

---

## 2. 整体架构

### 2.1 架构图

```
┌──────────────────────────────────────────────────────────────────┐
│                        小智云服务器                               │
│    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐        │
│    │  ASR 语音   │    │  LLM 大模型 │    │  TTS 语音   │        │
│    │    识别     │    │   推理      │    │    合成     │        │
│    └─────────────┘    └─────────────┘    └─────────────┘        │
└────────────────────────────────────────────────────────────────┬─┘
                             │ WebSocket / MQTT
                             ▼
┌──────────────────────────────────────────────────────────────────┐
│                      ESP32-S3 小智固件                            │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │                    Application (单例)                       │  │
│  │  - 设备状态管理 (Idle/Listening/Speaking/Thinking)         │  │
│  │  - 主事件循环                                               │  │
│  │  - 调度各子系统                                             │  │
│  └────────────────────────────────────────────────────────────┘  │
│       │              │              │              │              │
│       ▼              ▼              ▼              ▼              │
│  ┌─────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐       │
│  │Protocol │   │AudioSvc  │   │McpServer │   │  Board   │       │
│  │通信协议 │   │音频服务  │   │MCP工具   │   │硬件抽象  │       │
│  └─────────┘   └──────────┘   └──────────┘   └──────────┘       │
│       │              │              │              │              │
│       ▼              ▼              ▼              ▼              │
│  ┌─────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐       │
│  │WebSocket│   │I2S/Opus  │   │Lamp/Fan  │   │Display   │       │
│  │ MQTT    │   │AFE/WakeW │   │STM32Ctrl │   │AudioCodec│       │
│  └─────────┘   └──────────┘   └──────────┘   └──────────┘       │
└──────────────────────────────────────────────────────────────────┘
                             │
                             │ UART
                             ▼
┌──────────────────────────────────────────────────────────────────┐
│                        STM32F103                                  │
│    传感器采集：心率 · 温湿度 · 姿态 · GPS · 烟雾                  │
└──────────────────────────────────────────────────────────────────┘
```

### 2.2 核心设计模式

#### 单例模式 (Singleton)

关键组件均采用单例，确保全局唯一实例：

```cpp
// application.h:35
class Application {
public:
    static Application& GetInstance() {
        static Application instance;
        return instance;
    }
};

// board.h:28
class Board {
public:
    static Board& GetInstance();  // 返回具体板型实例
};

// mcp_server.h
class McpServer {
public:
    static McpServer& GetInstance();
};
```

#### 工厂模式 (Factory)

Board通过宏实现板型注册：

```cpp
// board.h:85
#define DECLARE_BOARD(BOARD_CLASS_NAME) \
    void* create_board() { return new BOARD_CLASS_NAME(); }

// compact_wifi_board.cc:192
DECLARE_BOARD(CompactWifiBoard);
```

#### 观察者模式 (Observer)

Protocol的回调机制：

```cpp
// protocol.h:24-27
void OnIncomingAudio(std::function<void(AudioPacket)> callback);
void OnIncomingJson(std::function<void(const cJSON*)> callback);
```

### 2.3 设备状态机

```
                    ┌──────────────────┐
                    │  kDeviceStarting │
                    └────────┬─────────┘
                             │ WiFi连接成功
                             ▼
    ┌───────────────────────────────────────────────┐
    │                                               │
    │  ┌──────────┐  按键/唤醒词  ┌──────────────┐ │
    │  │  Idle    │ ───────────► │  Listening   │ │
    │  │  待机    │ ◄─────────── │   录音中     │ │
    │  └──────────┘   松开/静音  └──────────────┘ │
    │       ▲                           │          │
    │       │                           │ 发送完成 │
    │       │ TTS播放完成               ▼          │
    │  ┌──────────┐             ┌──────────────┐  │
    │  │ Speaking │ ◄────────── │  Thinking    │  │
    │  │  播放中  │   收到TTS   │   AI处理中   │  │
    │  └──────────┘             └──────────────┘  │
    │                                               │
    └───────────────────────────────────────────────┘
```

---

## 3. 核心模块分析

### 3.1 Application 应用层

**文件位置**: `main/application.h`, `main/application.cc`

Application是整个系统的调度中心，负责：

| 职责 | 说明 |
|------|------|
| 状态管理 | 维护设备状态 (Starting/Idle/Listening/Speaking/Thinking) |
| 事件分发 | 主事件循环，处理各类事件 |
| 子系统协调 | 调度Protocol、AudioService、Board协同工作 |
| MCP消息路由 | 转发MCP消息到Protocol层 |

**关键接口**:

```cpp
// application.h
class Application {
public:
    void Start();                    // 启动应用
    void MainEventLoop();            // 主事件循环
    DeviceState GetDeviceState();    // 获取设备状态
    void ToggleChatState();          // 切换对话状态
    void StartListening();           // 开始录音
    void StopListening();            // 停止录音
    void SendMcpMessage(const std::string& payload);  // 发送MCP消息
    void PlaySound(const std::string& name);          // 播放提示音
};
```

**启动流程**:

```
Application::Start()
    │
    ├── Board::GetInstance().StartNetwork()  // 启动WiFi
    │
    ├── Protocol::Start()                    // 连接云服务器
    │
    ├── AudioService::Start()                // 启动音频服务
    │
    └── MainEventLoop()                      // 进入主循环
```

### 3.2 Board 硬件抽象层

**文件位置**: `main/boards/common/board.h`

Board是硬件抽象层的核心，通过多态支持不同开发板：

```cpp
// board.h
class Board {
public:
    // 静态方法获取当前板型实例
    static Board& GetInstance();

    // 纯虚函数，子类必须实现
    virtual std::string GetBoardType() = 0;
    virtual AudioCodec* GetAudioCodec() = 0;
    virtual NetworkInterface* GetNetwork() = 0;
    virtual void StartNetwork() = 0;

    // 可选实现
    virtual Display* GetDisplay() { return new NoDisplay(); }
    virtual Led* GetLed() { return nullptr; }
};
```

**板型注册机制**:

```cpp
// compact_wifi_board.cc
class CompactWifiBoard : public WifiBoard {
    // 实现所有纯虚函数...
};

// 宏注册板型
DECLARE_BOARD(CompactWifiBoard);

// 展开后：
void* create_board() {
    return new CompactWifiBoard();
}
```

**支持的开发板** (boards目录):

| 目录名 | 描述 |
|--------|------|
| bread-compact-wifi | 面包板WiFi版 (本项目使用) |
| esp-box-3 | 乐鑫ESP-BOX-3 |
| korvo-1 | ESP32-Korvo开发板 |
| atoms3-echo | M5Stack ATOMS3 |

### 3.3 Protocol 通信协议层

**文件位置**: `main/protocols/protocol.h`

Protocol定义了与云服务器通信的抽象接口：

```cpp
// protocol.h
class Protocol {
public:
    // 注册回调
    void OnIncomingAudio(std::function<void(AudioPacket)> callback);
    void OnIncomingJson(std::function<void(const cJSON*)> callback);

    // 纯虚接口
    virtual bool Start() = 0;
    virtual bool OpenAudioChannel() = 0;
    virtual bool SendAudio(AudioPacket packet) = 0;
    virtual void CloseAudioChannel() = 0;

    // MCP消息发送
    virtual void SendMcpMessage(const std::string& message);
};
```

**两种实现**:

| 协议 | 类名 | 特点 |
|------|------|------|
| WebSocket | WebsocketProtocol | 全双工，实时性好 |
| MQTT | MqttProtocol | 可靠性高，支持断线重连 |

### 3.4 AudioService 音频服务层

**文件位置**: `main/audio/audio_service.h`

AudioService管理整个音频流水线：

```cpp
// audio_service.h
class AudioService {
public:
    void Start();                          // 启动服务
    void StartRecording();                 // 开始录音
    void StopRecording();                  // 停止录音
    void PlayAudio(AudioPacket packet);    // 播放音频
    bool IsRecording();                    // 是否在录音
    bool IsPlaying();                      // 是否在播放
};
```

**音频流水线**:

```
录音流程:
麦克风 → I2S采集 → AFE(降噪/增强) → 唤醒词检测 → Opus编码 → Protocol发送

播放流程:
Protocol接收 → Opus解码 → AudioCodec播放 → 扬声器
```

---

## 4. MCP工具系统

### 4.1 什么是MCP？

**MCP (Model Context Protocol)** 是小智AI实现"AI控制硬件"的核心机制。

**工作原理**:

```
用户语音: "打开灯"
    │
    ▼
┌─────────────────────────────────────────────────────┐
│                  小智云服务器                        │
│  1. ASR识别: "打开灯"                               │
│  2. LLM分析: 匹配工具 self.lamp.turn_on            │
│  3. 生成MCP调用: {"tool": "self.lamp.turn_on"}     │
└─────────────────────────────────────────────────────┘
    │
    │ WebSocket/MQTT
    ▼
┌─────────────────────────────────────────────────────┐
│                  ESP32 McpServer                    │
│  1. 解析JSON                                        │
│  2. 查找已注册工具                                  │
│  3. 执行回调函数                                    │
│  4. 返回执行结果                                    │
└─────────────────────────────────────────────────────┘
    │
    │ GPIO操作
    ▼
┌─────────────────────────────────────────────────────┐
│                    硬件设备                         │
│              灯亮了！                               │
└─────────────────────────────────────────────────────┘
```

### 4.2 McpServer核心实现

**文件位置**: `main/mcp_server.h`

```cpp
// mcp_server.h
class McpServer {
public:
    static McpServer& GetInstance();

    // 注册工具
    void AddTool(
        const std::string& name,           // 工具名称
        const std::string& description,    // 工具描述（LLM用于理解意图）
        const PropertyList& properties,    // 参数列表
        std::function<ReturnValue(const PropertyList&)> callback  // 执行回调
    );

    // 处理MCP消息
    void HandleMessage(const cJSON* json);
};
```

### 4.3 工具注册示例

**灯控制器** (`lamp_controller.h:28-55`):

```cpp
// 构造函数中注册MCP工具
LampController(gpio_num_t gpio_num) : gpio_num_(gpio_num) {
    // 初始化GPIO...

    auto& mcp_server = McpServer::GetInstance();

    // 注册获取状态工具
    mcp_server.AddTool("self.lamp.get_state",
        "Get the power state of the lamp/light.\n"
        "MUST call when user asks: '灯开了吗', '灯的状态', 'is light on'",
        PropertyList(),
        [this](const PropertyList& properties) -> ReturnValue {
            return power_ ? "{\"power\": true}" : "{\"power\": false}";
        });

    // 注册开灯工具
    mcp_server.AddTool("self.lamp.turn_on",
        "Turn on the lamp/light.\n"
        "MUST call when user says: '开灯', '打开灯', '把灯打开', 'turn on light'",
        PropertyList(),
        [this](const PropertyList& properties) -> ReturnValue {
            power_ = true;
            gpio_set_level(gpio_num_, 1);
            return true;
        });

    // 注册关灯工具
    mcp_server.AddTool("self.lamp.turn_off",
        "Turn off the lamp/light.\n"
        "MUST call when user says: '关灯', '关闭灯', '把灯关掉', 'turn off light'",
        PropertyList(),
        [this](const PropertyList& properties) -> ReturnValue {
            power_ = false;
            gpio_set_level(gpio_num_, 0);
            return true;
        });
}
```

### 4.4 当前已注册的MCP工具

| 控制器 | 工具名称 | 功能 |
|--------|----------|------|
| LampController | self.lamp.get_state | 获取灯状态 |
| | self.lamp.turn_on | 开灯 |
| | self.lamp.turn_off | 关灯 |
| FanController | self.fan.get_state | 获取风扇状态 |
| | self.fan.turn_on | 开风扇 |
| | self.fan.turn_off | 关风扇 |
| STM32Controller | self.stm32.heart_rate.* | 心率传感器控制 |
| | self.stm32.temperature.* | 温湿度传感器控制 |
| | self.stm32.mpu6050.* | 姿态传感器控制 |
| | self.stm32.mq2.* | 烟雾传感器控制 |
| | self.stm32.gps.* | GPS定位控制 |

### 4.5 工具描述的重要性

**description字段是让AI理解用户意图的关键！**

❌ **错误示例**（AI难以理解中文命令）:
```cpp
mcp_server.AddTool("self.lamp.turn_on",
    "Turn on the lamp",  // 只有英文
    ...);
```

✅ **正确示例**（中英文关键词）:
```cpp
mcp_server.AddTool("self.lamp.turn_on",
    "Turn on the lamp/light.\n"
    "MUST call when user says: '开灯', '打开灯', '把灯打开', 'turn on light'",
    ...);
```

---

## 5. 音频处理流水线

### 5.1 整体流程

```
┌─────────────────────────────────────────────────────────────────┐
│                        录音流程                                  │
│                                                                 │
│  ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐     │
│  │ 麦克风   │ ─► │   I2S   │ ─► │   AFE   │ ─► │ 唤醒词  │     │
│  │ INMP441 │    │ 采集    │    │降噪增强 │    │ 检测    │     │
│  └─────────┘    └─────────┘    └─────────┘    └─────────┘     │
│                                                      │          │
│                                              唤醒后  │          │
│                                                      ▼          │
│                                               ┌─────────┐      │
│                                               │  Opus   │      │
│                                               │  编码   │      │
│                                               └─────────┘      │
│                                                      │          │
│                                                      ▼          │
│                                               ┌─────────┐      │
│                                               │Protocol │      │
│                                               │ 发送    │      │
│                                               └─────────┘      │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                        播放流程                                  │
│                                                                 │
│  ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐     │
│  │Protocol │ ─► │  Opus   │ ─► │ Audio   │ ─► │ 扬声器  │     │
│  │ 接收    │    │  解码   │    │ Codec   │    │ MAX98357│     │
│  └─────────┘    └─────────┘    └─────────┘    └─────────┘     │
└─────────────────────────────────────────────────────────────────┘
```

### 5.2 关键技术点

| 技术 | 说明 |
|------|------|
| I2S | 数字音频接口，连接麦克风和功放 |
| AFE | 音频前端处理，包括降噪、增益控制、回声消除 |
| 唤醒词 | 本地检测"小智小智"，无需联网 |
| Opus | 高压缩比音频编码，节省带宽 |

### 5.3 音频采样参数

```cpp
// config.h
#define AUDIO_INPUT_SAMPLE_RATE  16000   // 录音采样率 16kHz
#define AUDIO_OUTPUT_SAMPLE_RATE 24000   // 播放采样率 24kHz
```

---

## 6. 与STM32协同工作

### 6.1 通信架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        ESP32-S3                                  │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐       │
│  │ STM32       │     │  MCP工具    │     │  华为云IoT  │       │
│  │ Controller  │ ──► │  注册中心   │     │  上报模块   │       │
│  └─────────────┘     └─────────────┘     └─────────────┘       │
│         │                   │                   ▲               │
│         │ UART              │                   │               │
│         │ 115200bps         │                   │ 同步数据      │
│         ▼                   ▼                   │               │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                   数据处理逻辑                           │   │
│  │  - 解析STM32上报的传感器数据                            │   │
│  │  - 更新内部状态变量                                      │   │
│  │  - 同步到华为云IoT平台                                   │   │
│  │  - 处理紧急事件（摔倒、碰撞、烟雾）                     │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
         │ UART (GPIO17-TX, GPIO18-RX)
         ▼
┌─────────────────────────────────────────────────────────────────┐
│                        STM32F103                                 │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐  │
│  │ MAX30102│ │  DHT11  │ │ MPU6050 │ │   MQ2   │ │   GPS   │  │
│  │ 心率血氧│ │ 温湿度  │ │ 姿态    │ │ 烟雾    │ │ 定位    │  │
│  └─────────┘ └─────────┘ └─────────┘ └─────────┘ └─────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### 6.2 UART通信协议

**ESP32 → STM32 (命令)**:
```
{CMD:HR_START}\r\n      // 启动心率检测
{CMD:HR_STOP}\r\n       // 停止心率检测
{CMD:TEMP_START}\r\n    // 启动温度检测
{CMD:MPU_START}\r\n     // 启动姿态检测
{CMD:MQ2_START}\r\n     // 启动烟雾检测
{CMD:GPS_START}\r\n     // 启动GPS定位
```

**STM32 → ESP32 (数据)**:
```
{DATA:HR:75,SPO2:98}           // 心率75bpm, 血氧98%
{DATA:TEMP:25,60}              // 温度25°C, 湿度60%
{DATA:MPU:1.2,-0.5,180.3}      // Pitch, Roll, Yaw
{DATA:MQ2:150.5,0}             // PPM浓度, 报警级别
{DATA:GPS:39.908,116.397,8}    // 纬度,经度,卫星数

{STATUS:OK}                    // 命令执行成功
{STATUS:ALREADY_ON}            // 传感器已开启
{SENSOR:HR:ON}                 // 按键开启心率
{EVENT:FALL}                   // 摔倒事件
{EVENT:COLLISION}              // 碰撞事件
{EVENT:SMOKE}                  // 烟雾事件
```

### 6.3 数据处理流程

**`stm32_controller.h:124-303`**:

```cpp
void ProcessResponse(const char* response) {
    // 1. 解析心率数据
    if (strstr(response, "{DATA:HR:")) {
        int hr, spo2;
        sscanf(pos, "{DATA:HR:%d,SPO2:%d}", &hr, &spo2);
        latest_heart_rate_ = hr;
        latest_spo2_ = spo2;
        // 同步到华为云
        if (huawei_iot_) huawei_iot_->UpdateHeartRate(hr, spo2);
    }

    // 2. 处理紧急事件
    else if (strstr(response, "{EVENT:FALL}")) {
        // 显示警告
        display->ShowNotification("⚠️ 摔倒警告！");
        // 播放警报音
        app.PlaySound("alert");
        // 上报华为云
        huawei_iot_->SetFallFlag(1);
        huawei_iot_->ReportNow();
    }
}
```

### 6.4 语音控制传感器

用户可以通过语音控制STM32传感器：

| 语音指令 | MCP工具 | 动作 |
|----------|---------|------|
| "打开心率" | self.stm32.heart_rate.start | 发送 {CMD:HR_START} |
| "关闭心率" | self.stm32.heart_rate.stop | 发送 {CMD:HR_STOP} |
| "心率多少" | self.stm32.heart_rate.get_state | 返回最新数据 |
| "打开GPS" | self.stm32.gps.start | 发送 {CMD:GPS_START} |

---

## 7. 总结

### 7.1 架构优点

1. **模块化设计**: 各模块职责清晰，耦合度低
2. **多板型支持**: 通过Board抽象层支持多种开发板
3. **可扩展性**: MCP工具系统方便添加新硬件控制
4. **实时性**: FreeRTOS任务调度，音频流畅

### 7.2 数据流总结

```
语音输入 → ASR → LLM → MCP调用 → 硬件控制
                  ↓
            TTS → 语音输出

传感器 → STM32 → UART → ESP32 → 华为云
                          ↓
                     MCP查询接口
```

### 7.3 关键文件索引

| 文件 | 作用 |
|------|------|
| `application.h/cc` | 应用主入口 |
| `board.h` | 硬件抽象基类 |
| `protocol.h` | 通信协议基类 |
| `mcp_server.h` | MCP工具注册中心 |
| `audio_service.h` | 音频服务 |
| `stm32_controller.h` | STM32传感器控制 |
| `lamp_controller.h` | 灯控制器 |
| `fan_controller.h` | 风扇控制器 |
| `huaweicloud_iot.h` | 华为云IoT模块 |

---

> 文档生成时间: 2025年1月
> 作者: Claude Code 分析助手

