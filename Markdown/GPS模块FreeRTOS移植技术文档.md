# GPS模块FreeRTOS移植技术文档

> 更新：2026-01-06 | 模块：ATGM336H GPS | 通信：UART2 DMA

---

## 一、模块概述

### 1.1 硬件信息

| 参数 | 值 |
|------|-----|
| 模块型号 | ATGM336H（北斗/GPS双模） |
| 通信接口 | UART（TTL电平） |
| 波特率 | 9600bps（默认） |
| 数据格式 | NMEA-0183协议 |
| 工作电压 | 3.3V |
| 定位精度 | 2.5m CEP |
| 冷启动时间 | 约30秒 |

### 1.2 引脚连接

```
GPS模块          STM32F103
  TX    ───────>  PA3 (USART2_RX)
  RX    <───────  PA2 (USART2_TX)  [可选，用于配置]
  VCC   ───────>  3.3V
  GND   ───────>  GND
```

---

## 二、NMEA协议解析原理

### 2.1 NMEA-0183协议简介

GPS模块输出的是NMEA-0183标准协议数据，这是一种ASCII文本格式的协议。每条语句以`$`开头，以`\r\n`结尾，字段之间用逗号分隔。

常见语句类型：
- `$GPGGA` / `$GNGGA`：定位信息（最重要）
- `$GPRMC` / `$GNRMC`：推荐最小定位信息
- `$GPGSV` / `$GNGSV`：可见卫星信息
- `$GPGSA` / `$GNGSA`：卫星精度信息

> **注意**：`GP`前缀表示GPS卫星，`GN`前缀表示GPS+北斗混合定位，`BD`前缀表示北斗卫星。

### 2.2 GGA语句格式详解

本项目主要解析GGA语句，格式如下：

```
$GNGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
  │       │        │    │    │     │ │  │   │   │  │  │  │  │    │
  │       │        │    │    │     │ │  │   │   │  │  │  │  │    └─ 校验和
  │       │        │    │    │     │ │  │   │   │  │  │  │  └─ 差分站ID
  │       │        │    │    │     │ │  │   │   │  │  │  └─ 差分数据龄期
  │       │        │    │    │     │ │  │   │   │  │  └─ 大地水准面高度单位
  │       │        │    │    │     │ │  │   │   │  └─ 大地水准面高度
  │       │        │    │    │     │ │  │   │   └─ 海拔单位(M=米)
  │       │        │    │    │     │ │  │   └─ 海拔高度
  │       │        │    │    │     │ │  └─ HDOP水平精度因子
  │       │        │    │    │     │ └─ 可见卫星数
  │       │        │    │    │     └─ 定位质量(0=无效,1=GPS,2=DGPS)
  │       │        │    │    └─ 经度方向(E/W)
  │       │        │    └─ 经度(dddmm.mmmm格式)
  │       │        └─ 纬度方向(N/S)
  │       └─ 纬度(ddmm.mmmm格式)
  └─ UTC时间(hhmmss.ss)
```

### 2.3 坐标格式转换

**关键难点：NMEA坐标格式转换为十进制度数**

GPS模块输出的坐标格式是`度分`格式（ddmm.mmmm），需要转换为常用的`十进制度数`格式：

```
NMEA格式：3014.5678 (表示30度14.5678分)
转换公式：度数 = 整数部分的度 + (分/60)
         = 30 + (14.5678 / 60)
         = 30.242796°
```

**代码实现：**
```c
// 纬度转换（ddmm.mmmm → 十进制度数）
double raw_lat = atof(fields[2]);           // 例如：3014.5678
int lat_deg = (int)(raw_lat / 100);         // 取度：30
double lat_min = raw_lat - (lat_deg * 100); // 取分：14.5678
data->latitude = lat_deg + (lat_min / 60.0f); // 转换：30.242796

// 经度转换（dddmm.mmmm → 十进制度数）
double raw_lon = atof(fields[4]);           // 例如：12012.3456
int lon_deg = (int)(raw_lon / 100);         // 取度：120
double lon_min = raw_lon - (lon_deg * 100); // 取分：12.3456
data->longitude = lon_deg + (lon_min / 60.0f); // 转换：120.205760

// 方向处理
if (data->lat_dir == 'S') data->latitude = -data->latitude;   // 南纬为负
if (data->lon_dir == 'W') data->longitude = -data->longitude; // 西经为负
```

---

## 三、数据接收架构

### 3.1 DMA + 空闲中断接收

GPS模块持续输出数据（约1Hz），使用DMA+空闲中断可以高效接收不定长数据：

```
┌─────────────────────────────────────────────────────────────┐
│                      数据流架构                              │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  GPS模块 ──UART2──> DMA缓冲区 ──空闲中断──> 环形缓冲区      │
│                     (256B)                  (256B)          │
│                                                │            │
│                                                v            │
│                                          GPS任务解析        │
│                                          (100ms周期)        │
│                                                │            │
│                                                v            │
│                                          共享数据结构        │
│                                          (Mutex保护)        │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 环形缓冲区设计

```c
// 环形缓冲区
static uint8_t gps_rx_buffer[GPS_RX_BUFFER_SIZE];  // 256字节
static volatile uint16_t rx_write_pos = 0;          // 写指针（ISR修改）
static volatile uint16_t rx_read_pos = 0;           // 读指针（任务修改）

// 计算可用数据量
static uint16_t ringbuf_available(void) {
    return (rx_write_pos - rx_read_pos + GPS_RX_BUFFER_SIZE) % GPS_RX_BUFFER_SIZE;
}

// UART接收回调（由DMA空闲中断触发）
void gps_uart_rx_callback(uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        gps_rx_buffer[rx_write_pos] = data[i];
        rx_write_pos = (rx_write_pos + 1) % GPS_RX_BUFFER_SIZE;
    }
}
```

### 3.3 NMEA语句解析流程

```c
static void parse_nmea_buffer(const char *buffer, uint16_t len) {
    // 1. 查找GGA语句起始位置
    const char *start = strstr(buffer, "$GNGGA");
    if (!start) start = strstr(buffer, "$GPGGA");
    if (!start) return;

    // 2. 查找语句结束位置
    const char *end = strchr(start, '\r');
    if (!end) end = strchr(start, '*');
    if (!end || end <= start) return;

    // 3. 提取完整语句
    char sentence[128];
    int sentence_len = end - start;
    strncpy(sentence, start, sentence_len);
    sentence[sentence_len] = '\0';

    // 4. 解析GGA语句
    GPS_Data_t temp_data = {0};
    if (parse_gga(sentence, &temp_data)) {
        // 5. 更新共享数据（Mutex保护）
        xSemaphoreTake(gps_mutex, portMAX_DELAY);
        memcpy(&gps_shared_data, &temp_data, sizeof(GPS_Data_t));
        xSemaphoreGive(gps_mutex);
    }
}
```

---

## 四、FreeRTOS任务设计

### 4.1 任务参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 任务名 | "gps" | |
| 栈大小 | 512字 (2KB) | 字符串解析需要较大栈 |
| 优先级 | 2 | 与其他传感器任务相同 |
| 解析周期 | 100ms | 足够处理1Hz GPS数据 |
| 打印周期 | 2000ms | 避免串口输出过多 |

### 4.2 任务状态机

```
┌─────────┐    gps_task_start()    ┌─────────┐
│  INIT   │ ─────────────────────> │ RUNNING │
│(挂起态) │                        │(解析中) │
└─────────┘ <───────────────────── └─────────┘
               gps_task_stop()
```

### 4.3 数据结构定义

```c
typedef struct {
    // 位置信息
    float latitude;             // 纬度（十进制度数，正=北，负=南）
    float longitude;            // 经度（十进制度数，正=东，负=西）
    char lat_dir;               // 纬度方向 'N'/'S'
    char lon_dir;               // 经度方向 'E'/'W'
    float altitude;             // 海拔高度 (m)

    // 时间信息
    char utc_time[12];          // UTC 时间 "HH:MM:SS.SS"

    // 定位状态
    uint8_t fix_quality;        // 定位质量 (0=无效, 1=GPS, 2=DGPS)
    uint8_t satellites;         // 可见卫星数
    float hdop;                 // 水平精度因子

    // 有效标志
    uint8_t is_valid;           // 数据有效标志
    uint8_t is_fixed;           // 是否已定位
} GPS_Data_t;
```

---

## 五、遇到的困难与解决方案

### 5.1 问题1：UART2 DMA配置

**现象**：GPS数据无法接收，DMA回调不触发

**原因**：CubeMX生成的代码中UART2未配置DMA

**解决**：
1. CubeMX中为USART2添加DMA通道（RX使用DMA1_Channel6）
2. 配置为循环模式（Circular）
3. 在usart.c中添加空闲中断回调处理

```c
// usart.c 中添加
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART2) {
        // GPS数据处理
        gps_uart_rx_callback(&gps_dma_buffer[gps_last_rx_pos], len);
        gps_last_rx_pos = Size;
    }
}
```

### 5.2 问题2：坐标解析错误

**现象**：解析出的经纬度值明显错误（如纬度>90°）

**原因**：直接将NMEA格式当作十进制度数使用

**解决**：正确实现度分格式到十进制度数的转换（见2.3节）

### 5.3 问题3：室内无法定位

**现象**：`fix_quality=0`，`satellites=0`，始终显示"Searching..."

**原因**：GPS信号无法穿透建筑物

**解决**：
1. 将GPS天线移至窗边或室外
2. 首次定位（冷启动）需要30秒以上
3. 代码中正确处理未定位状态，避免显示无效坐标

### 5.4 问题4：数据解析不完整

**现象**：偶尔解析失败，部分语句被截断

**原因**：NMEA语句可能跨越两次DMA传输

**解决**：
1. 使用环形缓冲区累积数据
2. 解析时查找完整的`$....\r\n`语句
3. 不完整的数据留待下次处理

### 5.5 问题5：浮点数精度不足

**现象**：经纬度小数位数不够，定位精度下降

**原因**：`float`类型只有6-7位有效数字

**解决**：
1. 解析时使用`double`类型进行计算
2. 存储时使用`float`（节省内存，精度足够显示）
3. 发送给ESP32时保留6位小数

---

## 六、API接口说明

### 6.1 初始化与控制

```c
void gps_task_init(void);      // 初始化GPS任务（默认挂起）
void gps_task_start(void);     // 启动GPS任务，开始接收数据
void gps_task_stop(void);      // 停止GPS任务，关闭DMA接收
uint8_t gps_is_running(void);  // 查询运行状态
```

### 6.2 数据获取

```c
void gps_get_data(GPS_Data_t *data);                    // 获取完整数据
void gps_get_position(float *lat, float *lon, uint8_t *valid);  // 获取位置
uint8_t gps_get_satellites(void);                       // 获取卫星数
GPS_Status_t gps_get_status(void);                      // 获取状态
uint8_t gps_is_fixed(void);                             // 是否已定位
```

### 6.3 回调函数

```c
// 由usart.c中的DMA空闲中断调用
void gps_uart_rx_callback(uint8_t *data, uint16_t len);
```

---

## 七、与ESP32通信

GPS数据通过ESP32任务发送给小智AI：

```c
// 数据格式
{DATA:GPS:纬度,经度,卫星数}\r\n

// 示例
{DATA:GPS:30.242796,120.205760,8}\r\n

// 发送函数
void esp32_send_gps(float lat, float lon, uint8_t satellites);
```

---

## 八、调试技巧

### 8.1 串口监控原始数据

```c
// 在gps_uart_rx_callback中添加调试输出
uart_printf_dma(&huart1, "[GPS RAW] %.*s\r\n", len, data);
```

### 8.2 检查定位状态

```
正常输出示例：
[GPS] Searching... Satellites:0      ← 搜索中
[GPS] Searching... Satellites:4      ← 找到卫星但未定位
[GPS] Fixed! Lat:30.242796N Lon:120.205760E Sat:8 Alt:15.2m  ← 定位成功
```

### 8.3 常见NMEA语句示例

```
未定位：
$GNGGA,123519.00,,,,,0,00,99.99,,,,,,*7E

已定位：
$GNGGA,123519.00,3014.5678,N,12012.3456,E,1,08,0.9,15.2,M,0.0,M,,*47
```

---

## 九、文件清单

```
Core/task/
├── gps_task.c          # GPS任务实现
├── gps_task.h          # GPS任务接口
Core/Src/
├── usart.c             # UART2 DMA配置及回调
```

---

## 十、参考资料

1. NMEA-0183协议标准
2. ATGM336H模块数据手册
3. STM32F1 HAL库UART DMA使用指南
