# MQ2气体传感器 FreeRTOS 移植技术文档

> 更新：2026-01-04 | 作者：Claude Code

---

## 1. 概述

本文档详细记录 MQ2 气体传感器在 STM32F103RCT6 + FreeRTOS 环境下的移植过程，包括硬件配置、软件架构、IPC 机制、内存管理和常见问题解决方案。

### 1.1 MQ2 传感器特性

- **检测气体：** LPG、丙烷、氢气、甲烷、烟雾
- **输出信号：** 模拟电压（0-5V，需分压或限压）
- **预热时间：** 约 20 秒
- **工作电压：** 5V（传感器供电）

### 1.2 移植目标

- ADC DMA 循环采样，不阻塞 CPU
- 均值滤波降低噪声
- PPM 浓度计算
- 多级报警判断
- FreeRTOS 任务封装，支持挂起/恢复
- 线程安全的数据共享

---

## 2. 硬件配置

### 2.1 引脚定义

| 引脚 | 功能 | 说明 |
|------|------|------|
| PA0 | ADC1_IN0 | MQ2 模拟输出 |

### 2.2 CubeMX 配置

#### ADC1 配置
```
Mode: IN0 (Single Channel)
Resolution: 12 bits
Scan Conversion Mode: Disabled
Continuous Conversion: Enabled（连续转换）
DMA Continuous Requests: Enabled（DMA循环请求）
```

#### DMA 配置
```
DMA Request: ADC1
Channel: DMA1_Channel1
Direction: Peripheral to Memory
Mode: Circular（循环模式，关键！）
Data Width: Word (32-bit)
```

### 2.3 电路参数

```c
#define MQ2_VCC                 5.0f    // 传感器供电电压
#define MQ2_VREF                3.3f    // ADC参考电压
#define MQ2_ADC_RESOLUTION      4095.0f // 12位ADC
#define MQ2_RL_VALUE            4700.0f // 负载电阻 (Ω)
#define MQ2_R0_VALUE            3000.0f // 清洁空气中Rs值 (需校准)
```

---

## 3. 软件架构

### 3.1 分层设计

```
┌─────────────────────────────────────┐
│         应用层 (key_task.c)          │
│    按键控制启停、界面联动              │
├─────────────────────────────────────┤
│         任务层 (mq2_task.c)          │
│    FreeRTOS任务、IPC、数据共享         │
├─────────────────────────────────────┤
│        驱动层 (mq2.c/h)              │
│    ADC采集、PPM计算、报警判断          │
├─────────────────────────────────────┤
│         HAL层 (adc.c)                │
│    CubeMX生成的ADC DMA配置            │
└─────────────────────────────────────┘
```

### 3.2 文件结构

```
Core/task/
├── mq2_task.h          # 任务接口
├── mq2_task.c          # 任务实现
└── driver/
    ├── mq2.h           # 驱动头文件（类型定义、宏、接口）
    └── mq2.c           # 驱动实现
```

---

## 4. 驱动层实现

### 4.1 数据类型定义 (mq2.h)

```c
// 报警等级枚举
typedef enum {
    MQ2_ALARM_SAFE = 0,     // 安全 (<100 ppm)
    MQ2_ALARM_LOW,          // 低级 (100-300 ppm)
    MQ2_ALARM_MID,          // 中级 (300-500 ppm)
    MQ2_ALARM_HIGH,         // 高级 (500-800 ppm)
    MQ2_ALARM_DANGER        // 危险 (>800 ppm)
} MQ2_AlarmLevel_t;

// 传感器数据结构
typedef struct {
    uint32_t adc_raw;           // ADC原始值
    float voltage;              // 电压 (V)
    float rs;                   // 传感器电阻 (Ω)
    float ratio;                // Rs/R0比值
    float ppm;                  // 气体浓度 (ppm)
    MQ2_AlarmLevel_t alarm;     // 报警等级
} MQ2_Data_t;
```

### 4.2 计算宏

```c
// ADC → 电压
#define MQ2_ADC_TO_VOLTAGE(adc) ((float)(adc) * (MQ2_VREF / MQ2_ADC_RESOLUTION))

// 电压 → 传感器电阻Rs
#define MQ2_VOLTAGE_TO_RS(v)    ((MQ2_RL_VALUE * (MQ2_VCC - (v))) / (v))

// Rs → Rs/R0比值
#define MQ2_RS_TO_RATIO(rs)     ((float)(rs) / MQ2_R0_VALUE)

// 比值 → PPM（LPG曲线：A=270, B=-0.84）
#define MQ2_RATIO_TO_PPM(ratio) (powf((ratio) / 270.0f, 1.0f / (-0.84f)))
```

### 4.3 ADC DMA 初始化

```c
static uint32_t mq2_adc_buffer[MQ2_SAMPLE_COUNT];  // DMA缓冲区

void MQ2_Init(void) {
    // 启动ADC DMA循环采样
    // DMA会自动将ADC数据填充到缓冲区，无需CPU干预
    HAL_ADC_Start_DMA(&hadc1, mq2_adc_buffer, MQ2_SAMPLE_COUNT);
}
```

### 4.4 均值滤波

```c
static uint32_t MQ2_ReadADC_Average(void) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < MQ2_SAMPLE_COUNT; i++) {
        sum += mq2_adc_buffer[i];  // 从DMA缓冲区读取
    }
    return (sum / MQ2_SAMPLE_COUNT);
}
```

---

## 5. 任务层实现

### 5.1 IPC 机制

```c
// 共享数据（Mutex保护）
static struct {
    float ppm;
    MQ2_AlarmLevel_t alarm;
    uint8_t is_valid;
} mq2_shared_data;

// IPC对象
static SemaphoreHandle_t mq2_mutex = NULL;      // 互斥锁
static TaskHandle_t mq2_task_handle = NULL;     // 任务句柄
static volatile uint8_t mq2_running = 0;        // 运行状态标志
```

### 5.2 任务函数

```c
static void mq2_task(void *arg) {
    MQ2_Data_t data;

    // 错峰启动
    vTaskDelay(pdMS_TO_TICKS(300));

    // 初始化硬件
    MQ2_Init();
    vTaskDelay(pdMS_TO_TICKS(500));  // ADC稳定

    for (;;) {
        // 检查运行状态
        if (!mq2_running) {
            vTaskSuspend(NULL);  // 挂起自己
            continue;
        }

        // 更新数据
        MQ2_Update(&data);

        // Mutex保护更新共享数据
        xSemaphoreTake(mq2_mutex, portMAX_DELAY);
        mq2_shared_data.ppm = data.ppm;
        mq2_shared_data.alarm = data.alarm;
        mq2_shared_data.is_valid = 1;
        xSemaphoreGive(mq2_mutex);

        // 通知OLED更新
        oled_update_mq2(data.ppm, data.alarm, 1);

        // 高报警时LED快闪
        if (data.alarm >= MQ2_ALARM_HIGH) {
            led_set_mode(LED_EVENT_ERROR);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // 1秒采样周期
    }
}
```

### 5.3 任务控制接口

```c
void mq2_task_start(void) {
    if (mq2_task_handle && !mq2_running) {
        mq2_running = 1;
        vTaskResume(mq2_task_handle);  // 恢复任务
    }
}

void mq2_task_stop(void) {
    if (mq2_running) {
        mq2_running = 0;  // 设置标志，任务会自行挂起
    }
}

uint8_t mq2_is_running(void) {
    return mq2_running;
}
```

### 5.4 线程安全数据获取

```c
void mq2_get_data(float *ppm, MQ2_AlarmLevel_t *alarm, uint8_t *valid) {
    if (!mq2_mutex) return;

    xSemaphoreTake(mq2_mutex, portMAX_DELAY);
    *ppm = mq2_shared_data.ppm;
    *alarm = mq2_shared_data.alarm;
    *valid = mq2_shared_data.is_valid;
    xSemaphoreGive(mq2_mutex);
}
```

---

## 6. 内存管理

### 6.1 任务栈配置

```c
// MQ2任务栈 384 字节（比其他任务大）
xTaskCreate(mq2_task, "mq2", 384, NULL, 2, &mq2_task_handle);
```

**为什么需要更大的栈？**
- 浮点运算（powf、除法）需要更多栈空间
- MQ2_Data_t 结构体占用 24+ 字节
- 函数调用链较深

### 6.2 DMA 缓冲区

```c
#define MQ2_SAMPLE_COUNT  30  // 30个采样点

static uint32_t mq2_adc_buffer[MQ2_SAMPLE_COUNT];  // 120字节（静态分配）
```

**注意：** DMA 缓冲区必须是全局或静态变量，不能在栈上分配！

### 6.3 堆内存使用

```
Mutex: ~80 字节
Task TCB: ~88 字节
Task Stack: 384 × 4 = 1536 字节
总计: ~1.7 KB
```

---

## 7. 资源冲突处理

### 7.1 ADC 资源

**问题：** 多个传感器共用 ADC 外设

**解决方案：**
1. **专用通道：** MQ2 使用 ADC1_IN0，不与其他通道冲突
2. **DMA 独占：** 每个 ADC 通道配置独立的 DMA 请求
3. **如需多通道：** 使用扫描模式 + 多缓冲区

### 7.2 头文件循环依赖

**问题：** `mq2_task.h` 需要 `MQ2_AlarmLevel_t`，但类型在 `mq2.h` 中定义

**现象：**
```
error: unknown type name 'MQ2_AlarmLevel_t'
```

**解决方案：**

1. **调整 bsp_system.h 包含顺序：**
```c
// driver 头文件在前（提供类型定义）
#include "task/driver/mq2.h"

// task 头文件在后（使用类型定义）
#include "task/mq2_task.h"
```

2. **mq2.h 不包含 bsp_system.h：**
```c
// 只包含必要的HAL头文件
#include "main.h"
#include "adc.h"
#include <stdint.h>
#include <math.h>
```

### 7.3 任务同步

**问题：** 多任务访问共享数据

**解决方案：** Mutex 互斥锁

```c
// 写入方（mq2_task）
xSemaphoreTake(mq2_mutex, portMAX_DELAY);
mq2_shared_data.ppm = data.ppm;
xSemaphoreGive(mq2_mutex);

// 读取方（oled_task/其他任务）
xSemaphoreTake(mq2_mutex, portMAX_DELAY);
float ppm = mq2_shared_data.ppm;
xSemaphoreGive(mq2_mutex);
```

---

## 8. OLED 界面集成

### 8.1 界面枚举

```c
typedef enum {
    UI_PAGE_WELCOME = 0,
    UI_PAGE_DHT11,
    UI_PAGE_MQ2,
    UI_PAGE_COUNT
} ui_page_t;
```

### 8.2 更新接口

```c
void oled_update_mq2(float ppm, MQ2_AlarmLevel_t alarm, uint8_t valid);
```

### 8.3 按键联动

```c
// KEY1：根据当前界面控制对应传感器
if (events & KEY_EVENT_KEY1_PRESS) {
    ui_page_t page = oled_get_current_page();
    if (page == UI_PAGE_MQ2) {
        if (mq2_is_running()) {
            mq2_task_stop();
        } else {
            mq2_task_start();
        }
    }
}

// KEY2：切换界面
if (events & KEY_EVENT_KEY2_PRESS) {
    oled_switch_page();
}
```

---

## 9. 调试技巧

### 9.1 串口日志

```c
uart_printf_dma(&huart1, "[MQ2] PPM=%.1f Alarm=%d\r\n", data.ppm, data.alarm);
```

### 9.2 ADC 原始值检查

```c
uart_printf_dma(&huart1, "[MQ2] ADC=%lu V=%.2f\r\n", data.adc_raw, data.voltage);
```

### 9.3 R0 校准

在清洁空气中测量 Rs，作为 R0 基准值：

```c
// 校准模式：打印 Rs 值，取稳定后的平均值作为 R0
uart_printf_dma(&huart1, "[MQ2 CAL] Rs=%.0f\r\n", data.rs);
```

---

## 10. 常见问题

### Q1: PPM 值一直为 0 或异常

**可能原因：**
1. ADC 未启动或 DMA 配置错误
2. 传感器未预热（需 20 秒以上）
3. R0 校准值不正确

**检查方法：**
```c
uart_printf_dma(&huart1, "ADC=%lu V=%.2f Rs=%.0f\r\n",
                data.adc_raw, data.voltage, data.rs);
```

### Q2: 编译报错 `unknown type name 'MQ2_AlarmLevel_t'`

**原因：** 头文件循环依赖

**解决：** 调整 bsp_system.h 中的包含顺序，driver 在 task 之前

### Q3: 任务栈溢出

**现象：** 进入 `vApplicationStackOverflowHook`

**解决：** 增大 MQ2 任务栈（至少 384 字节）

### Q4: DMA 缓冲区数据不更新

**可能原因：**
1. DMA 模式未配置为 Circular
2. `HAL_ADC_Start_DMA()` 未调用
3. 缓冲区在栈上分配（被覆盖）

---

## 11. 总结

### 移植要点

1. **ADC DMA 循环模式：** 后台自动采样，不阻塞 CPU
2. **Mutex 保护共享数据：** 确保多任务访问安全
3. **头文件包含顺序：** driver 在 task 之前，避免循环依赖
4. **任务栈空间：** 浮点运算需要更大栈（384+ 字节）
5. **错峰启动：** 避免多任务同时初始化竞争资源

### 架构优势

- **解耦设计：** 驱动层与任务层分离
- **灵活控制：** 支持挂起/恢复
- **资源安全：** IPC 机制保护共享资源
- **易于扩展：** 可添加更多传感器任务
