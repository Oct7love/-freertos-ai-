# MAX30102 心率血氧传感器 FreeRTOS 移植技术文档

> 更新：2026-01-04 | 作者：Claude + User | 项目：Smart Helmet FreeRTOS

---

## 目录

1. [传感器概述](#1-传感器概述)
2. [硬件配置](#2-硬件配置)
3. [软件架构](#3-软件架构)
4. [IPC机制与资源分配](#4-ipc机制与资源分配)
5. [内存分配](#5-内存分配)
6. [驱动实现](#6-驱动实现)
7. [心率血氧算法](#7-心率血氧算法)
8. [调试问题与解决方案](#8-调试问题与解决方案)
9. [API参考](#9-api参考)
10. [附录：完整代码](#10-附录完整代码)

---

## 1. 传感器概述

### 1.1 MAX30102 简介

MAX30102 是 Maxim Integrated 生产的集成脉搏血氧仪和心率监测传感器模块，内置：

| 组件 | 说明 |
|------|------|
| 红光 LED | 波长 660nm，用于血氧检测 |
| 红外 LED | 波长 880nm，用于心率检测 |
| 光电探测器 | 高灵敏度光电二极管 |
| 18位 ADC | 高精度模数转换 |
| 环境光抑制 | 降低外界光干扰 |
| 32深度 FIFO | 数据缓冲 |

### 1.2 工作原理

**光电容积描记法（PPG）：**
1. LED 发射光穿透皮肤到达血管
2. 血液吸收部分光（血红蛋白特性）
3. 光电探测器检测反射光强度
4. 心跳引起血容量周期性变化 → 反射光周期性变化

**心率检测：** 检测 PPG 信号的峰值间隔

**血氧检测：** 氧合血红蛋白和脱氧血红蛋白对红光/红外光的吸收率不同
```
SpO2 = f(AC_red/DC_red) / (AC_ir/DC_ir)
```

### 1.3 技术规格

| 参数 | 数值 |
|------|------|
| 工作电压 | 1.8V（内核）/ 3.3V（LED） |
| 通信接口 | I2C（最高 400kHz） |
| I2C 地址 | 0x57（7位）/ 0xAE（8位写）/ 0xAF（8位读） |
| 采样率 | 50/100/200/400/800/1000/1600/3200 Hz |
| ADC 分辨率 | 18位（262144 级） |
| LED 电流 | 0-51mA（可编程） |
| 工作温度 | -40°C ~ +85°C |

---

## 2. 硬件配置

### 2.1 引脚连接

```
STM32F103C8T6          MAX30102
    PB10 (I2C2_SCL) ----> SCL
    PB11 (I2C2_SDA) ----> SDA
    3.3V -------------> VIN
    GND --------------> GND
    (可选) PA1 -------> INT（中断输出）
```

### 2.2 CubeMX 配置

**I2C2 配置：**
```
模式：I2C
速度：Standard Mode (100 kHz)
地址长度：7-bit
时钟拉伸：Enabled
```

**生成的 HAL 初始化代码（i2c.c）：**
```c
void MX_I2C2_Init(void)
{
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;           // 100kHz
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c2);
}
```

### 2.3 硬件注意事项

1. **上拉电阻**：I2C 总线需要 4.7kΩ 上拉电阻（模块通常已内置）
2. **电源去耦**：VIN 附近放置 100nF 去耦电容
3. **光学隔离**：传感器表面需紧贴皮肤，避免漏光
4. **I2C 地址冲突**：确保 I2C2 总线上无其他设备使用 0x57 地址

---

## 3. 软件架构

### 3.1 文件结构

```
Core/task/
├── max30102_task.c     # 任务实现（主文件）
├── max30102_task.h     # 任务接口（头文件）
```

### 3.2 模块架构图

```
┌─────────────────────────────────────────────────────────────┐
│                      max30102_task                          │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │ I2C 通信层  │  │  数据处理层  │  │     算法层          │ │
│  │             │  │             │  │                     │ │
│  │ write_reg() │  │ read_fifo() │  │ DC滤波              │ │
│  │ read_reg()  │  │ process()   │  │ 峰值检测(hysteresis)│ │
│  │ read_regs() │  │             │  │ 心率计算            │ │
│  │             │  │             │  │ SpO2查表            │ │
│  └─────────────┘  └─────────────┘  └─────────────────────┘ │
├─────────────────────────────────────────────────────────────┤
│                    共享数据 (Mutex保护)                      │
│  MAX30102_Data_t: heart_rate, spo2, finger_detected, etc.  │
├─────────────────────────────────────────────────────────────┤
│                      公共接口                               │
│  max30102_task_init/start/stop, max30102_get_data, etc.    │
└─────────────────────────────────────────────────────────────┘
```

### 3.3 任务状态机

```
        ┌──────────────────────────────────────┐
        │            任务创建                   │
        │     max30102_task_init()             │
        └───────────────┬──────────────────────┘
                        │
                        ▼
        ┌──────────────────────────────────────┐
        │            挂起状态                   │
        │       (等待 start 命令)              │
        └───────────────┬──────────────────────┘
                        │ max30102_task_start()
                        ▼
        ┌──────────────────────────────────────┐
        │            运行状态                   │
        │                                      │
        │  ┌────────────────────────────────┐  │
        │  │ 1. 读取 FIFO 数据              │  │
        │  │ 2. 手指检测（带消抖）          │  │
        │  │ 3. DC 滤波提取 AC 分量         │  │
        │  │ 4. 峰值检测（hysteresis）      │  │
        │  │ 5. 心率/血氧计算               │  │
        │  │ 6. 更新共享数据（Mutex）       │  │
        │  │ 7. 更新 OLED 显示              │  │
        │  └────────────────────────────────┘  │
        │                                      │
        │     vTaskDelayUntil(10ms)            │
        └───────────────┬──────────────────────┘
                        │ max30102_task_stop()
                        ▼
        ┌──────────────────────────────────────┐
        │            挂起状态                   │
        └──────────────────────────────────────┘
```

---

## 4. IPC机制与资源分配

### 4.1 使用的 FreeRTOS IPC 机制

| IPC 类型 | 名称 | 用途 |
|----------|------|------|
| **互斥锁** | `max30102_mutex` | 保护共享数据 `MAX30102_Data_t` |
| **任务句柄** | `max30102_task_handle` | 任务挂起/恢复控制 |

### 4.2 Mutex 使用详解

**为什么需要 Mutex？**
- `max30102_task` 写入心率/血氧数据
- `oled_task` 读取数据进行显示
- `key_task` 读取手指检测状态
- 多任务并发访问同一数据结构需要互斥保护

**Mutex 创建：**
```c
// max30102_task.c:458
void max30102_task_init(void) {
    max30102_mutex = xSemaphoreCreateMutex();
    if (!max30102_mutex) {
        uart_printf_dma(&huart1, "[ERROR] MAX30102 mutex failed!\r\n");
        return;
    }
    // ...
}
```

**写入数据时加锁：**
```c
// max30102_task.c:329
xSemaphoreTake(max30102_mutex, portMAX_DELAY);
if (hr >= MAX30102_MIN_HEART_RATE && hr <= MAX30102_MAX_HEART_RATE) {
    max30102_shared_data.heart_rate = hr;
    max30102_shared_data.hr_valid = 1;
    max30102_shared_data.hr_status = HR_STATUS_VALID;
}
if (spo2 >= MAX30102_MIN_SPO2) {
    max30102_shared_data.spo2 = spo2;
    max30102_shared_data.spo2_valid = 1;
}
xSemaphoreGive(max30102_mutex);
```

**读取数据时加锁：**
```c
// max30102_task.c:496
void max30102_get_data(uint16_t *heart_rate, uint8_t *spo2, uint8_t *valid) {
    if (!max30102_mutex) return;

    xSemaphoreTake(max30102_mutex, portMAX_DELAY);
    if (heart_rate) *heart_rate = max30102_shared_data.heart_rate;
    if (spo2) *spo2 = max30102_shared_data.spo2;
    if (valid) *valid = max30102_shared_data.is_valid &&
                        max30102_shared_data.finger_detected;
    xSemaphoreGive(max30102_mutex);
}
```

### 4.3 任务间通信

```
┌────────────────┐     Mutex      ┌────────────────┐
│  max30102_task │ ───────────────│  oled_task     │
│  (写入数据)    │                │  (读取显示)    │
└────────────────┘                └────────────────┘
        │                                  │
        │                                  │
        │         ┌────────────────┐       │
        └─────────│  key_task      │───────┘
                  │  (读取状态)    │
                  └────────────────┘
```

### 4.4 资源冲突避免

| 资源 | 冲突场景 | 解决方案 |
|------|----------|----------|
| I2C2 总线 | MAX30102 独占 | 单任务访问，无需额外保护 |
| 共享数据 | 多任务读写 | Mutex 互斥 |
| OLED 显示 | 多任务更新 | 通过 oled_update_max30102() 接口 |
| UART 打印 | 多任务打印 | uart_tx_mutex 保护 |

### 4.5 错峰启动

为避免系统启动时多任务同时初始化导致资源竞争，采用错峰启动策略：

```c
// scheduler_task.c
void scheduler_init(void) {
    uart_task_init();      // 0ms
    // led_task_init();    // 50ms (已注释)
    oled_task_init();      // 150ms
    key_task_init();       // 100ms
    dht11_task_init();     // 250ms
    mq2_task_init();       // 300ms
    mpu6050_task_init();   // 400ms
    max30102_task_init();  // 450ms
}
```

**MAX30102 任务内部延迟：**
```c
// max30102_task.c:392
static void max30102_task(void *arg) {
    // 错峰启动
    vTaskDelay(pdMS_TO_TICKS(450));

    // 硬件初始化
    MAX30102_Status_t init_status = max30102_hw_init();
    // ...
}
```

---

## 5. 内存分配

### 5.1 堆内存配置

**FreeRTOSConfig.h：**
```c
#define configTOTAL_HEAP_SIZE    ((size_t)17408)  // 17KB
```

**为什么需要 17KB？**

| 组件 | 内存需求 | 说明 |
|------|----------|------|
| 任务 TCB | ~6 × 100B = 600B | 6个任务控制块 |
| 任务栈 | 见下表 | 各任务栈空间 |
| Mutex | ~6 × 80B = 480B | 6个互斥锁 |
| 队列 | ~200B | OLED 消息队列 |
| 定时器 | ~2 × 80B = 160B | 按键去抖定时器 |
| **总计** | **~15KB** | 预留 2KB 余量 |

### 5.2 任务栈分配

```c
// 各任务栈大小（单位：字 = 4字节）
uart_task:    256  (1KB)
led_task:     128  (512B)
key_task:     128  (512B)
oled_task:    256  (1KB)
dht11_task:   256  (1KB)
mq2_task:     384  (1.5KB)  // 浮点运算
mpu6050_task: 512  (2KB)    // DMP + 浮点
max30102_task:1024 (4KB)    // 复杂算法 + 浮点
```

**MAX30102 为什么需要 4KB 栈？**

1. **浮点运算**：DC滤波、峰值检测、SpO2计算涉及大量浮点
2. **局部数组**：FIFO 读取缓冲区 `uint8_t buf[6 * 16]`
3. **函数调用深度**：process_one_sample → detect_peak_hys → calculate_spo2
4. **查表数组**：SpO2 查表 184 字节（虽然是静态的，但访问时可能需要栈）

### 5.3 静态变量内存

```c
// max30102_task.c 静态变量
static uint32_t ir_buffer[100];      // 400B
static uint32_t red_buffer[100];     // 400B
static uint8_t uch_spo2_table[184];  // 184B (const, Flash)
static MAX30102_Data_t max30102_shared_data;  // ~32B
static PeakCtx_t pk;                 // ~20B
// 其他滤波变量约 50B
// 总计 RAM 约 900B
```

### 5.4 内存调试

**运行时检查剩余堆：**
```c
size_t free = xPortGetFreeHeapSize();
uart_printf_dma(&huart1, "[MEM] Free heap: %u bytes\r\n", free);
```

**栈溢出检测（FreeRTOSConfig.h）：**
```c
#define configCHECK_FOR_STACK_OVERFLOW  2  // 方法2：最严格
```

---

## 6. 驱动实现

### 6.1 寄存器地址定义

```c
// max30102_task.h
#define MAX30102_ADDR               0xAE  // I2C 写地址

// 中断相关
#define MAX30102_REG_INTR_STATUS_1  0x00
#define MAX30102_REG_INTR_STATUS_2  0x01
#define MAX30102_REG_INTR_ENABLE_1  0x02
#define MAX30102_REG_INTR_ENABLE_2  0x03

// FIFO 相关
#define MAX30102_REG_FIFO_WR_PTR    0x04
#define MAX30102_REG_OVF_COUNTER    0x05
#define MAX30102_REG_FIFO_RD_PTR    0x06
#define MAX30102_REG_FIFO_DATA      0x07
#define MAX30102_REG_FIFO_CONFIG    0x08

// 配置相关
#define MAX30102_REG_MODE_CONFIG    0x09
#define MAX30102_REG_SPO2_CONFIG    0x0A
#define MAX30102_REG_LED1_PA        0x0C  // 红光 LED 电流
#define MAX30102_REG_LED2_PA        0x0D  // 红外 LED 电流

// 温度相关
#define MAX30102_REG_TEMP_INT       0x1F
#define MAX30102_REG_TEMP_FRAC      0x20
#define MAX30102_REG_TEMP_CONFIG    0x21

// ID
#define MAX30102_REG_PART_ID        0xFF  // 应返回 0x15
```

### 6.2 I2C 通信函数

```c
// 写单个寄存器
static HAL_StatusTypeDef max30102_write_reg(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(&hi2c2, MAX30102_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &value, 1,
                             MAX30102_I2C_TIMEOUT);
}

// 读单个寄存器
static HAL_StatusTypeDef max30102_read_reg(uint8_t reg, uint8_t *value) {
    return HAL_I2C_Mem_Read(&hi2c2, MAX30102_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, value, 1,
                            MAX30102_I2C_TIMEOUT);
}

// 读多个寄存器（用于 FIFO 批量读取）
static HAL_StatusTypeDef max30102_read_regs(uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(&hi2c2, MAX30102_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, data, len,
                            MAX30102_I2C_TIMEOUT);
}
```

### 6.3 硬件初始化

```c
static MAX30102_Status_t max30102_hw_init(void) {
    uint8_t part_id = 0;

    // 等待传感器上电稳定
    vTaskDelay(pdMS_TO_TICKS(100));

    // 1. 检测设备 ID
    if (max30102_read_reg(MAX30102_REG_PART_ID, &part_id) != HAL_OK) {
        return MAX30102_NOT_FOUND;
    }
    if (part_id != 0x15) {
        uart_printf_dma(&huart1, "[MAX30102] Wrong ID: 0x%02X\r\n", part_id);
        return MAX30102_NOT_FOUND;
    }

    // 2. 软复位
    max30102_write_reg(MAX30102_REG_MODE_CONFIG, 0x40);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 3. 配置中断
    max30102_write_reg(MAX30102_REG_INTR_ENABLE_1, 0x80);  // A_FULL 中断
    max30102_write_reg(MAX30102_REG_INTR_ENABLE_2, 0x00);

    // 4. FIFO 配置
    // 0x1F = 无平均(000) + rollover使能(1) + A_FULL=15(01111)
    max30102_write_reg(MAX30102_REG_FIFO_CONFIG, 0x1F);

    // 5. 清空 FIFO
    max30102_write_reg(MAX30102_REG_FIFO_WR_PTR, 0x00);
    max30102_write_reg(MAX30102_REG_OVF_COUNTER, 0x00);
    max30102_write_reg(MAX30102_REG_FIFO_RD_PTR, 0x00);

    // 6. 模式配置：SpO2 模式（红光 + 红外）
    max30102_write_reg(MAX30102_REG_MODE_CONFIG, 0x03);

    // 7. SpO2 配置
    // 0x27 = ADC量程4096(01) + 采样率100Hz(001) + 脉冲宽度411us(11)
    max30102_write_reg(MAX30102_REG_SPO2_CONFIG, 0x27);

    // 8. LED 电流（0x24 ≈ 7.2mA，避免 ADC 饱和）
    max30102_write_reg(MAX30102_REG_LED1_PA, 0x24);  // 红光
    max30102_write_reg(MAX30102_REG_LED2_PA, 0x24);  // 红外

    // 9. 清除中断状态
    uint8_t tmp;
    max30102_read_reg(MAX30102_REG_INTR_STATUS_1, &tmp);
    max30102_read_reg(MAX30102_REG_INTR_STATUS_2, &tmp);

    uart_printf_dma(&huart1, "[MAX30102] HW Init OK, ID=0x%02X\r\n", part_id);
    return MAX30102_OK;
}
```

### 6.4 FIFO 数据读取

```c
// 获取 FIFO 中的样本数
static uint8_t max30102_get_fifo_count(void) {
    uint8_t wr = 0, rd = 0;
    if (max30102_read_reg(MAX30102_REG_FIFO_WR_PTR, &wr) != HAL_OK) return 0;
    if (max30102_read_reg(MAX30102_REG_FIFO_RD_PTR, &rd) != HAL_OK) return 0;
    return (uint8_t)((wr - rd) & 0x1F);  // FIFO 深度 32
}

// 批量读取 FIFO 数据
static void max30102_process_data(void) {
    uint8_t n = max30102_get_fifo_count();

    // 打破 10ms 锁相（任务周期与采样周期相同时可能同步）
    if (n == 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
        n = max30102_get_fifo_count();
    }
    if (n == 0) n = 1;  // 兜底
    if (n > 16) n = 16;  // 限制单次读取

    // 批量读取 6 字节/样本（3字节红光 + 3字节红外）
    uint8_t buf[6 * 16];
    if (max30102_read_regs(MAX30102_REG_FIFO_DATA, buf, 6 * n) != HAL_OK) return;

    // 时间戳单调递增
    static uint32_t t_ms = 0;
    if (t_ms == 0) t_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    for (uint8_t i = 0; i < n; i++) {
        uint8_t *p = &buf[6 * i];

        // 组合 24 位数据（高 18 位有效）
        uint32_t red_raw = (((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2]) & 0x3FFFF;
        uint32_t ir_raw  = (((uint32_t)p[3] << 16) | ((uint32_t)p[4] << 8) | p[5]) & 0x3FFFF;

        t_ms += 10;  // 每样本 +10ms（100Hz）
        process_one_sample(red_raw, ir_raw, t_ms);
    }
}
```

---

## 7. 心率血氧算法

### 7.1 算法流程图

```
原始数据 (ir_raw, red_raw)
        │
        ▼
┌───────────────────────────────────────┐
│           手指检测（带消抖）            │
│  ir_raw < 25000 且连续 50 次 → 无手指  │
└───────────────────┬───────────────────┘
                    │ 有手指
                    ▼
┌───────────────────────────────────────┐
│             DC 滤波                    │
│  ir_dc = ir_dc * 0.998 + ir_raw * 0.002│
│  ir_ac = ir_raw - ir_dc               │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│            AC 低通滤波                 │
│  ir_ac_filtered = 0.7*old + 0.3*new   │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│      峰值检测（Hysteresis）            │
│  1. 包络跟踪 (env)                    │
│  2. 动态阈值 (thr_hi, thr_lo)         │
│  3. 过零重武装 (armed)                 │
│  4. 三点极大检测                       │
└───────────────────┬───────────────────┘
                    │ 检测到峰值
                    ▼
┌───────────────────────────────────────┐
│            心率计算                    │
│  interval = t_ms - last_beat_time     │
│  HR = 60000 / avg_interval            │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│         血氧计算（查表法）              │
│  ratio = (AC_red/DC_red)/(AC_ir/DC_ir)│
│  SpO2 = uch_spo2_table[ratio * 100]   │
└───────────────────────────────────────┘
```

### 7.2 手指检测（带消抖）

```c
// 消抖变量
static uint8_t no_finger_cnt = 0;
#define NO_FINGER_DEBOUNCE 50  // 连续 50 个样本（500ms）

// 手指检测逻辑
if (ir_raw < MAX30102_FINGER_THRESHOLD) {  // 阈值 25000
    no_finger_cnt++;
    if (no_finger_cnt >= NO_FINGER_DEBOUNCE) {
        // 确认无手指，重置算法
        max30102_reset_algorithm();
        max30102_shared_data.finger_detected = 0;
        return;
    }
    // 消抖期间：跳过此样本，避免异常数据污染
    return;
} else {
    no_finger_cnt = 0;
    max30102_shared_data.finger_detected = 1;
}
```

**为什么需要消抖？**
- 手指轻微移动会导致 IR 值短暂下降
- 环境光干扰会导致瞬时读数异常
- 不消抖会导致算法状态频繁重置，无法计算心率

### 7.3 DC 滤波

```c
// 超慢 DC 跟踪（IIR 低通滤波器）
if (ir_dc < 1000) {
    ir_dc = (float)ir_raw;  // 首次初始化
    red_dc = (float)red_raw;
} else {
    ir_dc = ir_dc * 0.998f + (float)ir_raw * 0.002f;   // α = 0.002
    red_dc = red_dc * 0.998f + (float)red_raw * 0.002f;
}

// 计算 AC 分量
float ir_ac = (float)ir_raw - ir_dc;
float red_ac = (float)red_raw - red_dc;
```

**参数选择：**
- α = 0.002 对应截止频率约 0.03Hz
- 心率信号频率约 1-2Hz，远高于截止频率
- 太快（α大）会丢失 AC 信号
- 太慢（α小）会导致收敛时间过长

### 7.4 峰值检测（Hysteresis）

```c
typedef struct {
    float prev;          // 前一个 AC 值
    float prev2;         // 前两个 AC 值
    float env;           // 包络跟踪
    uint8_t armed;       // 是否允许触发
    uint32_t last_peak_ms;
} PeakCtx_t;
static PeakCtx_t pk = {0};

static uint8_t detect_peak_hys(float ac, uint32_t t_ms) {
    // 1) 包络跟踪（快升慢降）
    float a = fabsf(ac);
    if (pk.env < 100) {
        pk.env = a;
        pk.armed = 1;
    } else {
        if (a > pk.env) {
            pk.env = 0.9f * pk.env + 0.1f * a;   // 快速上升
        } else {
            pk.env = 0.995f * pk.env + 0.005f * a;  // 缓慢下降
        }
    }

    // 2) 动态阈值
    float thr_hi = pk.env * 0.35f;
    float thr_lo = pk.env * 0.10f;
    if (thr_hi < 150.0f) thr_hi = 150.0f;
    if (thr_lo < 50.0f) thr_lo = 50.0f;

    uint8_t peak = 0;

    // 3) 重武装条件：AC 过零或低于低阈值
    // 关键：过零检测解决了 armed 卡死问题
    if (a < thr_lo || (pk.prev > 0 && ac < 0) || (pk.prev < 0 && ac > 0)) {
        pk.armed = 1;
    }

    // 4) 三点极大检测（prev > prev2 且 prev > ac）
    if (pk.armed) {
        if (pk.prev > pk.prev2 && pk.prev > ac && pk.prev > thr_hi) {
            if (t_ms - pk.last_peak_ms > 450) {  // 最小间隔 450ms
                peak = 1;
                pk.last_peak_ms = t_ms;
                pk.armed = 0;  // 触发后解除武装
            }
        }
    }

    pk.prev2 = pk.prev;
    pk.prev = ac;
    return peak;
}
```

**Hysteresis 机制解释：**

```
信号幅度
    ^
    |     ╱╲        ╱╲        ╱╲
    |    ╱  ╲      ╱  ╲      ╱  ╲
────┼───╱────╲────╱────╲────╱────╲──── thr_hi
    |  ╱      ╲  ╱      ╲  ╱      ╲
    | ╱        ╲╱        ╲╱        ╲
────┼─────────────────────────────────── thr_lo
    |
    └──────────────────────────────────> 时间

状态：
armed=1 → 上升穿过 thr_hi → 触发峰值 → armed=0
        → 下降穿过 thr_lo 或过零 → armed=1
```

### 7.5 心率计算

```c
if (detect_peak_hys(ir_ac_filtered, t_ms)) {
    uint32_t interval = t_ms - last_beat_time;

    // 有效间隔：450-1200ms（对应 50-133 BPM）
    if (interval > 450 && interval < 1200) {
        beat_intervals[beat_index] = interval;
        beat_index = (beat_index + 1) % 8;
        peak_count++;

        // 4 个峰值开始计算
        if (peak_count >= 4) {
            uint32_t avg_interval = 0;
            uint8_t valid_count = 0;

            // 计算有效间隔平均值
            for (int i = 0; i < 8; i++) {
                if (beat_intervals[i] > 450 && beat_intervals[i] < 1200) {
                    avg_interval += beat_intervals[i];
                    valid_count++;
                }
            }

            if (valid_count >= 3) {
                avg_interval /= valid_count;
                uint16_t hr_raw = (uint16_t)(60000 / avg_interval);

                // IIR 平滑
                if (smoothed_hr < 30) {
                    smoothed_hr = (float)hr_raw;
                } else {
                    smoothed_hr = 0.7f * smoothed_hr + 0.3f * (float)hr_raw;
                }

                uint16_t hr = (uint16_t)(smoothed_hr + 0.5f);
                // 更新共享数据...
            }
        }
    }
    last_beat_time = t_ms;
}
```

### 7.6 血氧计算（Maxim 官方查表法）

```c
// SpO2 查表（Maxim 官方算法）
static const uint8_t uch_spo2_table[184] = {
    95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
    99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
    // ... 省略中间部分
    3, 2, 1
};

static uint8_t calculate_spo2(float red_ac, float red_dc_val, float ir_ac, float ir_dc_val) {
    if (ir_dc_val < 1000 || red_dc_val < 1000 || ir_ac < 10 || red_ac < 10) {
        return 0;
    }

    // 计算 R 值
    float r_red = red_ac / red_dc_val;
    float r_ir = ir_ac / (ir_dc_val + 0.0001f);
    float ratio = r_red / (r_ir + 0.0001f);

    // 转换为查表索引
    int32_t ratio_index = (int32_t)(ratio * 100.0f);

    // 查表
    if (ratio_index > 2 && ratio_index < 184) {
        return uch_spo2_table[ratio_index];
    }

    // 超出范围
    if (ratio_index <= 2) return 100;
    if (ratio_index >= 184) return 70;

    return 0;
}
```

**为什么使用查表法？**
- Maxim 官方推荐算法
- 避免复杂浮点公式在 Cortex-M3 上的精度损失
- 查表法更快、更稳定

---

## 8. 调试问题与解决方案

### 8.1 问题1：堆栈溢出（LED 快闪）

**现象：**
- 系统启动后 LED 快速闪烁
- FreeRTOS 栈溢出检测钩子被触发

**原因分析：**
1. 堆内存 12KB 不足以分配所有任务
2. MAX30102 任务栈 512 字不足以支撑复杂算法

**解决方案：**
```c
// FreeRTOSConfig.h
#define configTOTAL_HEAP_SIZE    ((size_t)17408)  // 12KB → 17KB

// max30102_task.c
xTaskCreate(max30102_task, "max30102", 1024, NULL, 2, &max30102_task_handle);
// 栈从 512 → 1024 字
```

**验证方法：**
```c
// 检查剩余堆
size_t free = xPortGetFreeHeapSize();
uart_printf_dma(&huart1, "Free: %u\r\n", free);  // 应 > 2000
```

### 8.2 问题2：ADC 饱和（ir=262143）

**现象：**
- 日志显示 `ir=262143`（18位最大值）
- 无法计算心率

**原因分析：**
- LED 电流过大（0x50 ≈ 16mA）
- 手指反射光太强，ADC 饱和

**解决方案：**
```c
// 降低 LED 电流
max30102_write_reg(MAX30102_REG_LED1_PA, 0x24);  // 0x50 → 0x24 (~7.2mA)
max30102_write_reg(MAX30102_REG_LED2_PA, 0x24);
```

### 8.3 问题3：心率始终为 0（armed 卡死）

**现象：**
- 日志显示 `arm=0 pk=0 hr=0`
- `pk` 始终不增长

**原因分析：**
- 峰值检测后 `armed=0`
- AC 信号无法满足 `a < thr_lo` 条件重武装
- 因为 PPG 信号围绕 0 波动，|AC| 值很少降到很低

**解决方案：**
```c
// 添加过零检测重武装
if (a < thr_lo || (pk.prev > 0 && ac < 0) || (pk.prev < 0 && ac > 0)) {
    pk.armed = 1;  // AC 过零时也重武装
}
```

### 8.4 问题4：手指检测频繁 reset

**现象：**
- `pk` 从 1 变回 0
- 日志显示 IR 值波动大（120000 → 30000）

**原因分析：**
- 手指轻微移动导致 IR 值短暂下降
- 阈值 50000 太高
- 无消抖机制

**解决方案：**
```c
// max30102_task.h
#define MAX30102_FINGER_THRESHOLD   25000  // 50000 → 25000

// max30102_task.c
#define NO_FINGER_DEBOUNCE 50  // 30 → 50 样本（500ms）

// 消抖期间跳过数据处理
if (ir_raw < MAX30102_FINGER_THRESHOLD) {
    no_finger_cnt++;
    if (no_finger_cnt >= NO_FINGER_DEBOUNCE) {
        max30102_reset_algorithm();
        return;
    }
    return;  // 跳过此样本
}
```

### 8.5 问题5：心率偏高 30-40%

**现象：**
- 实际心率 76-80 BPM
- 测量值 96-115 BPM

**原因分析：**
- FIFO_CONFIG = 0x5F（4 次平均）
- 实际输出 25Hz，但代码按 100Hz 计算时间戳
- 导致峰间隔被计算成原来的 1/4

**解决方案：**
```c
// 禁用 FIFO 平均
max30102_write_reg(MAX30102_REG_FIFO_CONFIG, 0x1F);  // 0x5F → 0x1F
// 0x1F = 无平均(000) + rollover(1) + A_FULL=15(01111)
```

### 8.6 调试日志格式

```c
// 每 100 个周期打印一次
if (update_count % 100 == 0) {
    uart_printf_dma(&huart1,
        "[MAX] n=%d ir=%lu ac=%.0f env=%.0f arm=%d pk=%d hr=%d\r\n",
        fifo_n, ir, ir_ac_filtered, pk.env, pk.armed, peak_count, hr);
}
```

**日志字段说明：**
| 字段 | 含义 | 正常范围 |
|------|------|----------|
| n | FIFO 样本数 | 0-10 |
| ir | 红外原始值 | 80000-130000 |
| ac | AC 滤波值 | -5000 ~ +5000 |
| env | 包络值 | 1000-10000 |
| arm | 武装状态 | 0/1 交替 |
| pk | 峰值计数 | 持续增长 |
| hr | 心率 | 60-100 |

---

## 9. API 参考

### 9.1 任务控制接口

```c
// 初始化任务（自动挂起）
void max30102_task_init(void);

// 启动任务
void max30102_task_start(void);

// 停止任务
void max30102_task_stop(void);

// 查询运行状态
uint8_t max30102_is_running(void);
```

### 9.2 数据获取接口

```c
// 获取基本数据（线程安全）
void max30102_get_data(uint16_t *heart_rate, uint8_t *spo2, uint8_t *valid);

// 获取完整数据结构
void max30102_get_full_data(MAX30102_Data_t *data);

// 查询手指检测状态
uint8_t max30102_is_finger_detected(void);

// 获取信号质量
uint8_t max30102_get_signal_quality(void);

// 获取心率状态
MAX30102_HR_Status_t max30102_get_hr_status(void);
// 返回值：HR_STATUS_INVALID / HR_STATUS_CALCULATING /
//         HR_STATUS_VALID / HR_STATUS_NO_FINGER
```

### 9.3 传感器控制接口

```c
// 重置传感器
MAX30102_Status_t max30102_reset(void);

// 读取芯片温度
float max30102_read_temperature(void);
```

### 9.4 数据结构

```c
typedef struct {
    uint16_t heart_rate;        // 心率 (BPM)
    uint8_t hr_valid;           // 心率有效标志
    MAX30102_HR_Status_t hr_status;

    uint8_t spo2;               // 血氧 (%)
    uint8_t spo2_valid;         // 血氧有效标志

    uint8_t finger_detected;    // 手指检测
    uint8_t signal_quality;     // 信号质量 (0-100)

    uint32_t red_raw;           // 红光原始值
    uint32_t ir_raw;            // 红外原始值

    float temperature;          // 芯片温度 (°C)
    uint8_t is_valid;           // 整体有效标志
} MAX30102_Data_t;
```

---

## 10. 附录：关键代码位置

| 功能 | 文件 | 行号 |
|------|------|------|
| 寄存器定义 | max30102_task.h | 23-44 |
| 状态枚举 | max30102_task.h | 47-61 |
| 数据结构 | max30102_task.h | 64-87 |
| SpO2 查表 | max30102_task.c | 27-40 |
| I2C 通信 | max30102_task.c | 72-84 |
| 硬件初始化 | max30102_task.c | 87-137 |
| 峰值检测 | max30102_task.c | 174-219 |
| 手指消抖 | max30102_task.c | 296-317 |
| 心率计算 | max30102_task.c | 312-364 |
| SpO2 计算 | max30102_task.c | 249-277 |
| 任务主函数 | max30102_task.c | 386-455 |
| 公共接口 | max30102_task.c | 458-569 |

---

## 参考资料

1. [MAX30102 Datasheet](https://www.maximintegrated.com/en/products/sensors/MAX30102.html)
2. [Maxim MAXREFDES117# 官方算法](https://www.maximintegrated.com/en/design/reference-design-center/system-board/6300.html)
3. [STM32+HAL库驱动MAX30102 - 博客园](https://www.cnblogs.com/zxr-blog/p/18146582)
4. [MAX30102血氧心率完整版 - CSDN](https://blog.csdn.net/weixin_50622833/article/details/121951942)

---

> 文档版本：v1.0 | 最后更新：2026-01-04
