# OLED和按键移植到FreeRTOS - 完整移植文档

> 作者：傲娇大小姐哈雷酱
> 日期：2026-01-02
> 项目：freertos_test (STM32F103RCT6)
> 从裸机到RTOS的正确移植姿势！

---

## 📋 目录

1. [移植概述](#移植概述)
2. [架构设计](#架构设计)
3. [文件组织结构](#文件组织结构)
4. [OLED任务实现](#oled任务实现)
5. [按键任务实现](#按键任务实现)
6. [移植步骤](#移植步骤)
7. [遇到的问题和解决方案](#遇到的问题和解决方案)
8. [后续传感器对接方案](#后续传感器对接方案)
9. [经验总结](#经验总结)

---

## 移植概述

### 移植目标

将**裸机版本**的OLED显示和按键检测模块，完整迁移到**FreeRTOS环境**，遵循RTOS的任务划分和IPC通信机制。

### 原始代码特点

**OLED驱动：**
- 软件I2C（PB4=SCL, PB5=SDA）
- 1024字节显示缓冲区（OLED_Buffer）
- "写缓冲区+统一刷新"模式
- ⚠️ `OLED_Flush()` 耗时约50-100ms

**按键驱动：**
- KEY1(PC8) 和 KEY2(PC9)
- 裸机轮询方式，20ms周期
- 软件去抖（30ms）

### 移植原则

✅ **任务独立性**：每个功能独立成任务
✅ **IPC通信**：任务间通过队列/事件组/互斥锁通信
✅ **非阻塞设计**：避免长时间占用CPU
✅ **资源保护**：共享资源用互斥锁保护
✅ **优先级合理**：按实时性需求分配优先级

---

## 架构设计

### 整体架构图

```
┌──────────────────────────────────────────────────────────────┐
│                      应用层任务                               │
│  (uart_rx_task, 未来的sensor_task, 等等)                     │
└────────────────┬─────────────────────────────────────────────┘
                 │ 通过队列/事件组通信
     ┌───────────┴───────────┐
     │                       │
┌────▼────────┐      ┌───────▼────────┐
│ oled_task   │      │   key_task     │
│ (优先级2)   │      │   (优先级3)    │
│             │      │                │
│ 接收显示请求│      │  GPIO中断触发  │
│ 管理缓冲区  │◄─────┤  软件去抖      │
│ 定期刷新屏幕│      │  发送按键事件  │
└────┬────────┘      └───────┬────────┘
     │                       │
     └───────────┬───────────┘
                 │
         ┌───────▼────────┐
         │  IPC机制       │
         │  - 消息队列    │
         │  - 事件组      │
         │  - 互斥锁      │
         │  - 软件定时器  │
         └────────────────┘
```

### 任务划分

| 任务名称 | 优先级 | 栈大小 | 职责 |
|---------|--------|--------|------|
| uart_rx_task | 3 | 512字节 | 串口接收，最高优先级 |
| key_task | 3 | 128字节 | 按键处理，需要快速响应 |
| led_control_task | 2 | 128字节 | LED状态指示 |
| oled_task | 2 | 256字节 | OLED显示管理 |
| Timer Service | 2 | 系统默认 | FreeRTOS软件定时器服务 |

### IPC机制选择

| IPC类型 | 使用场景 | 本项目应用 |
|---------|---------|-----------|
| **消息队列** | 任务间传递数据 | OLED显示命令队列 |
| **事件组** | 一对多通知 | 按键事件通知 |
| **互斥锁** | 保护共享资源 | 保护I2C总线和OLED缓冲区 |
| **软件定时器** | 延时触发 | 按键去抖定时器 |

---

## 文件组织结构

```
freertos_test/
├── Core/
│   ├── driver/               # 底层驱动（可复用）
│   │   ├── oled.c           # OLED底层驱动（软件I2C）
│   │   ├── oled.h
│   │   ├── oledfont.c       # ASCII字库
│   │   └── oledfont.h
│   ├── task/                 # FreeRTOS任务层
│   │   ├── oled_task.c      # OLED任务（新增）
│   │   ├── oled_task.h
│   │   ├── key_task.c       # 按键任务（新增）
│   │   ├── key_task.h
│   │   ├── led_task.c       # 现有LED任务
│   │   └── uart_task.c      # 现有UART任务
│   ├── APP/
│   │   └── scheduler_task.c # 任务调度初始化
│   └── Src/
│       └── stm32f1xx_it.c   # 中断处理（GPIO中断回调）
```

---

## OLED任务实现

### 设计思路

**核心理念：** 生产者-消费者模式

```
其他任务（生产者）              oled_task（消费者）
      │                              │
      ├─ oled_show_text()           │
      │     └─ 构造消息              │
      │     └─ xQueueSend() ───────→ xQueueReceive()
      │                              ├─ 获取互斥锁
      │                              ├─ 写入OLED_Buffer
      │                              └─ 释放互斥锁
      │                              │
      │                         [定期刷新]
      │                              │
      │                         OLED_Flush()
      │                         (50-100ms)
```

### IPC机制：消息队列

#### 1. 消息结构体设计

```c
typedef enum {
    OLED_CMD_SHOW_TEXT,      // 显示文本
    OLED_CMD_SHOW_NUM,       // 显示数字
    OLED_CMD_SHOW_FLOAT,     // 显示浮点数
    OLED_CMD_CLEAR,          // 清屏
    OLED_CMD_FLUSH,          // 立即刷新
} oled_cmd_type_t;

typedef struct {
    oled_cmd_type_t cmd;
    uint8_t line;             // 行号：1-4
    uint8_t column;           // 列号：1-16
    union {
        char text[32];        // 文本内容
        struct {
            uint32_t value;
            uint8_t length;
        } num;
        // ... 其他类型
    } data;
} oled_msg_t;
```

**设计要点：**
- 使用 `union` 节省内存（共用空间）
- 每条消息约40字节
- 队列深度10条，总共400字节

#### 2. 队列创建

```c
static QueueHandle_t oled_queue = NULL;

void oled_task_init(void) {
    // 创建消息队列（10条消息）
    oled_queue = xQueueCreate(10, sizeof(oled_msg_t));
    if (!oled_queue) {
        // 创建失败，停止运行
        for(;;);
    }
    // ...
}
```

#### 3. 生产者：发送显示请求（非阻塞）

```c
void oled_show_text(uint8_t line, uint8_t column, const char *text) {
    if (!oled_queue) return;

    oled_msg_t msg;
    msg.cmd = OLED_CMD_SHOW_TEXT;
    msg.line = line;
    msg.column = column;
    strncpy(msg.data.text, text, sizeof(msg.data.text) - 1);

    // 不等待，立即返回（超时=0）
    xQueueSend(oled_queue, &msg, 0);
}
```

**关键点：** 超时参数为 `0`，调用任务不会阻塞，立即返回！

#### 4. 消费者：OLED任务主循环

```c
static void oled_task(void *arg) {
    oled_msg_t msg;
    TickType_t last_flush = 0;

    // 初始化OLED硬件
    OLED_Init();
    OLED_Clear();
    OLED_Flush();

    for (;;) {
        // 1. 非阻塞接收（100ms超时）
        if (xQueueReceive(oled_queue, &msg, pdMS_TO_TICKS(100)) == pdPASS) {

            // 2. 获取互斥锁
            xSemaphoreTake(oled_mutex, portMAX_DELAY);

            // 3. 处理命令（写缓冲区）
            switch (msg.cmd) {
                case OLED_CMD_SHOW_TEXT:
                    OLED_ShowString(msg.line, msg.column, msg.data.text);
                    break;
                // ... 其他命令
            }

            // 4. 释放互斥锁
            xSemaphoreGive(oled_mutex);
        }

        // 5. 定期自动刷新（200ms）
        if (xTaskGetTickCount() - last_flush > pdMS_TO_TICKS(200)) {
            xSemaphoreTake(oled_mutex, portMAX_DELAY);
            OLED_Flush();  // 耗时50-100ms
            xSemaphoreGive(oled_mutex);
            last_flush = xTaskGetTickCount();
        }
    }
}
```

### IPC机制：互斥锁

**保护对象：**
1. I2C总线（软件I2C的GPIO操作）
2. OLED_Buffer 显示缓冲区

**为什么需要互斥锁？**
- OLED_Flush() 需要50-100ms完成1024字节传输
- 如果其他任务同时写缓冲区，会导致显示错乱
- I2C时序不可被打断

```c
static SemaphoreHandle_t oled_mutex = NULL;

void oled_task_init(void) {
    // 创建互斥锁
    oled_mutex = xSemaphoreCreateMutex();
    // ...
}
```

### 刷新策略

**方案A：定期自动刷新（采用）**
- 每200ms刷新一次
- 优点：简单，CPU占用稳定
- 缺点：最高5fps，不适合动画

**方案B：按需刷新**
- 收到 `OLED_CMD_FLUSH` 命令才刷新
- 优点：节省CPU
- 缺点：需要手动触发

**本项目采用方案A + 手动触发**：
- 默认200ms自动刷新
- 关键信息可以调用 `oled_flush_now()` 立即刷新

---

## 按键任务实现

### 设计思路

**核心理念：** GPIO中断 + 软件定时器去抖 + 事件组通知

```
按键按下
   ↓
GPIO中断（ISR）
   ↓
启动去抖定时器（30ms）
   ↓
定时器回调（30ms后）
   ↓
再次读取GPIO确认
   ↓
设置事件组位
   ↓
key_task 被唤醒
   ↓
处理按键逻辑
```

### IPC机制：事件组

**为什么选择事件组？**
- 按键是"事件"，不是"数据"
- 事件组天然支持"一对多"通知
- 多个任务可以同时等待按键事件

#### 1. 事件位定义

```c
#define KEY_EVENT_KEY1_PRESS   (1 << 0)  // 按键1事件
#define KEY_EVENT_KEY2_PRESS   (1 << 1)  // 按键2事件

static EventGroupHandle_t key_event_group = NULL;
```

#### 2. GPIO中断回调

```c
void key_exti_callback(uint16_t GPIO_Pin) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (GPIO_Pin == KEY1_Pin) {
        // 启动去抖定时器（30ms单次）
        xTimerStartFromISR(key1_debounce_timer, &xHigherPriorityTaskWoken);
    } else if (GPIO_Pin == KEY2_Pin) {
        xTimerStartFromISR(key2_debounce_timer, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```

**关键点：**
- 在ISR中不能做复杂操作
- 只启动定时器，不直接处理按键
- `portYIELD_FROM_ISR()` 保证高优先级任务立即运行

#### 3. 软件定时器回调（去抖确认）

```c
static void key1_debounce_callback(TimerHandle_t xTimer) {
    // 30ms后再次读取GPIO，确认是否仍然按下
    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
        // 确认按下，设置事件位
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xEventGroupSetBitsFromISR(key_event_group, KEY_EVENT_KEY1_PRESS,
                                  &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
```

**去抖原理：**
```
按键抖动波形：
       ┌─┐ ┌─┐   ┌──────────
       │ │ │ │   │
───────┘ └─┘ └───┘
 ^             ^
 │             │
 中断触发      30ms后确认
```

#### 4. 按键任务：等待事件

```c
static void key_task(void *arg) {
    EventBits_t events;

    for (;;) {
        // 阻塞等待按键事件
        events = xEventGroupWaitBits(
            key_event_group,
            KEY_EVENT_KEY1_PRESS | KEY_EVENT_KEY2_PRESS,
            pdTRUE,   // 清除事件位
            pdFALSE,  // 任意事件触发
            portMAX_DELAY
        );

        if (events & KEY_EVENT_KEY1_PRESS) {
            // KEY1：触发LED错误模式
            led_set_mode(LED_EVENT_ERROR);
            oled_show_text(4, 1, "KEY1: Error  ");
        }

        if (events & KEY_EVENT_KEY2_PRESS) {
            // KEY2：恢复LED心跳模式
            led_set_mode(LED_EVENT_HEARTBEAT);
            oled_show_text(4, 1, "KEY2: Normal ");
        }
    }
}
```

**事件组优势：**
- 任务完全阻塞，不占CPU
- 支持等待多个事件（KEY1 | KEY2）
- 自动清除事件位（`pdTRUE`）

---

## 移植步骤

### 步骤1：复制底层驱动

将原项目的OLED驱动文件复制到新项目：

```
D:\Linux_study\smart-helmet-ai\STM32_Project\APP\
├─ oled.c        → C:\Users\13615\Desktop\freertos_test\Core\driver\oled.c
├─ oled.h        → C:\Users\13615\Desktop\freertos_test\Core\driver\oled.h
├─ oledfont.c    → C:\Users\13615\Desktop\freertos_test\Core\driver\oledfont.c
└─ oledfont.h    → C:\Users\13615\Desktop\freertos_test\Core\driver\oledfont.h
```

**注意：** 底层驱动不需要修改！直接复用！

---

### 步骤2：创建OLED任务文件

**文件位置：** `Core\task\oled_task.h` 和 `Core\task\oled_task.c`

**关键内容：**
- 消息队列定义
- 互斥锁定义
- 对外接口（`oled_show_text()` 等）
- OLED任务主函数

---

### 步骤3：创建按键任务文件

**文件位置：** `Core\task\key_task.h` 和 `Core\task\key_task.c`

**关键内容：**
- 事件组定义
- 软件定时器定义
- GPIO中断回调函数
- 按键任务主函数

---

### 步骤4：配置GPIO（CubeMX）

#### OLED引脚配置

| 引脚 | 模式 | 说明 |
|------|------|------|
| PB4 | GPIO_Output | SCL（推挽输出） |
| PB5 | GPIO_Output | SDA（推挽输出） |

#### 按键引脚配置（重要！）

| 引脚 | 模式 | 触发方式 | 上拉 |
|------|------|---------|------|
| PC8 | **GPIO_EXTI8** | Falling edge | Pull-up |
| PC9 | **GPIO_EXTI9** | Falling edge | Pull-up |

**NVIC中断使能：**
- ✅ 勾选 `EXTI line[9:5] interrupts`
- **Preemption Priority**: 5（必须 >= 5）
- **Sub Priority**: 0

**关键注意：** 必须选择 `GPIO_EXTI8/9`，不是 `GPIO_Input`！

---

### 步骤5：添加GPIO中断回调

在 `Core\Src\stm32f1xx_it.c` 中：

```c
/* USER CODE BEGIN Includes */
#include "key_task.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 1 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 调用我们的按键中断处理函数
    key_exti_callback(GPIO_Pin);
}
/* USER CODE END 1 */
```

**注意：** 必须在 `key_task.h` 中声明 `key_exti_callback()` 函数！

---

### 步骤6：修改 scheduler_task.c

在 `Core\APP\scheduler_task.c` 中：

```c
#include "scheduler_task.h"
#include "oled_task.h"  // 新增
#include "key_task.h"   // 新增

void scheduler_init(void) {
    g_printf_mutex = xSemaphoreCreateMutex();
    data_queue = xQueueCreate(10, sizeof(sentor_msg_t));

    uart_task_init();   // 优先级3
    led_task_init();    // 优先级2
    oled_task_init();   // 优先级2（新增）
    key_task_init();    // 优先级3（新增）
}
```

---

### 步骤7：调整堆内存（如果需要）

在 `Core\Inc\FreeRTOSConfig.h` 中：

```c
#define configTOTAL_HEAP_SIZE  ((size_t)12288)  // 12KB
```

**内存占用估算：**
- OLED_Buffer: 1024字节
- oled_queue: ~400字节
- oled_task栈: 256字节
- key_task栈: 128字节
- 其他任务栈: ~2KB
- **总计约4KB**，12KB绰绰有余

---

### 步骤8：编译、烧录、测试

1. **编译项目**
2. **烧录到MCU**
3. **查看串口输出**：

```
[UART] DMA TX/RX Init OK!
[RX Task] Running!
[LED Task] Running!
[LED] Timer started OK!
[OLED Task] Running!
[OLED] Initializing...
[OLED] Init OK!
[KEY] Init OK!
[KEY Task] Running!
```

4. **测试按键**：
   - 按下KEY1 → 串口打印 `[KEY] KEY1 Pressed!`，LED快闪（100ms）
   - 按下KEY2 → 串口打印 `[KEY] KEY2 Pressed!`，LED慢闪（500ms）
   - OLED第4行显示按键信息

---

## 遇到的问题和解决方案

### 问题1：打印冲突 - "[RED Task]Running"

**现象：**
```
[UART] DMA TX/RX Init OK!
[RED Task]Running        ← 字符重叠
[LED Task] Running!
```

**原因：**
- uart_rx_task 和 led_control_task 几乎同时打印
- 虽然有互斥锁，但DMA发送需要时间
- 字符重叠："[RX..." + "[LED..." → "[RED..."

**解决方案：**

在每个任务启动时增加**错峰延迟**：

```c
// uart_rx_task - 优先级3，无延迟
uart_printf_dma(&huart1, "[RX Task] Running!\r\n");

// led_control_task - 优先级2，延迟50ms
vTaskDelay(pdMS_TO_TICKS(50));
uart_printf_dma(&huart1, "[LED Task] Running!\r\n");

// oled_task - 优先级2，延迟150ms
vTaskDelay(pdMS_TO_TICKS(150));
uart_printf_dma(&huart1, "[OLED Task] Running!\r\n");

// key_task - 优先级3，延迟100ms
vTaskDelay(pdMS_TO_TICKS(100));
uart_printf_dma(&huart1, "[KEY Task] Running!\r\n");
```

**规律：** 每个任务延迟递增50ms

---

### 问题2：按键无反应

**现象：**
- 串口打印 `[KEY Task] Running!`
- 但按下按键没有任何反应
- 没有打印 `[KEY] KEY1 Pressed!`

**排查步骤：**

#### 1. 检查 stm32f1xx_it.c 中是否有 EXTI9_5_IRQHandler()

```bash
grep "EXTI9_5_IRQHandler" Core/Src/stm32f1xx_it.c
```

**如果没有，说明CubeMX配置错误！**

#### 2. 检查 key_task.h 是否声明了 key_exti_callback()

```c
// 必须有这一行！
void key_exti_callback(uint16_t GPIO_Pin);
```

**如果没有，编译可能报错或链接失败！**

#### 3. 检查GPIO配置

**错误配置：** PC8/PC9 设置为 `GPIO_Input`
**正确配置：** PC8/PC9 设置为 `GPIO_EXTI8/9`

**重新配置CubeMX：**
1. 打开 `.ioc` 文件
2. 点击 PC8，选择 **GPIO_EXTI8**
3. 点击 PC9，选择 **GPIO_EXTI9**
4. 配置：
   - **GPIO mode**: External Interrupt Mode with Falling edge trigger detection
   - **GPIO Pull-up/Pull-down**: Pull-up
5. 点击 **NVIC** 选项卡
6. ✅ 勾选 **EXTI line[9:5] interrupts**
7. 设置 **Preemption Priority = 5**
8. 重新生成代码

#### 4. 添加调试输出

在 `key_exti_callback()` 中添加：

```c
void key_exti_callback(uint16_t GPIO_Pin) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (GPIO_Pin == KEY1_Pin) {
        uart_printf_dma(&huart1, "[ISR] KEY1 INT!\r\n");  // 调试
        xTimerStartFromISR(key1_debounce_timer, &xHigherPriorityTaskWoken);
    }
    // ...
}
```

**如果看到 `[ISR] KEY1 INT!`，说明中断触发了！**

---

### 问题3：OLED显示不刷新

**现象：**
- 调用 `oled_show_text()` 没有报错
- 但OLED屏幕不显示内容

**可能原因：**

1. **I2C引脚配置错误**
   - 检查PB4/PB5是否正确配置为GPIO_Output

2. **OLED_Init() 失败**
   - 在 `oled_task()` 中增加调试打印

3. **互斥锁死锁**
   - 检查是否忘记 `xSemaphoreGive()`

4. **队列满**
   - 增加队列深度：`xQueueCreate(20, sizeof(oled_msg_t))`

---

### 问题4：堆内存不足

**现象：**
```
[ERROR] OLED queue failed!
```

**原因：**
- `configTOTAL_HEAP_SIZE` 太小
- 创建队列、任务、IPC对象失败

**解决方案：**

在 `FreeRTOSConfig.h` 中增加堆大小：

```c
#define configTOTAL_HEAP_SIZE  ((size_t)12288)  // 从10000改为12288
```

---

## 后续传感器对接方案

### 架构扩展

```
传感器任务层
┌─────────────┬─────────────┬─────────────┐
│ temp_task   │ mpu_task    │ gps_task    │
│ (温湿度)    │ (姿态)      │ (定位)      │
└──────┬──────┴──────┬──────┴──────┬──────┘
       │             │             │
       └─────────────┼─────────────┘
                     │ 通过IPC通信
       ┌─────────────▼─────────────┐
       │      显示和控制层          │
       ├─────────────┬─────────────┤
       │ oled_task   │  key_task   │
       └─────────────┴─────────────┘
```

### 方案A：传感器直接写OLED（推荐）

**优点：** 解耦，传感器任务不关心显示细节

```c
// 温度传感器任务
static void temp_sensor_task(void *arg) {
    float temperature;

    for (;;) {
        // 读取温度
        temperature = read_temperature();

        // 发送到OLED显示（非阻塞）
        oled_show_text(3, 1, "Temp:");
        oled_show_float(3, 7, temperature, 2, 1);  // 显示 "23.5"

        vTaskDelay(pdMS_TO_TICKS(1000));  // 1秒更新一次
    }
}
```

**OLED任务自动处理**：
- 接收温度显示命令
- 更新缓冲区
- 定期刷新屏幕（200ms）

---

### 方案B：传感器发布数据，OLED订阅（高级）

**适用场景：** 多个任务需要同一传感器数据

```c
// 共享数据结构（用互斥锁保护）
typedef struct {
    float temperature;
    float humidity;
    uint32_t timestamp;
} sensor_data_t;

static sensor_data_t g_sensor_data;
static SemaphoreHandle_t g_sensor_data_mutex;

// 温度传感器任务（发布者）
static void temp_sensor_task(void *arg) {
    for (;;) {
        float temp = read_temperature();

        // 更新共享数据
        xSemaphoreTake(g_sensor_data_mutex, portMAX_DELAY);
        g_sensor_data.temperature = temp;
        g_sensor_data.timestamp = xTaskGetTickCount();
        xSemaphoreGive(g_sensor_data_mutex);

        // 通知OLED任务更新显示（事件组）
        xEventGroupSetBits(sensor_event_group, SENSOR_EVENT_TEMP_UPDATE);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// OLED任务（订阅者）
static void oled_task(void *arg) {
    for (;;) {
        // 等待传感器更新事件
        EventBits_t events = xEventGroupWaitBits(
            sensor_event_group,
            SENSOR_EVENT_TEMP_UPDATE | SENSOR_EVENT_HUM_UPDATE,
            pdTRUE, pdFALSE, pdMS_TO_TICKS(100)
        );

        if (events & SENSOR_EVENT_TEMP_UPDATE) {
            // 读取共享数据
            xSemaphoreTake(g_sensor_data_mutex, portMAX_DELAY);
            float temp = g_sensor_data.temperature;
            xSemaphoreGive(g_sensor_data_mutex);

            // 显示
            oled_show_float(3, 7, temp, 2, 1);
        }

        // 定期刷新...
    }
}
```

---

### 方案C：按键切换传感器页面

**场景：** OLED只有4行，无法同时显示所有传感器数据

```c
typedef enum {
    PAGE_MAIN = 0,      // 主页：系统状态
    PAGE_TEMP,          // 温湿度页面
    PAGE_MPU,           // 姿态页面
    PAGE_GPS,           // 定位页面
    PAGE_MAX
} oled_page_t;

static oled_page_t current_page = PAGE_MAIN;

// 按键任务：切换页面
static void key_task(void *arg) {
    for (;;) {
        events = xEventGroupWaitBits(...);

        if (events & KEY_EVENT_KEY2_PRESS) {
            // KEY2：切换页面
            current_page = (current_page + 1) % PAGE_MAX;

            // 清屏并显示新页面
            oled_clear_screen();
            switch (current_page) {
                case PAGE_MAIN:
                    oled_show_text(1, 1, "System Ready");
                    break;
                case PAGE_TEMP:
                    oled_show_text(1, 1, "Temperature");
                    break;
                // ...
            }
        }
    }
}

// 传感器任务：只在对应页面更新
static void temp_sensor_task(void *arg) {
    for (;;) {
        float temp = read_temperature();

        // 只在温度页面更新显示
        if (current_page == PAGE_TEMP) {
            oled_show_float(2, 1, temp, 2, 1);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

---

### IPC机制对比

| 方案 | IPC机制 | 优点 | 缺点 |
|------|--------|------|------|
| 方案A | 消息队列 | 解耦，简单 | 队列可能满 |
| 方案B | 共享内存+互斥锁+事件组 | 灵活，一对多 | 复杂，需要保护 |
| 方案C | 全局变量+状态机 | 简单直观 | 耦合度高 |

**本小姐推荐：**
- **简单项目**：方案A（消息队列直接写OLED）
- **复杂项目**：方案B（发布-订阅模式）
- **多页面显示**：方案C（按键切换页面）

---

## 经验总结

### 核心教训

#### 1. FreeRTOS移植三原则

✅ **任务独立**：每个功能独立成任务，不要在一个任务里做太多事
✅ **IPC通信**：任务间通过队列/事件组通信，不要用全局变量
✅ **非阻塞设计**：长时间操作放在独立任务中，不要阻塞其他任务

#### 2. 选择合适的IPC机制

| 场景 | IPC机制 |
|------|--------|
| 传递数据 | 消息队列 |
| 通知事件 | 事件组 |
| 保护资源 | 互斥锁 |
| 延时触发 | 软件定时器 |

#### 3. GPIO中断配置要点

⚠️ **必须选择 GPIO_EXTI 模式**，不是 GPIO_Input！
⚠️ **必须使能NVIC中断**
⚠️ **中断优先级必须 >= 5**（FreeRTOS要求）
⚠️ **必须声明中断回调函数**（在头文件中）

#### 4. 任务启动打印冲突

**问题：** 多个任务同时打印，字符重叠
**解决：** 错峰延迟启动（每个任务延迟递增50ms）

#### 5. 堆内存配置

**经验值：**
- 小项目（3-5个任务）：8KB-10KB
- 中项目（5-10个任务）：12KB-16KB
- 大项目（>10个任务）：20KB+

**检查方法：**
```c
size_t free_heap = xPortGetFreeHeapSize();
uart_printf_dma(&huart1, "Free heap: %u bytes\r\n", free_heap);
```

#### 6. 调试技巧

**有效的调试手段：**
1. 在关键步骤打印调试信息
2. 检查所有API返回值
3. 在ISR中添加简短打印（调试用）
4. 使用LED闪烁指示任务状态
5. 定期打印堆内存剩余

---

### FreeRTOS移植清单

移植外设到FreeRTOS时，检查以下内容：

- [ ] 底层驱动是否线程安全？（需要互斥锁保护）
- [ ] 是否有长时间阻塞操作？（需要独立任务）
- [ ] 任务优先级是否合理？（实时性需求高的优先级高）
- [ ] IPC机制是否合适？（队列/事件组/互斥锁）
- [ ] 堆内存是否充足？（xPortGetFreeHeapSize()）
- [ ] 任务栈是否充足？（uxTaskGetStackHighWaterMark()）
- [ ] 是否有死锁风险？（避免portMAX_DELAY在初始化阶段）
- [ ] GPIO中断是否正确配置？（EXTI模式+NVIC使能）
- [ ] 中断优先级是否符合FreeRTOS要求？（>= 5）

---

### 代码审查清单

**IPC使用：**
- [ ] 队列发送是否检查返回值？
- [ ] 互斥锁是否配对使用？（Take/Give）
- [ ] 事件组是否清除事件位？（pdTRUE）
- [ ] ISR中是否使用FromISR版本？（xQueueSendFromISR）

**资源管理：**
- [ ] 共享资源是否有互斥锁保护？
- [ ] 是否有内存泄漏？（动态分配需要释放）
- [ ] 是否检查队列/信号量创建成功？

**任务设计：**
- [ ] 任务循环中是否有延时？（避免占满CPU）
- [ ] 是否有死循环不释放CPU？
- [ ] 长时间操作是否分段处理？

---

## 附录：完整IPC机制对比表

| IPC机制 | 用途 | 阻塞性 | 数据传递 | 多对多 | 本项目应用 |
|---------|------|--------|---------|--------|-----------|
| **消息队列** | 任务间传递数据 | 可阻塞 | ✅ | ❌ | OLED显示命令 |
| **事件组** | 通知事件发生 | 可阻塞 | ❌ | ✅ | 按键事件、LED模式切换 |
| **互斥锁** | 保护共享资源 | 可阻塞 | ❌ | ❌ | OLED缓冲区、I2C总线 |
| **二值信号量** | 同步通知 | 可阻塞 | ❌ | ❌ | 串口接收通知 |
| **软件定时器** | 延时触发 | 非阻塞 | ❌ | ❌ | 按键去抖、LED定时翻转 |
| **直接通知** | 轻量级通知 | 可阻塞 | 少量数据 | ❌ | 未使用 |

---

## 参考资料

1. [FreeRTOS官方文档 - Queue Management](https://www.freertos.org/Embedded-RTOS-Queues.html)
2. [FreeRTOS官方文档 - Event Groups](https://www.freertos.org/FreeRTOS-Event-Groups.html)
3. [FreeRTOS官方文档 - Mutex](https://www.freertos.org/Real-time-embedded-RTOS-mutexes.html)
4. [Mastering the FreeRTOS Real Time Kernel](https://www.freertos.org/Documentation/RTOS_book.html)
5. 《嵌入式实时操作系统原理与实践》
6. STM32F103xx HAL库用户手册

---

## 结语

这次OLED和按键的移植，完整展示了**从裸机到RTOS的正确迁移方式**：

**核心要点：**
- 任务独立：每个功能独立成任务
- IPC通信：队列传数据、事件组通知、互斥锁保护
- 非阻塞设计：长时间操作独立处理，不阻塞其他任务
- 资源保护：共享资源必须用互斥锁保护
- 优先级合理：实时性高的任务优先级高

**笨蛋的成长：**
- 从"能用就行"到"架构设计"
- 从"单线程轮询"到"多任务并发"
- 从"全局变量"到"IPC机制"
- 这才是真正的工程师思维！

后续添加传感器任务时，按照这个模式扩展就行了！温湿度、姿态、GPS等传感器，都可以用同样的方式集成到系统中！

---

> **作者寄语：**
> 哼，笨蛋这次移植做得不错嘛！(￣▽￣)／
> 学会了任务划分、IPC通信、资源保护，算是入门FreeRTOS了！
> 记住：**好的架构设计比代码实现更重要**！
> 继续加油，本小姐看好你哦！(*￣︶￣)
> —— 傲娇大小姐 哈雷酱
