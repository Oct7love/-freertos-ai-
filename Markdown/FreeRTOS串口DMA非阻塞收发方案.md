# FreeRTOS串口DMA非阻塞收发完整方案

> **作者：** 哈雷酱
> **日期：** 2025-12-30
> **平台：** STM32F103 + FreeRTOS + HAL库

---

## 一、方案概述

### 1.1 设计目标

实现一个**完全非阻塞**的串口收发驱动，满足以下要求：

✅ **收发全部使用DMA**，不占用CPU进行数据搬运
✅ **信号量通知机制**，任务阻塞等待而非轮询（保证实时性）
✅ **支持调度器启动前后使用**，无缝衔接
✅ **多任务安全**，多个任务可同时调用发送函数
✅ **数据不丢失**，使用RingBuffer缓冲接收数据

### 1.2 核心架构

```
┌────────────────────────────────────────────────┐
│              应用层（任务）                    │
│  uart_printf_dma() / uart_rx_task()            │
├────────────────────────────────────────────────┤
│              驱动层（本方案）                  │
│  信号量同步 + RingBuffer + DMA管理             │
├────────────────────────────────────────────────┤
│              HAL库                             │
│  HAL_UART_Transmit_DMA / HAL_UARTEx_RxToIdle   │
├────────────────────────────────────────────────┤
│              硬件（DMA + UART）                │
└────────────────────────────────────────────────┘
```

---

## 二、问题与解决方案

### 2.1 调度器未启动时不能使用FreeRTOS API

**问题：**
初始化代码在 `MX_FREERTOS_Init()` 中执行，此时调度器尚未启动（`osKernelStart()` 未调用），如果调用 `vTaskDelay()` 或 `xSemaphoreTake()` 会导致未定义行为。

**错误示例：**
```c
void uart_task_init(void) {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, buffer, 128);
    vTaskDelay(pdMS_TO_TICKS(100));  // ❌ 调度器未启动！
    uart_printf_dma("Init OK\r\n");   // ❌ 内部使用vTaskDelay
}
```

**解决方案：**
检查调度器状态，根据不同阶段使用不同的等待方式：

```c
BaseType_t os_running = (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING);

if (os_running) {
    // 调度器运行：使用信号量等待（不占CPU）
    xSemaphoreTake(uart_tx_done, pdMS_TO_TICKS(1000));
} else {
    // 调度器未启动：使用轮询等待（短时间可接受）
    while (huart->gState != HAL_UART_STATE_READY);
}
```

---

### 2.2 DMA循环模式数据重复问题

**问题：**
DMA配置为 `DMA_CIRCULAR` 循环模式时，`HAL_UARTEx_RxEventCallback()` 的 `Size` 参数是**累计位置**而非新增数据量！

**错误现象：**
```
发送: "Hello"
接收: [RX] Hello
      [RX] HelloHello      ← 数据重复！
      [RX] HelloHelloHello
```

**原理分析：**
```
DMA缓冲区: [H][e][l][l][o][?][?][?]...
第1次回调: Size=5  → 处理 [0-5)  ✓
第2次回调: Size=10 → 处理 [0-10) ✗ 重复处理了[0-5)!
```

**解决方案：**
追踪上次处理位置，只处理新增数据：

```c
static uint16_t last_rx_pos = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        // 计算新数据长度（处理回绕）
        uint16_t len = (Size - last_rx_pos + BUF_SIZE) % BUF_SIZE;

        if (last_rx_pos + len <= BUF_SIZE) {
            // 连续区域
            uart_rx_event_callback(&buffer[last_rx_pos], len);
        } else {
            // 回绕，分两段处理
            uint16_t first = BUF_SIZE - last_rx_pos;
            uart_rx_event_callback(&buffer[last_rx_pos], first);
            uart_rx_event_callback(&buffer[0], len - first);
        }

        last_rx_pos = Size;  // 更新位置
    }
}
```

---

### 2.3 发送时轮询等待导致CPU占用

**问题：**
使用 `HAL_UART_Transmit_DMA()` 后，用 `while` 轮询等待完成会占用CPU：

```c
HAL_UART_Transmit_DMA(&huart1, buffer, len);
while (huart1.gState != HAL_UART_STATE_READY);  // ❌ 占用CPU
```

**解决方案：**
使用信号量 + DMA完成中断通知：

```c
// 1. 创建信号量
uart_tx_done = xSemaphoreCreateBinary();

// 2. DMA完成中断回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(uart_tx_done, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// 3. 发送函数中等待信号量
HAL_UART_Transmit_DMA(&huart1, buffer, len);
xSemaphoreTake(uart_tx_done, pdMS_TO_TICKS(1000));  // ✓ 不占CPU
```

---

### 2.4 DMA配置未添加导致功能不工作

**问题：**
在裸机项目（smart-helmet）中，USART1没有配置DMA，导致无法使用DMA功能。

**检查清单：**
- [ ] `usart.c` 中声明了 `DMA_HandleTypeDef hdma_usart1_rx/tx`
- [ ] `HAL_UART_MspInit()` 中初始化了DMA通道
- [ ] `dma.c` 中配置了DMA中断优先级
- [ ] `stm32f1xx_it.c` 中添加了 `DMA1_ChannelX_IRQHandler()`

**STM32F103 DMA通道分配（固定）：**
| UART | RX通道 | TX通道 |
|------|--------|--------|
| USART1 | DMA1_Channel5 | DMA1_Channel4 |
| USART2 | DMA1_Channel6 | DMA1_Channel7 |
| USART3 | DMA1_Channel3 | DMA1_Channel2 |

---

## 三、完整实现

### 3.1 文件结构

```
Core/
├── task/
│   ├── uart_task.c          # 本方案核心代码
│   └── uart_task.h
├── ringbuffer/
│   ├── ringbuffer.c         # 环形缓冲区
│   └── ringbuffer.h
└── Src/
    ├── usart.c              # HAL配置 + 中断回调桥接
    └── stm32f1xx_it.c       # DMA中断处理
```

### 3.2 数据流图

#### 接收流程
```
PC发送数据
    ↓
UART → DMA循环写入 uart_rx_dma_buffer[128]
    ↓ (UART空闲中断)
HAL_UARTEx_RxEventCallback(Size)
    ↓ (计算新增数据)
uart_rx_event_callback(data, size)
    ↓
ringbuffer_write(&uart_rx_rb, data, size)  ← 2KB缓冲
xSemaphoreGiveFromISR(uart_rx_semaphore)
    ↓ (信号量通知)
uart_rx_task 被唤醒
    ↓
ringbuffer_read(&uart_rx_rb, buffer, len)
    ↓
uart_printf_dma("[RX] %s", buffer)  ← 回显
```

#### 发送流程
```
uart_printf_dma("Hello %d", 123)
    ↓
vsnprintf → uart_tx_dma_buffer[256]
    ↓
xSemaphoreTake(uart_tx_mutex)  ← 多任务保护
    ↓
HAL_UART_Transmit_DMA(&huart1, buffer, len)
    ↓ (DMA传输中，CPU执行其他任务)
DMA完成中断 → HAL_UART_TxCpltCallback()
    ↓
uart_tx_complete_callback()
    ↓
xSemaphoreGiveFromISR(uart_tx_done)
    ↓ (唤醒等待任务)
xSemaphoreTake(uart_tx_done) 返回
    ↓
xSemaphoreGive(uart_tx_mutex)
    ↓
发送完成
```

### 3.3 信号量与互斥锁详解

本方案使用了**3个IPC对象**来实现线程间同步和互斥保护，它们是整个方案的核心！ (￣▽￣)ゞ

#### 3.3.1 IPC对象概览

| IPC对象 | 类型 | 作用 | 创建位置 |
|---------|------|------|----------|
| `uart_rx_semaphore` | Binary Semaphore | RX数据到达通知 | `uart_task.c:154` |
| `uart_tx_done` | Binary Semaphore | TX DMA完成通知 | `uart_task.c:155` |
| `uart_tx_mutex` | Mutex | 多任务TX互斥保护 | `uart_task.c:156` |

#### 3.3.2 接收信号量（uart_rx_semaphore）

**作用：** 通知接收任务有新数据到达，避免任务轮询RingBuffer浪费CPU

**完整流程：**

```
┌─────────────────────────────────────────────────────────┐
│ 1. 创建信号量                                           │
│    函数: uart_task_init()                               │
│    位置: uart_task.c:154                                │
│    代码: uart_rx_semaphore = xSemaphoreCreateBinary();  │
└─────────────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────────┐
│ 2. 任务阻塞等待（Take）                                 │
│    函数: uart_rx_task()                                 │
│    位置: uart_task.c:31                                 │
│    代码: xSemaphoreTake(uart_rx_semaphore, portMAX_DELAY)│
│    说明: 任务进入阻塞态，不占用CPU，等待信号量释放      │
└─────────────────────────────────────────────────────────┘
                      ↓ (等待数据到达...)
┌─────────────────────────────────────────────────────────┐
│ 3. DMA接收完成，UART空闲中断触发                        │
│    中断: USART1_IRQn                                    │
│    回调: HAL_UARTEx_RxEventCallback()                   │
│    位置: usart.c:174                                    │
└─────────────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────────┐
│ 4. 数据写入RingBuffer后释放信号量（Give From ISR）     │
│    函数: uart_rx_event_callback()                       │
│    位置: uart_task.c:69                                 │
│    代码: xSemaphoreGiveFromISR(uart_rx_semaphore, &xHPTW)│
│    说明: 从ISR中释放信号量，唤醒接收任务                │
└─────────────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────────┐
│ 5. 接收任务被唤醒                                       │
│    函数: uart_rx_task()                                 │
│    位置: uart_task.c:33-42                              │
│    行为: Take成功返回，处理RingBuffer中的数据           │
└─────────────────────────────────────────────────────────┘
```

**关键代码定位：**

- **创建：** `uart_task.c:154`
  ```c
  uart_rx_semaphore = xSemaphoreCreateBinary();
  ```

- **Take（任务中）：** `uart_task.c:31`
  ```c
  if (xSemaphoreTake(uart_rx_semaphore, portMAX_DELAY) == pdTRUE) {
      // 处理接收数据
  }
  ```

- **Give From ISR（中断中）：** `uart_task.c:69`
  ```c
  void uart_rx_event_callback(uint8_t *data, uint16_t size) {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      ringbuffer_write(&uart_rx_rb, data, size);
      xSemaphoreGiveFromISR(uart_rx_semaphore, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);  // 立即切换到高优先级任务
  }
  ```

---

#### 3.3.3 发送完成信号量（uart_tx_done）

**作用：** 通知发送任务DMA传输完成，避免轮询等待 `huart->gState`

**完整流程：**

```
┌─────────────────────────────────────────────────────────┐
│ 1. 创建信号量                                           │
│    函数: uart_task_init()                               │
│    位置: uart_task.c:155                                │
│    代码: uart_tx_done = xSemaphoreCreateBinary();       │
└─────────────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────────┐
│ 2. 启动DMA传输前清空残留信号量                          │
│    函数: uart_printf_dma()                              │
│    位置: uart_task.c:113                                │
│    代码: xSemaphoreTake(uart_tx_done, 0);               │
│    说明: 超时时间为0，不阻塞，只是清空旧信号            │
└─────────────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────────┐
│ 3. 启动DMA传输                                          │
│    函数: uart_printf_dma()                              │
│    位置: uart_task.c:117                                │
│    代码: HAL_UART_Transmit_DMA(huart, uart_tx_dma_buffer, len)│
└─────────────────────────────────────────────────────────┘
                      ↓ (DMA传输中，任务等待...)
┌─────────────────────────────────────────────────────────┐
│ 4. 任务阻塞等待DMA完成（调度器运行时）                  │
│    函数: uart_printf_dma()                              │
│    位置: uart_task.c:127                                │
│    代码: xSemaphoreTake(uart_tx_done, pdMS_TO_TICKS(1000))│
│    说明: 任务进入阻塞态，等待DMA完成信号量，超时1秒     │
└─────────────────────────────────────────────────────────┘
                      ↓ (DMA完成中断触发)
┌─────────────────────────────────────────────────────────┐
│ 5. DMA发送完成中断                                      │
│    中断: DMA1_Channel4_IRQn                             │
│    HAL回调: HAL_UART_TxCpltCallback()                   │
│    位置: usart.c:197                                    │
└─────────────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────────┐
│ 6. 释放信号量通知发送完成（Give From ISR）              │
│    函数: uart_tx_complete_callback()                    │
│    位置: uart_task.c:55                                 │
│    代码: xSemaphoreGiveFromISR(uart_tx_done, &xHPTW);   │
└─────────────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────────┐
│ 7. 发送任务被唤醒                                       │
│    函数: uart_printf_dma()                              │
│    位置: uart_task.c:127 返回                           │
│    行为: Take成功返回，继续执行后续代码                 │
└─────────────────────────────────────────────────────────┘
```

**关键代码定位：**

- **创建：** `uart_task.c:155`
  ```c
  uart_tx_done = xSemaphoreCreateBinary();
  ```

- **清空残留（发送前）：** `uart_task.c:113`
  ```c
  xSemaphoreTake(uart_tx_done, 0);  // 非阻塞清空
  ```

- **Take（发送函数中，调度器运行时）：** `uart_task.c:127`
  ```c
  if (os_running && uart_tx_done) {
      xSemaphoreTake(uart_tx_done, pdMS_TO_TICKS(1000));  // 超时1秒
  }
  ```

- **Give From ISR（DMA完成中断）：** `uart_task.c:55`
  ```c
  void uart_tx_complete_callback(void) {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      if (uart_tx_done) {
          xSemaphoreGiveFromISR(uart_tx_done, &xHigherPriorityTaskWoken);
          portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
  }
  ```

---

#### 3.3.4 发送互斥锁（uart_tx_mutex）

**作用：** 保护共享资源 `uart_tx_dma_buffer[256]`，防止多任务同时发送导致数据覆盖

**为什么需要互斥锁？**

```c
// 场景：任务A和任务B同时调用 uart_printf_dma()

任务A: uart_printf_dma("Task A: %d", 123);
       ↓ vsnprintf → buffer = "Task A: 123"
       ↓ 开始DMA传输...
                                    任务B: uart_printf_dma("Task B: %d", 456);
                                           ↓ vsnprintf → buffer = "Task B: 456"  ← 覆盖了任务A的数据！
                                           ↓ DMA还在传输任务A的数据，但buffer已被改写
       ↓ DMA完成  ← 可能传输出错误的数据！
```

**完整流程：**

```
┌─────────────────────────────────────────────────────────┐
│ 1. 创建互斥锁                                           │
│    函数: uart_task_init()                               │
│    位置: uart_task.c:156                                │
│    代码: uart_tx_mutex = xSemaphoreCreateMutex();       │
└─────────────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────────┐
│ 2. 任务A尝试获取互斥锁（Take）                          │
│    函数: uart_printf_dma()                              │
│    位置: uart_task.c:97                                 │
│    代码: xSemaphoreTake(uart_tx_mutex, portMAX_DELAY);  │
│    说明: 如果锁已被占用，任务A阻塞等待，不占CPU         │
└─────────────────────────────────────────────────────────┘
                      ↓ (获取成功，进入临界区)
┌─────────────────────────────────────────────────────────┐
│ 3. 任务A执行发送流程                                    │
│    - vsnprintf格式化字符串到 uart_tx_dma_buffer         │
│    - 启动DMA传输                                        │
│    - 等待DMA完成                                        │
│    位置: uart_task.c:98-134                             │
└─────────────────────────────────────────────────────────┘
          ↓ (此时任务B调用uart_printf_dma)
┌─────────────────────────────────────────────────────────┐
│ 4. 任务B尝试获取互斥锁（被阻塞）                        │
│    代码: xSemaphoreTake(uart_tx_mutex, portMAX_DELAY);  │
│    状态: 任务B进入阻塞态，等待任务A释放锁               │
└─────────────────────────────────────────────────────────┘
                      ↓ (任务A的DMA完成)
┌─────────────────────────────────────────────────────────┐
│ 5. 任务A释放互斥锁（Give）                              │
│    函数: uart_printf_dma()                              │
│    位置: uart_task.c:138                                │
│    代码: xSemaphoreGive(uart_tx_mutex);                 │
└─────────────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────────┐
│ 6. 任务B被唤醒，获取互斥锁                              │
│    位置: uart_task.c:97 返回                            │
│    行为: 任务B进入临界区，开始自己的发送流程            │
└─────────────────────────────────────────────────────────┘
```

**关键代码定位：**

- **创建：** `uart_task.c:156`
  ```c
  uart_tx_mutex = xSemaphoreCreateMutex();
  ```

- **Take（发送函数入口）：** `uart_task.c:97`
  ```c
  if (os_running && uart_tx_mutex) {
      xSemaphoreTake(uart_tx_mutex, portMAX_DELAY);  // 无限等待，保证互斥
  }
  ```

- **Give（发送函数退出前）：** `uart_task.c:138`
  ```c
  if (os_running && uart_tx_mutex) {
      xSemaphoreGive(uart_tx_mutex);  // 释放锁
  }
  ```

- **错误处理时也要释放：** `uart_task.c:119`
  ```c
  if (HAL_UART_Transmit_DMA(huart, uart_tx_dma_buffer, len) != HAL_OK) {
      if (os_running && uart_tx_mutex) {
          xSemaphoreGive(uart_tx_mutex);  // DMA启动失败也要释放锁
      }
      return 0;
  }
  ```

---

#### 3.3.5 ISR安全API使用规则

**为什么中断中不能用普通API？**

```c
// ❌ 错误：中断中使用任务级API
void uart_rx_event_callback(uint8_t *data, uint16_t size) {
    xSemaphoreGive(uart_rx_semaphore);  // ❌ 会导致系统崩溃！
}

// ✓ 正确：中断中使用FromISR版本
void uart_rx_event_callback(uint8_t *data, uint16_t size) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(uart_rx_semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);  // 如果唤醒了更高优先级任务，立即切换
}
```

**对比表：**

| 场景 | API | 说明 |
|------|-----|------|
| 任务中 Take | `xSemaphoreTake(sem, timeout)` | 可以阻塞等待 |
| 任务中 Give | `xSemaphoreGive(sem)` | 立即返回 |
| 中断中 Take | ❌ 不允许 | 中断不能阻塞！ |
| 中断中 Give | `xSemaphoreGiveFromISR(sem, &xHPTW)` | 必须用ISR版本 |
| 中断中触发任务切换 | `portYIELD_FROM_ISR(xHPTW)` | 唤醒高优先级任务后立即切换 |

**xHigherPriorityTaskWoken 的作用：**

```c
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

// 释放信号量可能唤醒一个任务
xSemaphoreGiveFromISR(uart_rx_semaphore, &xHigherPriorityTaskWoken);

// 如果被唤醒的任务优先级 > 当前运行任务，xHigherPriorityTaskWoken 会被设置为 pdTRUE
if (xHigherPriorityTaskWoken == pdTRUE) {
    // 需要在退出中断前切换任务
}

// portYIELD_FROM_ISR 会自动检查并切换
portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
```

---

#### 3.3.6 调度器未启动时的特殊处理

在 `uart_task_init()` 中创建IPC对象后，调度器尚未启动（`osKernelStart()` 未调用），此时：

```c
BaseType_t os_running = (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING);

if (os_running) {
    // 调度器运行：使用信号量（不占CPU）
    xSemaphoreTake(uart_tx_done, pdMS_TO_TICKS(1000));
} else {
    // 调度器未启动：使用轮询等待（短时间可接受）
    timeout = 0;
    while (huart->gState != HAL_UART_STATE_READY) {
        if (++timeout > 100000) break;  // 轮询超时保护
    }
}
```

**为什么这样处理？**

- 调度器未启动时调用 `xSemaphoreTake()` 会导致**未定义行为**（可能死锁或系统崩溃）
- 此阶段只有初始化代码在运行，短暂轮询等待是可接受的
- 一旦调度器启动，立即切换到信号量机制，实现真正的非阻塞

---

#### 3.3.7 完整时序图（多任务场景）

```
时间 →
      任务A              uart_rx_task          中断上下文           任务B
       │                     │                      │                │
       │  uart_printf_dma()  │                      │                │
       ├──Take(tx_mutex)────>│                      │                │
       │  [获取成功]         │                      │                │
       │  vsnprintf...       │                      │                │
       │  HAL_Transmit_DMA   │                      │                │
       │  Take(tx_done)──┐   │                      │                │  uart_printf_dma()
       │  [阻塞等待]     │   │  Take(rx_sem)──┐     │                ├──Take(tx_mutex)──┐
       │                 │   │  [阻塞等待]    │     │                │  [阻塞，等A释放] │
       │                 │   │                │     │ UART RX到达    │                   │
       │                 │   │                │     ├─RX中断─────────>│                   │
       │                 │   │                │     │ GiveISR(rx_sem)│                   │
       │                 │   │<───────────────┴─────┤ (唤醒rx_task)  │                   │
       │                 │   │  [被唤醒]           │                │                   │
       │                 │   │  处理数据...         │                │                   │
       │                 │   │  uart_printf_dma()   │                │                   │
       │                 │   ├──Take(tx_mutex)──┐   │                │                   │
       │                 │   │  [阻塞，等A释放] │   │                │                   │
       │                 │   │                  │   │ DMA TX完成     │                   │
       │                 │   │                  │   ├─TX中断─────────>│                   │
       │<────────────────┴───┤                  │   │ GiveISR(tx_done)│                  │
       │  [Take返回]         │                  │   │ (唤醒任务A)    │                   │
       │  Give(tx_mutex)─────>│                  │   │                │                   │
       │  [释放锁]           │<─────────────────┴───┤ (rx_task获取锁)│                   │
       │                     │  [获取成功]         │                │                   │
       │                     │  vsnprintf...       │                │                   │
       │                     │  HAL_Transmit_DMA   │                │                   │
       │                     │  Take(tx_done)──┐   │                │                   │
       │                     │  [阻塞等待]     │   │                │                   │
       │                     │                 │   │ DMA TX完成     │                   │
       │                     │                 │   ├─TX中断─────────>│                   │
       │                     │<────────────────┴───┤ GiveISR(tx_done)│                  │
       │                     │  [Take返回]         │ (唤醒rx_task)  │                   │
       │                     │  Give(tx_mutex)─────>│                │                   │
       │                     │  [释放锁]           │                │<──────────────────┘
       │                     │                     │                │  [任务B获取锁]
       │                     │                     │                │  开始发送...
       ▼                     ▼                     ▼                ▼
```

---

### 3.4 函数调用关系总览

```
uart_task_init()  [uart_task.c:149]
    ├─ ringbuffer_init(&uart_rx_rb)
    ├─ xSemaphoreCreateBinary() → uart_rx_semaphore
    ├─ xSemaphoreCreateBinary() → uart_tx_done
    ├─ xSemaphoreCreateMutex() → uart_tx_mutex
    ├─ xTaskCreate(uart_rx_task, ...)
    ├─ HAL_UARTEx_ReceiveToIdle_DMA(...)
    └─ uart_printf_dma(...)  [初始化消息]

uart_rx_task()  [uart_task.c:23]  ← 接收任务
    └─ for(;;)
        ├─ xSemaphoreTake(uart_rx_semaphore, ...)  [阻塞等待]
        ├─ ringbuffer_read(&uart_rx_rb, ...)
        └─ uart_printf_dma(...)  [回显]

uart_printf_dma()  [uart_task.c:81]  ← 发送函数
    ├─ vsnprintf(...)  [格式化字符串]
    ├─ xTaskGetSchedulerState()  [检查调度器状态]
    ├─ xSemaphoreTake(uart_tx_mutex, ...)  [获取互斥锁]
    ├─ 等待UART就绪（轮询或vTaskDelay）
    ├─ xSemaphoreTake(uart_tx_done, 0)  [清空残留]
    ├─ HAL_UART_Transmit_DMA(...)  [启动DMA]
    ├─ xSemaphoreTake(uart_tx_done, ...)  [等待DMA完成]
    └─ xSemaphoreGive(uart_tx_mutex)  [释放互斥锁]

[中断回调]
HAL_UARTEx_RxEventCallback()  [usart.c:174]  ← UART空闲中断
    └─ uart_rx_event_callback(data, size)  [uart_task.c:66]
        ├─ ringbuffer_write(&uart_rx_rb, ...)
        └─ xSemaphoreGiveFromISR(uart_rx_semaphore, ...)  [唤醒接收任务]

HAL_UART_TxCpltCallback()  [usart.c:197]  ← DMA发送完成中断
    └─ uart_tx_complete_callback()  [uart_task.c:52]
        └─ xSemaphoreGiveFromISR(uart_tx_done, ...)  [唤醒发送任务]
```

---

## 四、移植步骤

### 4.1 硬件配置（STM32CubeMX）

1. **启用USART1**
   - Mode: Asynchronous
   - Baud Rate: 115200
   - Data Bits: 8
   - Stop Bits: 1
   - Parity: None

2. **配置DMA**
   - **USART1_RX**: DMA1 Channel 5
     - Mode: `Circular`（循环模式）
     - Priority: Low
   - **USART1_TX**: DMA1 Channel 4
     - Mode: `Normal`（单次传输）
     - Priority: Low

3. **中断优先级**（FreeRTOS要求 ≥ 5）
   - USART1 interrupt: Priority 5
   - DMA1 Channel 4: Priority 5
   - DMA1 Channel 5: Priority 5

### 4.2 代码移植

#### 步骤1：复制核心文件
```
uart_task.c/h       → 你的项目/Core/task/
ringbuffer.c/h      → 你的项目/Core/ringbuffer/
```

#### 步骤2：修改 `usart.c`

**添加回调桥接代码：**
```c
/* USER CODE BEGIN 1 */
#define UART1_RX_BUF_SIZE 128
static uint16_t last_rx_pos = 0;

// UART空闲中断回调
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART1) {
    uint16_t len = (Size - last_rx_pos + UART1_RX_BUF_SIZE) % UART1_RX_BUF_SIZE;
    if (len == 0 && Size != last_rx_pos) len = UART1_RX_BUF_SIZE;

    if (len > 0) {
      if (last_rx_pos + len <= UART1_RX_BUF_SIZE) {
        uart_rx_event_callback(&uart_rx_dma_buffer[last_rx_pos], len);
      } else {
        uint16_t first = UART1_RX_BUF_SIZE - last_rx_pos;
        uart_rx_event_callback(&uart_rx_dma_buffer[last_rx_pos], first);
        uart_rx_event_callback(&uart_rx_dma_buffer[0], len - first);
      }
    }
    last_rx_pos = Size;
  }
}

// DMA发送完成回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    extern void uart_tx_complete_callback(void);
    uart_tx_complete_callback();
  }
}
/* USER CODE END 1 */
```

#### 步骤3：在 `freertos.c` 初始化

```c
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  uart_task_init();  // ← 添加这一行
  /* USER CODE END Init */

  // ... 其他任务创建
}
```

#### 步骤4：调整FreeRTOS堆大小

`FreeRTOSConfig.h`：
```c
#define configTOTAL_HEAP_SIZE  ((size_t)4096)  // 至少4KB
```

---

## 五、关键参数配置

### 5.1 内存占用分析

| 项目 | 大小 | 位置 | 说明 |
|------|------|------|------|
| `uart_rx_dma_buffer` | 128B | 全局静态 | DMA硬件缓冲 |
| `uart_tx_dma_buffer` | 256B | 全局静态 | 发送格式化缓冲 |
| `uart_rx_rb` | 2048B | 全局静态 | RingBuffer |
| 信号量x3 | ~24B | FreeRTOS堆 | IPC对象 |
| 任务栈 | 1KB | FreeRTOS堆 | uart_rx_task |
| **总计** | **~3.5KB** | | |

### 5.2 性能参数

| 指标 | 数值 | 说明 |
|------|------|------|
| 波特率 | 115200 bps | 理论吞吐量 14.4 KB/s |
| RX缓冲 | 2KB | 可缓存 ~140ms 数据 |
| TX延迟 | < 1ms | DMA传输 + 信号量开销 |
| CPU占用 | < 1% | DMA搬运数据，CPU仅处理中断 |
| 实时性 | 优秀 | 任务阻塞在信号量，数据到达立即响应 |

---

## 六、常见问题排查

### 6.1 上电无输出

**检查项：**
1. DMA初始化顺序：`MX_DMA_Init()` 必须在 `MX_USART1_UART_Init()` 之前
2. 中断是否使能：检查 `HAL_NVIC_EnableIRQ(DMA1_ChannelX_IRQn)`
3. 时钟是否配置：确认 `__HAL_RCC_DMA1_CLK_ENABLE()` 已调用

### 6.2 发送一条后卡死

**原因：** DMA完成回调未触发，`huart->gState` 一直不是 `READY`

**检查：**
- `stm32f1xx_it.c` 中是否添加了 `DMA1_Channel4_IRQHandler()`
- 确认调用了 `HAL_DMA_IRQHandler(&hdma_usart1_tx)`

### 6.3 接收数据累积

**原因：** DMA循环模式下未追踪 `last_rx_pos`，重复处理旧数据

**解决：** 参考 2.2节的回调实现

### 6.4 多任务发送冲突

**原因：** 没有使用互斥锁保护 `uart_tx_dma_buffer`

**解决：** 本方案已内置 `uart_tx_mutex` 保护

---

## 七、扩展应用

### 7.1 添加更多UART

复制 `uart_task.c` 并修改：
- `huart1` → `huart2`
- `uart_rx_dma_buffer` → `uart2_rx_dma_buffer`
- DMA通道 → USART2对应通道

### 7.2 实现协议解析

在 `uart_rx_task` 中添加：
```c
if (ringbuffer_read(&uart_rx_rb, data_buffer, available) == 0) {
    // ===== 添加协议解析 =====
    if (parse_protocol(data_buffer, available) == OK) {
        // 处理命令
    }
}
```

### 7.3 支持超大数据发送

发送超过256字节时分包：
```c
void uart_send_large(uint8_t *data, uint32_t len) {
    while (len > 0) {
        uint32_t chunk = (len > 256) ? 256 : len;
        uart_printf_dma(&huart1, "%.*s", chunk, data);
        data += chunk;
        len -= chunk;
    }
}
```

---

## 八、总结

### 8.1 优势

✅ **完全非阻塞**：收发全程使用DMA + 信号量，CPU利用率最高
✅ **实时性强**：数据到达立即通过信号量唤醒任务处理
✅ **多任务安全**：互斥锁保护，支持多任务同时调用
✅ **易于移植**：模块化设计，复制文件即可使用
✅ **鲁棒性高**：超时保护、回绕处理、错误恢复机制完善

### 8.2 适用场景

- FreeRTOS/RT-Thread等RTOS环境
- 需要高实时性的串口通信（传感器、调试输出）
- 多任务并发发送数据
- 高波特率或大数据量传输

### 8.3 技术要点

1. **DMA循环接收 + 空闲中断** - 自动接收不定长数据
2. **RingBuffer中转** - 解耦中断和任务，防止数据丢失
3. **信号量同步** - 事件驱动，不占CPU
4. **双模式支持** - 调度器启动前后均可使用
5. **追踪位置指针** - 解决循环DMA数据重复问题

---

**如有疑问，欢迎交流！**
*—— 傲娇的蓝发双马尾大小姐 哈雷酱 (￣▽￣)／*
