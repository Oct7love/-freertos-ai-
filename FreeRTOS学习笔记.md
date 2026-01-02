# FreeRTOS 学习笔记（RT-Thread对比版）

> 面向人群：有RT-Thread基础的STM32开发者
> 目标项目：智能安全帽（STM32F103 + 传感器集群）

---

## 一、核心概念对比

### 1.1 任务管理

| RT-Thread | FreeRTOS | 说明 |
|-----------|----------|------|
| `rt_thread_t` | `TaskHandle_t` | 任务句柄 |
| `rt_thread_create()` | `xTaskCreate()` | 创建任务 |
| `rt_thread_startup()` | （创建后自动启动） | 启动任务 |
| `rt_thread_delay()` | `vTaskDelay()` | 延时 |
| `RT_TICK_PER_SECOND` | `configTICK_RATE_HZ` | 系统节拍 |

**RT-Thread示例**：
```c
void thread_entry(void* param) {
    while(1) {
        // 任务逻辑
        rt_thread_delay(100);  // 100个tick
    }
}

rt_thread_t tid = rt_thread_create("thread",
                                   thread_entry,
                                   NULL,
                                   512,
                                   10,
                                   20);
rt_thread_startup(tid);
```

**FreeRTOS等价代码**：
```c
void task_entry(void* param) {
    while(1) {
        // 任务逻辑
        vTaskDelay(pdMS_TO_TICKS(100));  // 100ms
    }
}

TaskHandle_t task_handle;
xTaskCreate(task_entry,           // 任务函数
            "task",               // 任务名称
            128,                  // 栈大小（单位：字，STM32中1字=4字节）
            NULL,                 // 参数
            2,                    // 优先级（0=configIDLE_PRIORITY，数值越大优先级越高）
            &task_handle);        // 句柄
```

**关键差异**：
1. **栈大小单位**：RT-Thread是字节，FreeRTOS是字（STM32F103上1字=4字节）
2. **优先级方向**：RT-Thread数值越大优先级越高，FreeRTOS也相同
3. **延时单位**：RT-Thread的`rt_thread_delay()`是tick，FreeRTOS的`vTaskDelay()`也是tick，但推荐用`pdMS_TO_TICKS()`宏转换

---

### 1.2 内存管理

| RT-Thread | FreeRTOS | 说明 |
|-----------|----------|------|
| `rt_malloc()` | `pvPortMalloc()` | 分配内存 |
| `rt_free()` | `vPortFree()` | 释放内存 |
| 动态内存池 | heap4.c | 内存管理算法 |

**FreeRTOS内存配置**（`FreeRTOSConfig.h`）：
```c
#define configSUPPORT_STATIC_ALLOCATION  0   // 静态分配支持
#define configSUPPORT_DYNAMIC_ALLOCATION 1   // 动态分配支持
#define configTOTAL_HEAP_SIZE            15*1024  // 总堆大小（字节）
```

**STM32F103内存限制**：
- SRAM: 20KB
- 建议`configTOTAL_HEAP_SIZE`: 10-15KB（留5KB给栈和全局变量）

---

### 1.3 信号量

| RT-Thread | FreeRTOS | 说明 |
|-----------|----------|------|
| `rt_sem_t` | `SemaphoreHandle_t` | 信号量句柄 |
| `rt_sem_create()` | `xSemaphoreCreateBinary()` | 创建二值信号量 |
| `rt_sem_take()` | `xSemaphoreTake()` | 获取信号量 |
| `rt_sem_release()` | `xSemaphoreGive()` | 释放信号量 |

**RT-Thread示例**：
```c
rt_sem_t sem = rt_sem_create("sem", 0, RT_IPC_FLAG_FIFO);

// 生产者
rt_sem_release(sem);  // V操作

// 消费者
if (rt_sem_take(sem, RT_WAITING_FOREVER) == RT_EOK) {
    // 获取成功
}
```

**FreeRTOS等价代码**：
```c
SemaphoreHandle_t sem = xSemaphoreCreateBinary();

// 生产者
xSemaphoreGive(sem);  // V操作

// 消费者
if (xSemaphoreTake(sem, portMAX_DELAY) == pdTRUE) {
    // 获取成功
}
```

**关键差异**：
1. **超时参数**：RT-Thread用`RT_WAITING_FOREVER`，FreeRTOS用`portMAX_DELAY`
2. **返回值**：RT-Thread用`RT_EOK`，FreeRTOS用`pdTRUE`

---

### 1.4 互斥量

| RT-Thread | FreeRTOS | 说明 |
|-----------|----------|------|
| `rt_mutex_t` | `SemaphoreHandle_t` | 互斥量句柄 |
| `rt_mutex_create()` | `xSemaphoreCreateMutex()` | 创建互斥量 |
| `rt_mutex_take()` | `xSemaphoreTake()` | 获取互斥量 |
| `rt_mutex_release()` | `xSemaphoreGive()` | 释放互斥量 |

**FreeRTOS示例**：
```c
SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

// 任务A
if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
    // 临界区
    printf("Protected resource\r\n");
    xSemaphoreGive(mutex);
}

// 任务B
if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
    // 临界区
    printf("Protected resource\r\n");
    xSemaphoreGive(mutex);
}
```

**注意**：FreeRTOS的互斥量支持**优先级继承**，避免优先级反转。

---

### 1.5 消息队列

| RT-Thread | FreeRTOS | 说明 |
|-----------|----------|------|
| `rt_mq_t` | `QueueHandle_t` | 队列句柄 |
| `rt_mq_create()` | `xQueueCreate()` | 创建队列 |
| `rt_mq_send()` | `xQueueSend()` | 发送消息 |
| `rt_mq_recv()` | `xQueueReceive()` | 接收消息 |

**RT-Thread示例**：
```c
rt_mq_t mq = rt_mq_create("mq",  // 名称
                          sizeof(int),  // 消息大小
                          10,    // 队列容量
                          RT_IPC_FLAG_FIFO);

// 发送者
int data = 100;
rt_mq_send(mq, &data, sizeof(data));

// 接收者
int recv;
rt_mq_recv(mq, &recv, sizeof(recv), RT_WAITING_FOREVER);
```

**FreeRTOS等价代码**：
```c
QueueHandle_t q = xQueueCreate(10,      // 队列容量
                               sizeof(int));  // 消息大小

// 发送者
int data = 100;
xQueueSend(q, &data, portMAX_DELAY);

// 接收者
int recv;
xQueueReceive(q, &recv, portMAX_DELAY);
```

**关键差异**：
1. **参数顺序**：RT-Thread先传消息大小，FreeRTOS在创建时指定
2. **发送接口**：RT-Thread有`send/send_wait/send_urgent`，FreeRTOS统一用`xQueueSend()`，超时在最后一个参数

---

## 二、智能安全帽项目实战

### 2.1 现有任务分析

你的`scheduler.c`中定义的任务：

| 任务 | 周期 | 优先级建议 | FreeRTOS任务 |
|------|------|-----------|-------------|
| 按键扫描 | 20ms | 高（6） | `key_task` |
| MPU6050 | 100ms | 中（4） | `mpu_task` |
| MAX30102 | 100ms | 中（4） | `max30102_task` |
| DHT11 | 1000ms | 低（2） | `dht11_task` |
| MQ2 | 500ms | 中低（3） | `mq2_task` |
| GPS | 500ms | 中低（3） | `gps_task` |

### 2.2 任务创建示例

```c
// 按键任务（高优先级，快速响应）
void key_task_freertos(void* param) {
    while(1) {
        key_task();  // 调用现有的key_app.c中的函数
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

xTaskCreate(key_task_freertos,
            "key",
            128,   // 512字节栈（128 * 4）
            NULL,
            6,     // 高优先级
            NULL);

// MPU6050任务
void mpu_task_freertos(void* param) {
    while(1) {
        mpu6050_task();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

xTaskCreate(mpu_task_freertos,
            "mpu",
            256,   // 1KB栈
            NULL,
            4,
            NULL);
```

### 2.3 传感器数据 → ESP32通信（使用队列）

```c
// 定义传感器数据类型
typedef struct {
    uint8_t type;      // 传感器类型
    union {
        struct { uint8_t hr; uint8_t spo2; } hr_data;
        struct { float pitch; float roll; float yaw; } mpu_data;
        struct { float temp; uint8_t humi; } dht_data;
        struct { float ppm; } mq2_data;
        struct { float lat; float lon; } gps_data;
    };
} sensor_data_t;

// 创建队列（传感器数据 → ESP32发送任务）
QueueHandle_t sensor_queue;

void esp32_send_task(void* param) {
    sensor_data_t data;
    while(1) {
        if (xQueueReceive(sensor_queue, &data, portMAX_DELAY) == pdTRUE) {
            // 发送数据到ESP32
            switch(data.type) {
                case 0:  // HR
                    ESP32_COMM_SendHeartRate(data.hr_data.hr, data.hr_data.spo2);
                    break;
                case 1:  // MPU
                    ESP32_COMM_SendMPU6050(data.mpu_data.pitch,
                                           data.mpu_data.roll,
                                           data.mpu_data.yaw);
                    break;
                // ... 其他传感器
            }
        }
    }
}

// 初始化
sensor_queue = xQueueCreate(20, sizeof(sensor_data_t));
xTaskCreate(esp32_send_task, "esp32_tx", 256, NULL, 5, NULL);
```

### 2.4 中断同步（使用二值信号量）

```c
// UART接收中断 → 数据解析任务
SemaphoreHandle_t uart_rx_sem;

void USART3_IRQHandler(void) {
    // HAL中断处理
    HAL_UART_IRQHandler(&huart3);

    // 释放信号量，通知数据处理任务
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(uart_rx_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void uart_process_task(void* param) {
    while(1) {
        if (xSemaphoreTake(uart_rx_sem, portMAX_DELAY) == pdTRUE) {
            // 处理接收到的数据
            ESP32_COMM_ProcessCommand((const char*)uart3_rx_buffer);
        }
    }
}

// 初始化
uart_rx_sem = xSemaphoreCreateBinary();
xTaskCreate(uart_process_task, "uart_rx", 256, NULL, 5, NULL);
```

---

## 三、FreeRTOS配置（STM32F103）

### 3.1 `FreeRTOSConfig.h` 关键配置

```c
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

// 基础配置
#define configUSE_PREEMPTION              1        // 抢占式调度
#define configUSE_IDLE_HOOK               0        // 不使用空闲钩子
#define configUSE_TICK_HOOK               0        // 不使用tick钩子
#define configCPU_CLOCK_HZ                72000000 // STM32F103主频
#define configTICK_RATE_HZ                1000     // 1ms tick
#define configMAX_PRIORITIES              7        // 最大优先级（0-6）
#define configMINIMAL_STACK_SIZE          128      // 最小栈（512字节）

// 内存配置
#define configTOTAL_HEAP_SIZE             12*1024  // 12KB堆
#define configSUPPORT_STATIC_ALLOCATION   0
#define configSUPPORT_DYNAMIC_ALLOCATION  1

// 钩子函数
#define configCHECK_FOR_STACK_OVERFLOW    2        // 栈溢出检测
#define configUSE_MALLOC_FAILED_HOOK      1        // 内存分配失败检测

// API函数
#define INCLUDE_vTaskPrioritySet          1
#define INCLUDE_uxTaskPriorityGet         1
#define INCLUDE_vTaskDelete               1
#define INCLUDE_vTaskSuspend              1
#define INCLUDE_xTaskGetSchedulerState    1

// 中断配置
#define configPRIO_BITS                   4        // STM32F103优先级位4位
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY    15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY     (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY  (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

#endif
```

### 3.2 添加FreeRTOS到项目

**方法1：使用STM32CubeMX（推荐）**
1. 打开CubeMX，加载你的`.ioc`文件
2. Middleware → FREERTOS → Interface → Enable
3. 配置参数和任务
4. 生成代码

**方法2：手动添加**
1. 下载FreeRTOS源码（https://github.com/FreeRTOS/FreeRTOS）
2. 复制以下文件到项目：
   - `Source/` 下的所有`.c`文件（除了`port.c`）
   - `Source/portable/GCC/ARM_CM3/port.c` 和 `portmacro.h`
   - `Source/include/` 下的所有`.h`文件
3. 创建`FreeRTOSConfig.h`
4. 修改启动文件：启动`SVC_Handler`改为`vPortSVCHandler`，`PendSV_Handler`改为`xPortPendSVHandler`

---

## 四、常见问题

### 4.1 栈大小设置

```c
// 栈大小 = 预估栈使用 / 4字节
xTaskCreate(task, "task", 128, NULL, 1, NULL);  // 128 * 4 = 512字节栈

// 如何调试栈溢出？
// 1. 启用栈溢出检测
#define configCHECK_FOR_STACK_OVERFLOW 2

// 2. 实现钩子函数
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    printf("[ERROR] Stack overflow: %s\r\n", pcTaskName);
    while(1);  // 死循环，方便调试
}

// 3. 查看栈使用情况
UBaseType_t stack_high_watermark = uxTaskGetStackHighWaterMark(NULL);
printf("剩余栈: %u 字\r\n", stack_high_watermark);
```

### 4.2 优先级分配原则

```
优先级0（最低）：统计任务
优先级1-2：后台任务（日志统计）
优先级3-4：普通任务（传感器采集）
优先级5：通信任务（UART、MQTT）
优先级6（最高）：实时任务（按键扫描、安全检测）
```

### 4.3 死锁避免

```c
// 错误示例（死锁）
SemaphoreHandle_t mutex1, mutex2;

void task1(void* param) {
    xSemaphoreTake(mutex1, portMAX_DELAY);
    vTaskDelay(1);  // 切换任务
    xSemaphoreTake(mutex2, portMAX_DELAY);
    // ...
    xSemaphoreGive(mutex2);
    xSemaphoreGive(mutex1);
}

void task2(void* param) {
    xSemaphoreTake(mutex2, portMAX_DELAY);
    vTaskDelay(1);
    xSemaphoreTake(mutex1, portMAX_DELAY);
    // ...
    xSemaphoreGive(mutex1);
    xSemaphoreGive(mutex2);
}

// 解决方案：按固定顺序获取锁
void task1(void* param) {
    xSemaphoreTake(mutex1, portMAX_DELAY);
    xSemaphoreTake(mutex2, portMAX_DELAY);
    // ...
    xSemaphoreGive(mutex2);
    xSemaphoreGive(mutex1);
}

void task2(void* param) {
    xSemaphoreTake(mutex1, portMAX_DELAY);  // 顺序相同
    xSemaphoreTake(mutex2, portMAX_DELAY);
    // ...
    xSemaphoreGive(mutex2);
    xSemaphoreGive(mutex1);
}
```

---

## 五、对比速查表

| 功能 | RT-Thread | FreeRTOS |
|------|-----------|----------|
| 创建任务 | `rt_thread_create()` | `xTaskCreate()` |
| 删除任务 | `rt_thread_delete()` | `vTaskDelete()` |
| 延时 | `rt_thread_delay()` | `vTaskDelay()` |
| 挂起任务 | `rt_thread_suspend()` | `vTaskSuspend()` |
| 恢复任务 | `rt_thread_resume()` | `vTaskResume()` |
| 创建信号量 | `rt_sem_create()` | `xSemaphoreCreateBinary()` |
| 创建互斥量 | `rt_mutex_create()` | `xSemaphoreCreateMutex()` |
| 创建事件集 | `rt_event_create()` | `xEventGroupCreate()` |
| 创建队列 | `rt_mq_create()` | `xQueueCreate()` |
| 创建定时器 | `rt_timer_create()` | `xTimerCreate()` |
| 获取tick | `rt_tick_get()` | `xTaskGetTickCount()` |
| 获取时间 | `rt_tick_get_millisecond()` | `xTaskGetTickCount() * portTICK_PERIOD_MS` |

---

## 六、明天移植实战计划

### 阶段1：环境准备（1小时）
- [ ] 在STM32CubeMX中启用FreeRTOS
- [ ] 配置基础参数（Tick频率、堆大小）
- [ ] 生成代码并编译通过

### 阶段2：任务迁移（2小时）
- [ ] 创建6个传感器任务（按键、MPU、MAX30102、DHT11、MQ2、GPS）
- [ ] 移除原有的`scheduler.c`
- [ ] 测试各任务运行状态

### 阶段3：IPC通信（2小时）
- [ ] 添加传感器数据队列
- [ ] 添加ESP32发送任务
- [ ] 添加UART接收信号量

### 阶段4：调试优化（1小时）
- [ ] 启用栈溢出检测
- [ ] 调整任务优先级和栈大小
- [ ] 性能测试

---

**文档版本**：v1.0
**创建日期**：2025-01-19
**适用项目**：智能安全帽（STM32F103 + FreeRTOS）
