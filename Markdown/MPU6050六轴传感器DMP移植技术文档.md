# MPU6050六轴传感器DMP移植技术文档

> 作者：Claude & 用户 | 日期：2026-01-04 | 项目：Smart Helmet FreeRTOS

---

## 目录

1. [MPU6050传感器概述](#1-mpu6050传感器概述)
2. [I2C通信协议与时序分析](#2-i2c通信协议与时序分析)
3. [无DMP模式：原始数据获取](#3-无dmp模式原始数据获取)
4. [DMP数字运动处理器](#4-dmp数字运动处理器)
5. [FreeRTOS任务架构设计](#5-freertos任务架构设计)
6. [核心代码分析](#6-核心代码分析)
7. [常见问题与解决方案](#7-常见问题与解决方案)
8. [总结与优化建议](#8-总结与优化建议)

---

## 1. MPU6050传感器概述

### 1.1 硬件特性

MPU6050是InvenSense（现TDK）公司推出的全球首款整合三轴陀螺仪和三轴加速度计的六轴运动处理组件。其主要特性包括：

| 参数 | 规格 |
|------|------|
| 工作电压 | 2.375V ~ 3.46V |
| 陀螺仪量程 | ±250/500/1000/2000 °/s |
| 加速度计量程 | ±2/4/8/16 g |
| 通信接口 | I2C (最高400kHz) / SPI |
| 内置DMP | 支持四元数输出 |
| ADC分辨率 | 16位 |
| I2C地址 | 0x68 (AD0=GND) / 0x69 (AD0=VCC) |

### 1.2 内部架构

```
                    ┌─────────────────────────────────────┐
                    │            MPU6050                   │
                    │  ┌─────────┐    ┌─────────┐         │
                    │  │ 3轴陀螺仪 │    │3轴加速度计│         │
                    │  │ (角速度) │    │ (线加速度)│         │
                    │  └────┬────┘    └────┬────┘         │
                    │       │              │               │
                    │       ▼              ▼               │
                    │  ┌─────────────────────┐            │
                    │  │    16位 ADC转换器    │            │
                    │  └──────────┬──────────┘            │
                    │             │                        │
                    │             ▼                        │
                    │  ┌─────────────────────┐            │
                    │  │   DMP (可选处理)     │◄── 固件    │
                    │  └──────────┬──────────┘            │
                    │             │                        │
                    │             ▼                        │
                    │  ┌─────────────────────┐            │
                    │  │      FIFO缓冲区      │            │
                    │  │      (1024字节)      │            │
                    │  └──────────┬──────────┘            │
                    │             │                        │
                    │             ▼                        │
                    │  ┌─────────────────────┐            │
                    │  │   I2C/SPI 接口      │            │
                    │  └──────────┬──────────┘            │
                    └─────────────┼─────────────────────────┘
                                  │
                                  ▼
                              MCU (STM32)
```

---

## 2. I2C通信协议与时序分析

### 2.1 I2C基础时序

I2C（Inter-Integrated Circuit）是一种两线制串行通信协议，由飞利浦公司开发。它使用两根信号线：

- **SCL（Serial Clock）**：时钟线，由主机产生
- **SDA（Serial Data）**：数据线，双向传输

#### 2.1.1 起始和停止条件

```
起始条件(START):  SCL高电平时，SDA从高到低跳变
停止条件(STOP):   SCL高电平时，SDA从低到高跳变

        START                           STOP
          │                               │
    ──────┐                         ┌─────────
SDA       └─────────────────────────┘
    ────────┐                     ┌───────────
SCL         └─────────────────────┘
```

#### 2.1.2 数据传输时序

```
    ┌───┐   ┌───┐   ┌───┐   ┌───┐   ┌───┐   ┌───┐   ┌───┐   ┌───┐
SCL │   │   │   │   │   │   │   │   │   │   │   │   │   │   │   │
────┘   └───┘   └───┘   └───┘   └───┘   └───┘   └───┘   └───┘   └──
     ←───────────────────────────────────────────────────────────→
SDA  │ D7  │ D6  │ D5  │ D4  │ D3  │ D2  │ D1  │ D0  │ ACK │
     └─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

### 2.2 MPU6050的I2C通信

MPU6050支持标准模式（100kHz）和快速模式（400kHz）的I2C通信。在本项目中，我们使用标准模式以确保稳定性。

#### 2.2.1 寄存器读取时序

读取MPU6050寄存器需要两个I2C事务：

```
写入寄存器地址：
[START] [设备地址+W] [ACK] [寄存器地址] [ACK] [STOP]

读取寄存器数据：
[START] [设备地址+R] [ACK] [数据] [NACK] [STOP]
```

#### 2.2.2 HAL库实现

```c
// I2C读取函数封装
static HAL_StatusTypeDef mpu6050_i2c_read(uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(&hi2c1,          // I2C句柄
                            MPU6050_ADDR,     // 设备地址 0xD0 (0x68<<1)
                            reg,              // 寄存器地址
                            I2C_MEMADD_SIZE_8BIT,  // 8位寄存器地址
                            data,             // 数据缓冲区
                            len,              // 读取长度
                            MPU6050_I2C_TIMEOUT);  // 超时时间
}

// I2C写入函数封装
static HAL_StatusTypeDef mpu6050_i2c_write(uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Write(&hi2c1,
                             MPU6050_ADDR,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             data,
                             len,
                             MPU6050_I2C_TIMEOUT);
}
```

### 2.3 I2C时序对系统的影响

#### 2.3.1 通信速率选择

| 模式 | 速率 | 传输6字节耗时 | 适用场景 |
|------|------|---------------|----------|
| 标准模式 | 100kHz | ~0.5ms | 稳定性优先 |
| 快速模式 | 400kHz | ~0.15ms | 高采样率场景 |

#### 2.3.2 时序问题与解决

**问题1：上拉电阻选择**

I2C总线需要外部上拉电阻，阻值影响信号质量：
- 阻值过大：上升沿变缓，可能导致通信失败
- 阻值过小：功耗增加，驱动能力要求高

推荐值：
- 100kHz模式：4.7kΩ ~ 10kΩ
- 400kHz模式：2.2kΩ ~ 4.7kΩ

**问题2：FreeRTOS中的I2C访问**

在多任务环境中，I2C访问需要互斥保护：

```c
// 错误示例：无保护的I2C访问
void task1(void) {
    HAL_I2C_Mem_Read(...);  // 可能被task2打断
}

// 正确示例：使用Mutex保护
void task1(void) {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    HAL_I2C_Mem_Read(...);
    xSemaphoreGive(i2c_mutex);
}
```

---

## 3. 无DMP模式：原始数据获取

### 3.1 原始数据读取原理

不使用DMP时，需要直接读取MPU6050的原始寄存器数据，然后在MCU端进行姿态解算。

#### 3.1.1 关键寄存器

| 寄存器地址 | 名称 | 数据内容 |
|------------|------|----------|
| 0x3B-0x40 | ACCEL_XOUT_H/L等 | 加速度计原始数据 |
| 0x41-0x42 | TEMP_OUT_H/L | 温度原始数据 |
| 0x43-0x48 | GYRO_XOUT_H/L等 | 陀螺仪原始数据 |
| 0x6B | PWR_MGMT_1 | 电源管理 |
| 0x75 | WHO_AM_I | 设备ID (返回0x68) |

#### 3.1.2 原始数据读取代码

```c
// 读取加速度计原始数据
static MPU6050_Status_t mpu6050_read_accel_raw(short *ax, short *ay, short *az) {
    uint8_t buf[6];

    // 一次性读取6字节（X/Y/Z各2字节，高字节在前）
    if (mpu6050_i2c_read(MPU6050_ACCEL_XOUT_H, buf, 6) == HAL_OK) {
        // 合并高低字节（大端序）
        *ax = (int16_t)((buf[0] << 8) | buf[1]);
        *ay = (int16_t)((buf[2] << 8) | buf[3]);
        *az = (int16_t)((buf[4] << 8) | buf[5]);
        return MPU6050_OK;
    }
    return MPU6050_ERROR;
}

// 转换为物理值（以±2g量程为例）
// 灵敏度：16384 LSB/g
float accel_x_g = (float)ax / 16384.0f;  // 单位：g
float accel_y_g = (float)ay / 16384.0f;
float accel_z_g = (float)az / 16384.0f;
```

### 3.2 MCU端姿态解算

不使用DMP时，需要在MCU端实现姿态解算算法。常用方法包括：

#### 3.2.1 互补滤波（简单但精度一般）

```c
// 互补滤波原理：
// 陀螺仪：短期精确，长期漂移
// 加速度计：长期稳定，短期噪声大
// 结合两者优点

#define ALPHA 0.98f  // 互补滤波系数

void complementary_filter(float *pitch, float *roll,
                          float ax, float ay, float az,
                          float gx, float gy, float gz,
                          float dt) {
    // 加速度计计算角度
    float pitch_acc = atan2f(ay, sqrtf(ax*ax + az*az)) * 57.3f;
    float roll_acc = atan2f(-ax, az) * 57.3f;

    // 陀螺仪积分
    *pitch += gx * dt;
    *roll += gy * dt;

    // 互补滤波融合
    *pitch = ALPHA * (*pitch) + (1.0f - ALPHA) * pitch_acc;
    *roll = ALPHA * (*roll) + (1.0f - ALPHA) * roll_acc;
}
```

#### 3.2.2 卡尔曼滤波（复杂但精度高）

卡尔曼滤波是一种递归的最优估计算法，需要更多的计算资源：

```c
// 简化的一维卡尔曼滤波
typedef struct {
    float Q_angle;    // 过程噪声协方差
    float Q_bias;     // 陀螺仪漂移噪声
    float R_measure;  // 测量噪声协方差
    float angle;      // 状态估计
    float bias;       // 陀螺仪漂移估计
    float P[2][2];    // 误差协方差矩阵
} Kalman_t;

float kalman_update(Kalman_t *kf, float newAngle, float newRate, float dt) {
    // 预测步骤
    kf->angle += dt * (newRate - kf->bias);
    kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // 更新步骤
    float S = kf->P[0][0] + kf->R_measure;
    float K[2] = {kf->P[0][0] / S, kf->P[1][0] / S};
    float y = newAngle - kf->angle;
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;

    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}
```

### 3.3 无DMP模式的缺点

| 缺点 | 说明 |
|------|------|
| CPU负担重 | 需要MCU执行浮点运算、三角函数 |
| 精度有限 | 简单算法难以消除陀螺仪漂移 |
| 实时性差 | 复杂算法计算耗时长 |
| 功耗高 | MCU持续运算增加功耗 |
| 调参困难 | 滤波参数需要针对应用场景调整 |

---

## 4. DMP数字运动处理器

### 4.1 DMP的作用与优势

DMP（Digital Motion Processor）是MPU6050内置的专用处理器，可以直接输出四元数和欧拉角，具有以下优势：

| 优势 | 说明 |
|------|------|
| 减轻MCU负担 | 姿态解算在DMP内部完成 |
| 高精度 | InvenSense优化的融合算法 |
| 低功耗 | DMP功耗远低于MCU运算 |
| 抗漂移 | 内置漂移补偿算法 |
| 内置功能 | 计步器、手势识别、运动检测 |

### 4.2 DMP固件加载

DMP需要加载固件才能工作，固件存储在`dmpKey.h`和`dmpmap.h`中：

```c
// DMP初始化流程
uint8_t mpu_dmp_init(void) {
    uint8_t res;

    // 1. 复位MPU6050
    mpu_write_byte(PWR_MGMT_1, 0x80);
    delay_ms(100);

    // 2. 唤醒MPU6050
    mpu_write_byte(PWR_MGMT_1, 0x00);
    delay_ms(50);

    // 3. 设置传感器
    res = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if (res) return 1;

    // 4. 配置FIFO
    res = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if (res) return 2;

    // 5. 设置采样率
    res = mpu_set_sample_rate(100);  // 100Hz
    if (res) return 3;

    // 6. 加载DMP固件（约3KB）
    res = dmp_load_motion_driver_firmware();
    if (res) return 4;

    // 7. 设置方向矩阵
    res = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    if (res) return 5;

    // 8. 使能DMP功能
    res = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT |      // 6轴四元数
                             DMP_FEATURE_SEND_RAW_ACCEL |  // 原始加速度
                             DMP_FEATURE_SEND_CAL_GYRO |   // 校准后陀螺仪
                             DMP_FEATURE_GYRO_CAL |        // 陀螺仪校准
                             DMP_FEATURE_PEDOMETER);       // 计步器
    if (res) return 6;

    // 9. 设置DMP输出速率
    res = dmp_set_fifo_rate(100);  // 100Hz
    if (res) return 7;

    // 10. 使能DMP
    res = mpu_set_dmp_state(1);
    if (res) return 8;

    return 0;
}
```

### 4.3 DMP数据读取

DMP输出的是四元数，需要转换为欧拉角：

```c
// 从DMP读取姿态数据
uint8_t mpu_dmp_get_data(float *pitch, float *roll, float *yaw) {
    short gyro[3], accel[3], sensors;
    unsigned long timestamp;
    unsigned char more;
    long quat[4];
    float q0, q1, q2, q3;

    // 从FIFO读取DMP数据
    if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more)) {
        return 1;
    }

    // 四元数归一化（DMP输出的是Q30格式）
    q0 = quat[0] / (float)(1 << 30);
    q1 = quat[1] / (float)(1 << 30);
    q2 = quat[2] / (float)(1 << 30);
    q3 = quat[3] / (float)(1 << 30);

    // 四元数转欧拉角
    *pitch = asinf(-2.0f * (q1*q3 - q0*q2)) * 57.3f;
    *roll = atan2f(2.0f * (q0*q1 + q2*q3),
                   1.0f - 2.0f * (q1*q1 + q2*q2)) * 57.3f;
    *yaw = atan2f(2.0f * (q0*q3 + q1*q2),
                  1.0f - 2.0f * (q2*q2 + q3*q3)) * 57.3f;

    return 0;
}
```

### 4.4 DMP内置功能

#### 4.4.1 计步器

```c
// 获取DMP计步器数据
unsigned long steps;
dmp_get_pedometer_step_count(&steps);

// 重置步数
dmp_set_pedometer_step_count(0);
```

#### 4.4.2 运动检测

DMP可以检测设备是否静止，用于省电和自动校准：

```c
// 配置运动检测阈值
dmp_set_tap_thresh(TAP_XYZ, 250);  // 敲击检测
dmp_set_shake_reject_thresh(200); // 抖动抑制
```

---

## 5. FreeRTOS任务架构设计

### 5.1 任务设计原则

在RTOS环境中设计MPU6050任务，需要考虑以下因素：

1. **采样率**：DMP默认100Hz输出，任务周期应匹配
2. **优先级**：姿态数据对实时性要求中等，优先级设为2
3. **栈大小**：DMP + 浮点运算需要较大栈空间（512字节）
4. **线程安全**：共享数据需要Mutex保护

### 5.2 任务状态机

```
                    ┌─────────────────────────────────────┐
                    │                                     │
                    ▼                                     │
    ┌──────────┐  初始化成功  ┌──────────┐  start()  ┌──────────┐
    │  初始化   │ ──────────► │  挂起态   │ ────────► │  运行态   │
    └──────────┘             └──────────┘           └──────────┘
         │                        ▲                      │
         │ 初始化失败              │ stop()              │
         ▼                        └──────────────────────┘
    ┌──────────┐
    │  错误态   │
    └──────────┘
```

### 5.3 数据结构设计

```c
// MPU6050共享数据结构
typedef struct {
    // DMP姿态角（度）
    float pitch;            // 俯仰角 -90° ~ +90°
    float roll;             // 横滚角 -180° ~ +180°
    float yaw;              // 航向角 -180° ~ +180°

    // 加速度（g）
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    // 温度（°C）
    float temperature;

    // 计步器
    uint32_t step_count;    // 步数
    float distance;         // 距离（米）

    // 摔倒检测
    uint8_t fall_flag;      // 摔倒标志
    uint8_t collision_flag; // 碰撞标志
    int svm;                // 加速度矢量和

    // 状态
    uint8_t dmp_enabled;    // DMP使能标志
    uint8_t is_valid;       // 数据有效标志
} MPU6050_Data_t;
```

---

## 6. 核心代码分析

### 6.1 任务主函数

```c
static void mpu6050_task(void *arg) {
    uint32_t update_count = 0;
    uint32_t error_count = 0;

    // 错峰启动，避免与其他任务冲突
    vTaskDelay(pdMS_TO_TICKS(400));

    // 初始化DMP
    MPU6050_Status_t init_status = mpu6050_dmp_init_internal();

    // 更新DMP状态
    xSemaphoreTake(mpu6050_mutex, portMAX_DELAY);
    mpu6050_shared_data.dmp_enabled = (init_status == MPU6050_OK) ? 1 : 0;
    xSemaphoreGive(mpu6050_mutex);

    if (init_status != MPU6050_OK) {
        uart_printf_dma(&huart1, "[MPU6050] Init failed!\r\n");
    }

    for (;;) {
        // 检查运行状态，未运行则挂起
        if (!mpu6050_running || !mpu6050_shared_data.dmp_enabled) {
            vTaskSuspend(NULL);  // 挂起自己
            continue;
        }

        // 更新传感器数据
        MPU6050_Status_t status = mpu6050_update_internal();

        if (status == MPU6050_OK) {
            update_count++;

            // 通知OLED更新显示
            oled_update_mpu6050(mpu6050_shared_data.pitch,
                               mpu6050_shared_data.roll,
                               mpu6050_shared_data.yaw,
                               mpu6050_shared_data.step_count,
                               1);

            // 摔倒/碰撞报警
            if (mpu6050_shared_data.fall_flag ||
                mpu6050_shared_data.collision_flag) {
                led_set_mode(LED_EVENT_ERROR);
            }
        } else {
            error_count++;
        }

        // 100ms采样周期（匹配DMP 100Hz输出）
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

### 6.2 摔倒检测算法

```c
static void mpu6050_update_fall_detection(void) {
    // 计算SVM（加速度矢量和）
    float ax = mpu6050_shared_data.accel_x_g * 16384.0f;
    float ay = mpu6050_shared_data.accel_y_g * 16384.0f;
    float az = mpu6050_shared_data.accel_z_g * 16384.0f;
    mpu6050_shared_data.svm = (int)sqrtf(ax*ax + ay*ay + az*az);

    // 角度检测：倾斜超过60度判定为摔倒
    if (fabsf(mpu6050_shared_data.pitch) > 60.0f ||
        fabsf(mpu6050_shared_data.roll) > 60.0f) {
        mpu6050_shared_data.fall_flag = 1;
    } else {
        mpu6050_shared_data.fall_flag = 0;
    }

    // 碰撞检测：加速度突变
    if (mpu6050_shared_data.svm > 30000 ||
        mpu6050_shared_data.svm < 8000) {
        mpu6050_shared_data.collision_flag = 1;
    }
}
```

### 6.3 软件计步器

```c
static void mpu6050_update_step_counter(void) {
    // 从DMP读取步数
    unsigned long dmp_steps = 0;
    dmp_get_pedometer_step_count(&dmp_steps);

    // 软件辅助计步（提高灵敏度）
    float magnitude = sqrtf(
        mpu6050_shared_data.accel_x_g * mpu6050_shared_data.accel_x_g +
        mpu6050_shared_data.accel_y_g * mpu6050_shared_data.accel_y_g +
        mpu6050_shared_data.accel_z_g * mpu6050_shared_data.accel_z_g
    );

    float accel_change = fabsf(magnitude - last_accel_magnitude);
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // 加速度变化超过阈值且间隔足够
    if (accel_change > 0.15f &&
        (current_time - last_step_time) > 250) {
        soft_step_count++;
        last_step_time = current_time;
    }

    last_accel_magnitude = magnitude;

    // 使用较大值
    mpu6050_shared_data.step_count =
        (soft_step_count > dmp_steps) ? soft_step_count : dmp_steps;
    mpu6050_shared_data.distance =
        mpu6050_shared_data.step_count * 0.35f;  // 步距0.35米
}
```

---

## 7. 常见问题与解决方案

### 7.1 DMP初始化失败

**现象**：`[MPU6050] DMP Init Failed! Code=10`

**原因**：I2C通信失败，找不到MPU6050

**解决方案**：
1. 检查硬件连接（SCL/SDA是否接反）
2. 检查上拉电阻是否存在
3. 检查I2C地址（AD0引脚电平）
4. 使用I2C扫描确认设备存在

```c
// I2C扫描代码
for (uint16_t addr = 0x00; addr <= 0xFE; addr += 2) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, addr, 1, 10) == HAL_OK) {
        uart_printf_dma(&huart1, "Found: 0x%02X\r\n", addr);
    }
}
```

### 7.2 数据读取错误率高

**现象**：`[MPU6050] Read error` 频繁出现

**原因**：
1. DMP FIFO溢出
2. I2C通信不稳定
3. 采样率与任务周期不匹配

**解决方案**：
1. 降低DMP输出速率或提高任务频率
2. 增加I2C速率
3. 偶发错误是正常的，只要成功率>70%即可

### 7.3 堆栈溢出

**现象**：LED疯狂闪烁，系统死机

**原因**：任务栈空间不足

**解决方案**：
1. 增加任务栈大小（512 → 768或更大）
2. 减少局部变量使用
3. 使用`uxTaskGetStackHighWaterMark()`监控栈使用

---

## 8. 总结与优化建议

### 8.1 DMP vs 无DMP对比

| 方面 | 无DMP | 有DMP |
|------|-------|-------|
| MCU负担 | 重（需要自行解算） | 轻（DMP内部处理） |
| 精度 | 取决于算法质量 | 高（专业算法） |
| 功耗 | 高 | 低 |
| 开发难度 | 高 | 中（需要移植库） |
| 灵活性 | 高（可自定义算法） | 中（功能固定） |

### 8.2 优化建议

1. **降低采样率**：如果应用不需要100Hz，可降至50Hz减少CPU负担
2. **使用DMA**：I2C使用DMA模式进一步释放CPU
3. **动态调整**：静止时降低采样率，运动时提高
4. **缓存复用**：多个任务共享姿态数据，避免重复读取
5. **错误恢复**：连续错误时自动重置DMP FIFO

### 8.3 本项目实现总结

本项目成功将MPU6050 + DMP移植到FreeRTOS环境，实现了：

- 三轴姿态实时输出（Pitch/Roll/Yaw）
- 计步器功能（DMP + 软件辅助）
- 摔倒/碰撞检测
- 与OLED界面联动显示
- 按键控制启停

整体架构遵循RTOS最佳实践：Mutex保护共享数据、任务挂起/恢复机制、错峰启动避免冲突。

---

> 文档结束 | 字数：约4500字
