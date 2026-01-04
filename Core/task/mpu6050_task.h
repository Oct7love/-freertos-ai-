//
// Created by Claude on 2026/1/4.
// MPU6050 DMP FreeRTOS 任务接口
//

#ifndef FREERTOS_TEST_MPU6050_TASK_H
#define FREERTOS_TEST_MPU6050_TASK_H

#include "APP/bsp_system.h"

// ==================== 配置参数 ====================
#define MPU6050_ADDR                0xD0            // I2C 地址 (AD0=GND)
#define MPU6050_I2C_TIMEOUT         100             // I2C 超时 (ms)

// 计步器配置
#define MPU6050_STEP_LENGTH         0.35f           // 步距 (米)
#define MPU6050_SOFT_STEP_THRESHOLD 0.15f           // 软件计步阈值 (g)
#define MPU6050_STEP_MIN_INTERVAL   250             // 最小步伐间隔 (ms)

// 摔倒检测阈值
#define MPU6050_FALL_ANGLE_THRESHOLD    60.0f       // 倾斜角度阈值 (度)
#define MPU6050_COLLISION_SVM_HIGH      30000       // 碰撞检测上限
#define MPU6050_COLLISION_SVM_LOW       8000        // 碰撞检测下限

// ==================== 状态枚举 ====================
typedef enum {
    MPU6050_OK = 0,
    MPU6050_ERROR,
    MPU6050_TIMEOUT,
    MPU6050_NOT_FOUND,
    MPU6050_DMP_ERROR
} MPU6050_Status_t;

// ==================== 数据结构 ====================
typedef struct {
    // DMP 姿态角 (度)
    float pitch;            // 俯仰角 -90° ~ +90°
    float roll;             // 横滚角 -180° ~ +180°
    float yaw;              // 航向角 -180° ~ +180°

    // 加速度 (g)
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    // 角速度 (°/s)
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    // 温度 (°C)
    float temperature;

    // 计步器
    uint32_t step_count;    // 步数
    float distance;         // 距离 (米)

    // 摔倒检测
    uint8_t fall_flag;      // 摔倒标志
    uint8_t collision_flag; // 碰撞标志
    int svm;                // 加速度矢量和

    // 状态
    uint8_t dmp_enabled;    // DMP 使能标志
    uint8_t is_valid;       // 数据有效标志
} MPU6050_Data_t;

// ==================== 任务接口 ====================
// 初始化 MPU6050 任务（默认挂起状态）
void mpu6050_task_init(void);

// 启动/停止任务
void mpu6050_task_start(void);
void mpu6050_task_stop(void);
uint8_t mpu6050_is_running(void);

// 获取数据（线程安全）
void mpu6050_get_attitude(float *pitch, float *roll, float *yaw, uint8_t *valid);
void mpu6050_get_step_data(uint32_t *steps, float *distance);
uint8_t mpu6050_is_fall_detected(void);
uint8_t mpu6050_is_collision_detected(void);
float mpu6050_get_temperature(void);

// 计步器控制
void mpu6050_reset_step_count(void);
void mpu6050_set_step_length(float length);

#endif //FREERTOS_TEST_MPU6050_TASK_H
