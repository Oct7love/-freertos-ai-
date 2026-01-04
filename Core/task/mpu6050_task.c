//
// Created by Claude on 2026/1/4.
// MPU6050 DMP FreeRTOS 任务实现
//

#include "mpu6050_task.h"


// ==================== MPU6050 寄存器 ====================
#define MPU6050_WHO_AM_I        0x75
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_PWR_MGMT_1      0x6B

// ==================== 共享数据（Mutex 保护） ====================
static MPU6050_Data_t mpu6050_shared_data = {0};

// ==================== IPC 对象 ====================
static SemaphoreHandle_t mpu6050_mutex = NULL;
static TaskHandle_t mpu6050_task_handle = NULL;
static volatile uint8_t mpu6050_running = 0;

// ==================== 私有变量 ====================
static float step_length = MPU6050_STEP_LENGTH;
static uint32_t soft_step_count = 0;
static float last_accel_magnitude = 1.0f;
static uint32_t last_step_time = 0;
static uint8_t collision_counter = 10;

// ==================== 私有函数声明 ====================
static MPU6050_Status_t mpu6050_read_accel_raw(short *ax, short *ay, short *az);
static MPU6050_Status_t mpu6050_read_temp_raw(short *temp);
static void mpu6050_calculate_physical_values(void);
static void mpu6050_update_fall_detection(void);
static void mpu6050_update_step_counter(void);

// ==================== I2C 读写函数 ====================
static HAL_StatusTypeDef mpu6050_i2c_read(uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, data, len, MPU6050_I2C_TIMEOUT);
}

static HAL_StatusTypeDef mpu6050_i2c_write(uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, data, len, MPU6050_I2C_TIMEOUT);
}

// ==================== DMP 初始化 ====================
static MPU6050_Status_t mpu6050_dmp_init_internal(void) {
    uint8_t res;
    uint8_t cmd;

    uart_printf_dma(&huart1, "[MPU6050] Waiting for power stable...\r\n");
    vTaskDelay(pdMS_TO_TICKS(500));

    // 软复位 MPU6050
    cmd = 0x80;
    mpu6050_i2c_write(MPU6050_PWR_MGMT_1, &cmd, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // 唤醒 MPU6050
    cmd = 0x00;
    mpu6050_i2c_write(MPU6050_PWR_MGMT_1, &cmd, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // 调用 DMP 初始化
    res = mpu_dmp_init();

    if (res == 0) {
        uart_printf_dma(&huart1, "[MPU6050] DMP Init Success!\r\n");
        return MPU6050_OK;
    } else {
        uart_printf_dma(&huart1, "[MPU6050] DMP Init Failed! Code=%d\r\n", res);

        // 错误说明
        switch(res) {
            case 1:  uart_printf_dma(&huart1, "  - Set sensors failed\r\n"); break;
            case 2:  uart_printf_dma(&huart1, "  - Configure FIFO failed\r\n"); break;
            case 3:  uart_printf_dma(&huart1, "  - Set sample rate failed\r\n"); break;
            case 4:  uart_printf_dma(&huart1, "  - Load DMP firmware failed\r\n"); break;
            case 5:  uart_printf_dma(&huart1, "  - Set orientation failed\r\n"); break;
            case 6:  uart_printf_dma(&huart1, "  - Enable features failed\r\n"); break;
            case 7:  uart_printf_dma(&huart1, "  - Set FIFO rate failed\r\n"); break;
            case 8:  uart_printf_dma(&huart1, "  - Self test failed\r\n"); break;
            case 10: uart_printf_dma(&huart1, "  - MPU6050 not found!\r\n"); break;
            default: uart_printf_dma(&huart1, "  - Unknown error\r\n"); break;
        }

        // I2C 扫描
        if (res == 10) {
            uart_printf_dma(&huart1, "[I2C Scan] Scanning I2C1...\r\n");
            uint8_t found = 0;
            for (uint16_t addr = 0x00; addr <= 0xFE; addr += 2) {
                if (HAL_I2C_IsDeviceReady(&hi2c1, addr, 1, 10) == HAL_OK) {
                    uart_printf_dma(&huart1, "  Found: 0x%02X\r\n", addr);
                    found++;
                }
            }
            if (found == 0) {
                uart_printf_dma(&huart1, "  No device found!\r\n");
            }
        }

        return MPU6050_DMP_ERROR;
    }
}

// ==================== 更新传感器数据 ====================
static MPU6050_Status_t mpu6050_update_internal(void) {
    uint8_t res;
    float new_pitch, new_roll, new_yaw;
    static float last_pitch = 0, last_roll = 0, last_yaw = 0;
    static uint8_t first_read = 1;

    if (!mpu6050_shared_data.dmp_enabled) {
        return MPU6050_ERROR;
    }

    // 读取 DMP 姿态数据
    res = mpu_dmp_get_data(&new_pitch, &new_roll, &new_yaw);

    if (res != 0) {
        mpu6050_shared_data.is_valid = 0;
        return MPU6050_ERROR;
    }

    // 数据有效性检查（过滤 >180° 突变）
    if (!first_read) {
        if (fabsf(new_pitch - last_pitch) > 180.0f ||
            fabsf(new_roll - last_roll) > 180.0f ||
            fabsf(new_yaw - last_yaw) > 180.0f) {
            return MPU6050_ERROR;
        }
    }

    // 更新姿态数据
    mpu6050_shared_data.pitch = new_pitch;
    mpu6050_shared_data.roll = new_roll;
    mpu6050_shared_data.yaw = new_yaw;
    last_pitch = new_pitch;
    last_roll = new_roll;
    last_yaw = new_yaw;
    first_read = 0;
    mpu6050_shared_data.is_valid = 1;

    // 读取原始加速度
    short ax, ay, az;
    mpu6050_read_accel_raw(&ax, &ay, &az);

    // 读取温度
    short temp_raw;
    mpu6050_read_temp_raw(&temp_raw);
    mpu6050_shared_data.temperature = 36.53f + ((float)temp_raw) / 340.0f;

    // 计算物理值
    mpu6050_calculate_physical_values();

    // 更新摔倒检测
    mpu6050_update_fall_detection();

    // 更新计步器
    mpu6050_update_step_counter();

    return MPU6050_OK;
}

// ==================== MPU6050 任务 ====================
static void mpu6050_task(void *arg) {
    uint32_t update_count = 0;
    uint32_t error_count = 0;

    // 错峰启动
    vTaskDelay(pdMS_TO_TICKS(400));

    // 初始化 DMP
    MPU6050_Status_t init_status = mpu6050_dmp_init_internal();

    xSemaphoreTake(mpu6050_mutex, portMAX_DELAY);
    mpu6050_shared_data.dmp_enabled = (init_status == MPU6050_OK) ? 1 : 0;
    mpu6050_shared_data.is_valid = 0;
    xSemaphoreGive(mpu6050_mutex);

    if (init_status != MPU6050_OK) {
        uart_printf_dma(&huart1, "[MPU6050] Init failed, task suspended.\r\n");
    } else {
        uart_printf_dma(&huart1, "[MPU6050] Hardware init OK, waiting for start...\r\n");
    }

    for (;;) {
        // 检查运行状态
        if (!mpu6050_running || !mpu6050_shared_data.dmp_enabled) {
            vTaskSuspend(NULL);
            uart_printf_dma(&huart1, "[MPU6050] Task resumed!\r\n");
            continue;
        }

        // 更新数据
        MPU6050_Status_t status = mpu6050_update_internal();

        if (status == MPU6050_OK) {
            update_count++;

            // Mutex 保护更新共享数据（已在 update_internal 中更新）
            xSemaphoreTake(mpu6050_mutex, portMAX_DELAY);
            // 数据已在 update_internal 中更新
            xSemaphoreGive(mpu6050_mutex);

            // 串口打印（每秒一次）
            if (update_count % 10 == 0) {
                uart_printf_dma(&huart1, "[MPU6050] P=%.1f R=%.1f Y=%.1f T=%.1f Steps=%lu [OK:%lu]\r\n",
                               mpu6050_shared_data.pitch,
                               mpu6050_shared_data.roll,
                               mpu6050_shared_data.yaw,
                               mpu6050_shared_data.temperature,
                               mpu6050_shared_data.step_count,
                               update_count);
            }

            // 通知 OLED 更新
            oled_update_mpu6050(mpu6050_shared_data.pitch,
                               mpu6050_shared_data.roll,
                               mpu6050_shared_data.yaw,
                               mpu6050_shared_data.step_count,
                               1);

            // 摔倒/碰撞报警
            if (mpu6050_shared_data.fall_flag || mpu6050_shared_data.collision_flag) {
                led_set_mode(LED_EVENT_ERROR);
            }
        } else {
            error_count++;
            if (error_count % 50 == 0) {
                uart_printf_dma(&huart1, "[MPU6050] Read error [ERR:%lu]\r\n", error_count);
            }
        }

        // 100ms 采样周期（DMP 默认 100Hz）
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ==================== 初始化函数 ====================
void mpu6050_task_init(void) {
    // 创建互斥锁
    mpu6050_mutex = xSemaphoreCreateMutex();
    if (!mpu6050_mutex) {
        uart_printf_dma(&huart1, "[ERROR] MPU6050 mutex failed!\r\n");
        for(;;);
    }

    // 初始化共享数据
    memset(&mpu6050_shared_data, 0, sizeof(MPU6050_Data_t));

    // 默认停止状态
    mpu6050_running = 0;

    // 创建任务（优先级2，栈512字节：DMP + 浮点运算需要较大栈）
    if (xTaskCreate(mpu6050_task, "mpu6050", 512, NULL, 2, &mpu6050_task_handle) != pdPASS) {
        uart_printf_dma(&huart1, "[ERROR] MPU6050 task failed!\r\n");
        for(;;);
    }

    uart_printf_dma(&huart1, "[MPU6050] Task created (suspended)!\r\n");
}

// ==================== 任务控制 ====================
void mpu6050_task_start(void) {
    if (mpu6050_task_handle && !mpu6050_running) {
        mpu6050_running = 1;
        vTaskResume(mpu6050_task_handle);
        uart_printf_dma(&huart1, "[MPU6050] Started!\r\n");
    }
}

void mpu6050_task_stop(void) {
    if (mpu6050_running) {
        mpu6050_running = 0;
        uart_printf_dma(&huart1, "[MPU6050] Stopped!\r\n");
    }
}

uint8_t mpu6050_is_running(void) {
    return mpu6050_running;
}

// ==================== 数据获取（线程安全） ====================
void mpu6050_get_attitude(float *pitch, float *roll, float *yaw, uint8_t *valid) {
    if (!mpu6050_mutex) return;

    xSemaphoreTake(mpu6050_mutex, portMAX_DELAY);
    if (pitch) *pitch = mpu6050_shared_data.pitch;
    if (roll) *roll = mpu6050_shared_data.roll;
    if (yaw) *yaw = mpu6050_shared_data.yaw;
    if (valid) *valid = mpu6050_shared_data.is_valid;
    xSemaphoreGive(mpu6050_mutex);
}

void mpu6050_get_step_data(uint32_t *steps, float *distance) {
    if (!mpu6050_mutex) return;

    xSemaphoreTake(mpu6050_mutex, portMAX_DELAY);
    if (steps) *steps = mpu6050_shared_data.step_count;
    if (distance) *distance = mpu6050_shared_data.distance;
    xSemaphoreGive(mpu6050_mutex);
}

uint8_t mpu6050_is_fall_detected(void) {
    return mpu6050_shared_data.fall_flag;
}

uint8_t mpu6050_is_collision_detected(void) {
    return mpu6050_shared_data.collision_flag;
}

float mpu6050_get_temperature(void) {
    return mpu6050_shared_data.temperature;
}

// ==================== 计步器控制 ====================
void mpu6050_reset_step_count(void) {
    xSemaphoreTake(mpu6050_mutex, portMAX_DELAY);
    dmp_set_pedometer_step_count(0);
    soft_step_count = 0;
    mpu6050_shared_data.step_count = 0;
    mpu6050_shared_data.distance = 0.0f;
    xSemaphoreGive(mpu6050_mutex);
}

void mpu6050_set_step_length(float length) {
    step_length = length;
}

// ==================== 私有函数实现 ====================
static MPU6050_Status_t mpu6050_read_accel_raw(short *ax, short *ay, short *az) {
    uint8_t buf[6];

    if (mpu6050_i2c_read(MPU6050_ACCEL_XOUT_H, buf, 6) == HAL_OK) {
        *ax = (int16_t)((buf[0] << 8) | buf[1]);
        *ay = (int16_t)((buf[2] << 8) | buf[3]);
        *az = (int16_t)((buf[4] << 8) | buf[5]);

        // 更新共享数据中的原始值
        mpu6050_shared_data.accel_x_g = (float)(*ax) / 16384.0f;
        mpu6050_shared_data.accel_y_g = (float)(*ay) / 16384.0f;
        mpu6050_shared_data.accel_z_g = (float)(*az) / 16384.0f;

        return MPU6050_OK;
    }
    return MPU6050_ERROR;
}

static MPU6050_Status_t mpu6050_read_temp_raw(short *temp) {
    uint8_t buf[2];

    if (mpu6050_i2c_read(MPU6050_TEMP_OUT_H, buf, 2) == HAL_OK) {
        *temp = (int16_t)((buf[0] << 8) | buf[1]);
        return MPU6050_OK;
    }
    return MPU6050_ERROR;
}

static void mpu6050_calculate_physical_values(void) {
    // 加速度已在 read_accel_raw 中计算
    // 这里可以添加陀螺仪转换（如果需要）
}

static void mpu6050_update_fall_detection(void) {
    // 计算 SVM（加速度矢量和）
    float ax = mpu6050_shared_data.accel_x_g * 16384.0f;
    float ay = mpu6050_shared_data.accel_y_g * 16384.0f;
    float az = mpu6050_shared_data.accel_z_g * 16384.0f;
    mpu6050_shared_data.svm = (int)sqrtf(ax*ax + ay*ay + az*az);

    // 角度检测
    if (fabsf(mpu6050_shared_data.pitch) > MPU6050_FALL_ANGLE_THRESHOLD ||
        fabsf(mpu6050_shared_data.roll) > MPU6050_FALL_ANGLE_THRESHOLD) {
        mpu6050_shared_data.fall_flag = 1;
    } else {
        mpu6050_shared_data.fall_flag = 0;
    }

    // 碰撞检测
    if (mpu6050_shared_data.svm > MPU6050_COLLISION_SVM_HIGH ||
        mpu6050_shared_data.svm < MPU6050_COLLISION_SVM_LOW) {
        collision_counter = 0;
    }

    collision_counter++;

    if (collision_counter <= 10) {
        mpu6050_shared_data.collision_flag = 1;
    } else {
        if (collision_counter > 10) collision_counter = 10;
        mpu6050_shared_data.collision_flag = 0;
    }
}

static void mpu6050_update_step_counter(void) {
    // 从 DMP 读取步数
    unsigned long dmp_steps = 0;
    dmp_get_pedometer_step_count(&dmp_steps);

    // 软件辅助计步
    float magnitude = sqrtf(
        mpu6050_shared_data.accel_x_g * mpu6050_shared_data.accel_x_g +
        mpu6050_shared_data.accel_y_g * mpu6050_shared_data.accel_y_g +
        mpu6050_shared_data.accel_z_g * mpu6050_shared_data.accel_z_g
    );

    float accel_change = fabsf(magnitude - last_accel_magnitude);
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (accel_change > MPU6050_SOFT_STEP_THRESHOLD &&
        (current_time - last_step_time) > MPU6050_STEP_MIN_INTERVAL) {
        soft_step_count++;
        last_step_time = current_time;
    }

    last_accel_magnitude = magnitude;

    // 使用较大值
    mpu6050_shared_data.step_count = (soft_step_count > dmp_steps) ? soft_step_count : dmp_steps;
    mpu6050_shared_data.distance = mpu6050_shared_data.step_count * step_length;
}
