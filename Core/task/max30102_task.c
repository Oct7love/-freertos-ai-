//
// Created by Claude on 2026/1/4.
// MAX30102 心率血氧传感器 FreeRTOS 任务实现
//

#include "max30102_task.h"
#include "oled_task.h"
#include "esp32_task.h"
#include "i2c.h"
#include <math.h>
#include <string.h>


// ==================== 私有变量 ====================
static TaskHandle_t max30102_task_handle = NULL;
static SemaphoreHandle_t max30102_mutex = NULL;
static volatile uint8_t max30102_running = 0;

// 共享数据（Mutex保护）
static MAX30102_Data_t max30102_shared_data = {0};

// 采样缓冲区（静态分配，避免栈溢出）
static uint32_t ir_buffer[MAX30102_BUFFER_SIZE];
static uint32_t red_buffer[MAX30102_BUFFER_SIZE];
static uint8_t buffer_index = 0;
static uint8_t buffer_filled = 0;

// SpO2查表（Maxim官方算法）
// 公式近似：-45.060*ratio^2 + 30.354*ratio + 94.845
static const uint8_t uch_spo2_table[184] = {
    95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
    99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
    97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
    90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
    80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
    66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
    49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
    28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
    3, 2, 1
};

// 心率检测变量
static uint32_t last_beat_time = 0;
static uint32_t beat_intervals[8] = {0};
static uint8_t beat_index = 0;
static float smoothed_hr = 0;
static float smoothed_spo2 = 0;

// 滤波变量
static float ir_dc = 0.0f;
static float red_dc = 0.0f;
static float ir_ac_filtered = 0.0f;
static float red_ac_filtered = 0.0f;
static uint8_t peak_count = 0;

// 手指检测消抖变量
static uint8_t no_finger_cnt = 0;
#define NO_FINGER_DEBOUNCE 50  // 连续50个样本（500ms）才判定无手指

// 峰值检测上下文（支持hysteresis）
typedef struct {
    float prev;
    float prev2;
    float env;           // 包络跟踪
    uint8_t armed;       // 是否允许触发
    uint32_t last_peak_ms;
} PeakCtx_t;
static PeakCtx_t pk = {0};

// 重置所有算法状态（手指移开时调用）
static void max30102_reset_algorithm(void) {
    last_beat_time = 0;
    memset(beat_intervals, 0, sizeof(beat_intervals));
    beat_index = 0;
    peak_count = 0;
    smoothed_hr = 0;
    smoothed_spo2 = 0;
    ir_dc = 0;
    red_dc = 0;
    ir_ac_filtered = 0;
    red_ac_filtered = 0;
    no_finger_cnt = 0;  // 重置手指检测消抖计数器
    memset(&pk, 0, sizeof(pk));
    pk.armed = 1;  // 关键：允许第一次触发
}

// ==================== I2C 通信函数 ====================
static HAL_StatusTypeDef max30102_write_reg(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(&hi2c2, MAX30102_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &value, 1,
                             MAX30102_I2C_TIMEOUT);
}

static HAL_StatusTypeDef max30102_read_reg(uint8_t reg, uint8_t *value) {
    return HAL_I2C_Mem_Read(&hi2c2, MAX30102_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, value, 1,
                            MAX30102_I2C_TIMEOUT);
}

static HAL_StatusTypeDef max30102_read_regs(uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(&hi2c2, MAX30102_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, data, len,
                            MAX30102_I2C_TIMEOUT);
}

// ==================== 传感器初始化 ====================
static MAX30102_Status_t max30102_hw_init(void) {
    uint8_t part_id = 0;

    // 等待传感器上电稳定
    vTaskDelay(pdMS_TO_TICKS(100));

    // 检测设备ID
    if (max30102_read_reg(MAX30102_REG_PART_ID, &part_id) != HAL_OK) {
        return MAX30102_NOT_FOUND;
    }
    if (part_id != 0x15) {
        uart_printf_dma(&huart1, "[MAX30102] Wrong ID: 0x%02X\r\n", part_id);
        return MAX30102_NOT_FOUND;
    }

    // 软复位传感器
    if (max30102_write_reg(MAX30102_REG_MODE_CONFIG, 0x40) != HAL_OK) {
        return MAX30102_ERROR;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // 配置中断（A_FULL使能）
    max30102_write_reg(MAX30102_REG_INTR_ENABLE_1, 0x80);
    max30102_write_reg(MAX30102_REG_INTR_ENABLE_2, 0x00);

    // FIFO配置：sample avg=1(无平均), rollover=1, A_FULL=15
    max30102_write_reg(MAX30102_REG_FIFO_CONFIG, 0x1F);

    // 清空FIFO指针
    max30102_write_reg(MAX30102_REG_FIFO_WR_PTR, 0x00);
    max30102_write_reg(MAX30102_REG_OVF_COUNTER, 0x00);
    max30102_write_reg(MAX30102_REG_FIFO_RD_PTR, 0x00);

    // 进入SpO2模式（红光+红外）
    max30102_write_reg(MAX30102_REG_MODE_CONFIG, 0x03);

    // SpO2配置（ADC=4096, 采样率=100Hz, LED脉宽=411us）
    max30102_write_reg(MAX30102_REG_SPO2_CONFIG, 0x27);

    // LED电流配置（红光和红外都设为0x24 ≈ 7.2mA，避免ADC饱和）
    max30102_write_reg(MAX30102_REG_LED1_PA, 0x24);
    max30102_write_reg(MAX30102_REG_LED2_PA, 0x24);

    // 清除中断状态
    uint8_t tmp;
    max30102_read_reg(MAX30102_REG_INTR_STATUS_1, &tmp);
    max30102_read_reg(MAX30102_REG_INTR_STATUS_2, &tmp);

    uart_printf_dma(&huart1, "[MAX30102] HW Init OK, ID=0x%02X\r\n", part_id);
    return MAX30102_OK;
}

// ==================== 数据读取 ====================
static MAX30102_Status_t max30102_read_fifo(uint32_t *red, uint32_t *ir) {
    uint8_t data[6];

    if (max30102_read_regs(MAX30102_REG_FIFO_DATA, data, 6) != HAL_OK) {
        return MAX30102_ERROR;
    }

    // 组合24位数据（高18位有效）
    *red = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    *red &= 0x3FFFF;  // 18位掩码

    *ir = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];
    *ir &= 0x3FFFF;

    return MAX30102_OK;
}

// ==================== 信号处理算法 ====================

// DC滤波（低通滤波）
static float dc_filter(float x, float *dc, float alpha) {
    *dc = *dc + alpha * (x - *dc);
    return x - *dc;
}

// 均值滤波
static uint32_t mean_filter(uint32_t *buffer, uint8_t len) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += buffer[i];
    }
    return sum / len;
}

// 峰值检测（改进版：使用斜率过零检测）
static uint8_t detect_peak_hys(float ac, uint32_t t_ms) {
    // 1) 包络跟踪（更快响应）
    float a = fabsf(ac);
    if (pk.env < 100) {
        pk.env = a;
        pk.armed = 1;
    } else {
        // 快升慢降的包络跟踪
        if (a > pk.env) {
            pk.env = 0.9f * pk.env + 0.1f * a;  // 快速上升
        } else {
            pk.env = 0.995f * pk.env + 0.005f * a;  // 缓慢下降
        }
    }

    // 动态阈值（更宽松）
    float thr_hi = pk.env * 0.35f;  // 降低触发阈值
    float thr_lo = pk.env * 0.10f;  // 降低重武装阈值
    if (thr_hi < 150.0f) thr_hi = 150.0f;  // 降低最小值
    if (thr_lo < 50.0f) thr_lo = 50.0f;

    uint8_t peak = 0;

    // 2) 重新武装条件：AC值过零或低于低阈值
    // 关键修复：AC过零时也重新武装（PPG信号特性）
    if (a < thr_lo || (pk.prev > 0 && ac < 0) || (pk.prev < 0 && ac > 0)) {
        pk.armed = 1;
    }

    // 3) 三点极大检测（使用原始AC值，检测正峰）
    if (pk.armed) {
        // 检测正向峰值（AC从上升转为下降）
        if (pk.prev > pk.prev2 && pk.prev > ac && pk.prev > thr_hi) {
            if (t_ms - pk.last_peak_ms > 450) {  // 最小间隔450ms（最大133BPM）
                peak = 1;
                pk.last_peak_ms = t_ms;
                pk.armed = 0;
            }
        }
    }

    pk.prev2 = pk.prev;
    pk.prev = ac;
    return peak;
}

// 计算心率
static uint16_t calculate_heart_rate(uint32_t interval_ms) {
    if (interval_ms < 300 || interval_ms > 1500) {
        return 0;  // 无效间隔
    }
    return (uint16_t)(60000 / interval_ms);
}

// 计算血氧饱和度（使用Maxim官方查表法）
static uint8_t calculate_spo2(float red_ac, float red_dc_val, float ir_ac, float ir_dc_val) {
    if (ir_dc_val < 1000 || red_dc_val < 1000 || ir_ac < 10 || red_ac < 10) {
        return 0;
    }

    // R = (AC_red/DC_red) / (AC_ir/DC_ir) * 100
    float r_red = red_ac / red_dc_val;
    float r_ir = ir_ac / (ir_dc_val + 0.0001f);
    float ratio = r_red / (r_ir + 0.0001f);

    // 转换为查表索引（ratio * 100）
    int32_t ratio_index = (int32_t)(ratio * 100.0f);

    // 使用查表法（Maxim官方算法）
    if (ratio_index > 2 && ratio_index < 184) {
        return uch_spo2_table[ratio_index];
    }

    // 超出查表范围，使用线性插值
    if (ratio_index <= 2) {
        return 100;  // ratio很小说明血氧很高
    }
    if (ratio_index >= 184) {
        return 70;   // ratio很大说明血氧很低
    }

    return 0;
}

// ==================== 数据处理主函数（重构版） ====================

// 获取FIFO中的样本数
static uint8_t max30102_get_fifo_count(void) {
    uint8_t wr = 0, rd = 0;
    if (max30102_read_reg(MAX30102_REG_FIFO_WR_PTR, &wr) != HAL_OK) return 0;
    if (max30102_read_reg(MAX30102_REG_FIFO_RD_PTR, &rd) != HAL_OK) return 0;
    return (uint8_t)((wr - rd) & 0x1F);  // FIFO深度32
}

// 处理单个样本
static void process_one_sample(uint32_t red_raw, uint32_t ir_raw, uint32_t t_ms) {
    // 更新原始数据
    xSemaphoreTake(max30102_mutex, portMAX_DELAY);
    max30102_shared_data.red_raw = red_raw;
    max30102_shared_data.ir_raw = ir_raw;

    // 手指检测（带消抖）
    if (ir_raw < MAX30102_FINGER_THRESHOLD) {
        no_finger_cnt++;
        if (no_finger_cnt >= NO_FINGER_DEBOUNCE) {
            // 连续50个样本低于阈值，确认无手指
            if (max30102_shared_data.finger_detected) {
                max30102_reset_algorithm();
            }
            max30102_shared_data.finger_detected = 0;
            max30102_shared_data.hr_status = HR_STATUS_NO_FINGER;
            max30102_shared_data.is_valid = 0;
            xSemaphoreGive(max30102_mutex);
            return;
        }
        // 消抖期间：跳过此样本，避免异常数据污染算法
        xSemaphoreGive(max30102_mutex);
        return;
    } else {
        no_finger_cnt = 0;  // 有手指，重置计数器
        max30102_shared_data.finger_detected = 1;
    }
    xSemaphoreGive(max30102_mutex);

    // 超慢DC跟踪
    if (ir_dc < 1000) {
        ir_dc = (float)ir_raw;
        red_dc = (float)red_raw;
    } else {
        ir_dc = ir_dc * 0.998f + (float)ir_raw * 0.002f;
        red_dc = red_dc * 0.998f + (float)red_raw * 0.002f;
    }

    // 计算AC
    float ir_ac = (float)ir_raw - ir_dc;
    float red_ac = (float)red_raw - red_dc;

    // AC低通滤波
    ir_ac_filtered = ir_ac_filtered * 0.7f + ir_ac * 0.3f;
    red_ac_filtered = red_ac_filtered * 0.7f + red_ac * 0.3f;

    // 峰值检测（改进版）
    if (detect_peak_hys(ir_ac_filtered, t_ms)) {
        uint32_t interval = t_ms - last_beat_time;

        // 间隔范围：450-1200ms（对应 50-133 BPM）
        if (interval > 450 && interval < 1200) {
            beat_intervals[beat_index] = interval;
            beat_index = (beat_index + 1) % 8;
            peak_count++;

            // 4个峰值就开始计算（更快响应）
            if (peak_count >= 4) {
                uint32_t avg_interval = 0;
                uint8_t valid_count = 0;
                for (int i = 0; i < 8; i++) {
                    if (beat_intervals[i] > 450 && beat_intervals[i] < 1200) {
                        avg_interval += beat_intervals[i];
                        valid_count++;
                    }
                }
                // 至少3个有效间隔才计算
                if (valid_count >= 3) {
                    avg_interval /= valid_count;
                    uint16_t hr_raw = calculate_heart_rate(avg_interval);
                    uint8_t spo2_raw = calculate_spo2(fabsf(red_ac_filtered), red_dc,
                                                       fabsf(ir_ac_filtered), ir_dc);

                    // IIR平滑
                    if (smoothed_hr < 30) {
                        smoothed_hr = (float)hr_raw;
                    } else {
                        smoothed_hr = 0.7f * smoothed_hr + 0.3f * (float)hr_raw;
                    }
                    if (smoothed_spo2 < 50) {
                        smoothed_spo2 = (float)spo2_raw;
                    } else {
                        smoothed_spo2 = 0.8f * smoothed_spo2 + 0.2f * (float)spo2_raw;
                    }

                    uint16_t hr = (uint16_t)(smoothed_hr + 0.5f);
                    uint8_t spo2 = (uint8_t)(smoothed_spo2 + 0.5f);

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
                    max30102_shared_data.is_valid = 1;
                    max30102_shared_data.signal_quality = (ir_dc > 80000) ? 75 : 50;
                    xSemaphoreGive(max30102_mutex);
                }
            } else {
                xSemaphoreTake(max30102_mutex, portMAX_DELAY);
                max30102_shared_data.hr_status = HR_STATUS_CALCULATING;
                xSemaphoreGive(max30102_mutex);
            }
        }
        last_beat_time = t_ms;
    }
}

// 批量处理FIFO数据
static void max30102_process_data(void) {
    uint8_t n = max30102_get_fifo_count();

    // ★关键：n==0时打破锁相
    if (n == 0) {
        vTaskDelay(pdMS_TO_TICKS(1));  // 等1ms打破10ms锁相
        n = max30102_get_fifo_count();
    }
    if (n == 0) {
        n = 1;  // 兜底：至少读一个
    }
    if (n > 16) n = 16;

    uint8_t buf[6 * 16];
    if (max30102_read_regs(MAX30102_REG_FIFO_DATA, buf, 6 * n) != HAL_OK) return;

    // 时间戳单调递增
    static uint32_t t_ms = 0;
    uint32_t base_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (t_ms == 0) t_ms = base_ms;

    for (uint8_t i = 0; i < n; i++) {
        uint8_t *p = &buf[6 * i];
        uint32_t red_raw = (((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2]) & 0x3FFFF;
        uint32_t ir_raw  = (((uint32_t)p[3] << 16) | ((uint32_t)p[4] << 8) | p[5]) & 0x3FFFF;

        t_ms += 10;  // 每样本+10ms（100Hz）
        process_one_sample(red_raw, ir_raw, t_ms);
    }
}

// ==================== 任务主函数 ====================
static void max30102_task(void *arg) {
    (void)arg;
    uint32_t update_count = 0;
    TickType_t last_wake = xTaskGetTickCount();

    // 错峰启动
    vTaskDelay(pdMS_TO_TICKS(450));

    // 硬件初始化
    MAX30102_Status_t init_status = max30102_hw_init();

    if (init_status != MAX30102_OK) {
        uart_printf_dma(&huart1, "[MAX30102] Init Failed! Code=%d\r\n", init_status);
        xSemaphoreTake(max30102_mutex, portMAX_DELAY);
        max30102_shared_data.is_valid = 0;
        xSemaphoreGive(max30102_mutex);
    } else {
        uart_printf_dma(&huart1, "[MAX30102] Init OK, Task Ready\r\n");
    }

    last_wake = xTaskGetTickCount();

    for (;;) {
        // 检查运行状态
        if (!max30102_running || init_status != MAX30102_OK) {
            vTaskSuspend(NULL);
            last_wake = xTaskGetTickCount();  // 恢复后重新同步
            continue;
        }

        // 处理数据
        max30102_process_data();
        update_count++;

        // 定期日志（每100次=1秒打印一次）
        if (update_count % 100 == 0) {
            uint8_t fifo_n = max30102_get_fifo_count();

            xSemaphoreTake(max30102_mutex, portMAX_DELAY);
            uint8_t finger = max30102_shared_data.finger_detected;
            uint16_t hr = max30102_shared_data.heart_rate;
            uint8_t spo2 = max30102_shared_data.spo2;
            uint32_t ir = max30102_shared_data.ir_raw;
            xSemaphoreGive(max30102_mutex);

            // Debug输出：定位问题
            uart_printf_dma(&huart1,
                "[MAX] n=%d ir=%lu ac=%.0f env=%.0f arm=%d pk=%d hr=%d\r\n",
                fifo_n, ir, ir_ac_filtered, pk.env, pk.armed, peak_count, hr);
        }

        // OLED更新（每10次=100ms更新一次，且在mutex外）
        if (update_count % 10 == 0) {
            uint16_t hr_copy;
            uint8_t spo2_copy, finger_copy, valid_copy;

            xSemaphoreTake(max30102_mutex, portMAX_DELAY);
            hr_copy = max30102_shared_data.heart_rate;
            spo2_copy = max30102_shared_data.spo2;
            finger_copy = max30102_shared_data.finger_detected;
            valid_copy = max30102_shared_data.is_valid;
            xSemaphoreGive(max30102_mutex);

            oled_update_max30102(hr_copy, spo2_copy, finger_copy, valid_copy);

            // 发送数据给ESP32（只在有效时发送）
            if (valid_copy && finger_copy) {
                esp32_send_heartrate((uint8_t)hr_copy, spo2_copy);
            }
        }

        // 使用vTaskDelayUntil保证稳定10ms周期
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

// ==================== 公共接口实现 ====================
void max30102_task_init(void) {
    // 创建互斥锁
    max30102_mutex = xSemaphoreCreateMutex();
    if (!max30102_mutex) {
        uart_printf_dma(&huart1, "[ERROR] MAX30102 mutex failed!\r\n");
        return;
    }

    // 初始化共享数据
    memset(&max30102_shared_data, 0, sizeof(MAX30102_Data_t));
    max30102_shared_data.hr_status = HR_STATUS_INVALID;

    // 创建任务（优先级2，栈1024字节）
    if (xTaskCreate(max30102_task, "max30102", 1024, NULL, 2, &max30102_task_handle) != pdPASS) {
        uart_printf_dma(&huart1, "[ERROR] MAX30102 task creation failed!\r\n");
        return;
    }

    uart_printf_dma(&huart1, "[MAX30102] Task Init OK!\r\n");
}

void max30102_task_start(void) {
    if (max30102_task_handle) {
        max30102_running = 1;
        vTaskResume(max30102_task_handle);
        uart_printf_dma(&huart1, "[MAX30102] Task Started\r\n");
    }
}

void max30102_task_stop(void) {
    max30102_running = 0;
    uart_printf_dma(&huart1, "[MAX30102] Task Stopped\r\n");
}

uint8_t max30102_is_running(void) {
    return max30102_running;
}

void max30102_get_data(uint16_t *heart_rate, uint8_t *spo2, uint8_t *valid) {
    if (!max30102_mutex) return;

    xSemaphoreTake(max30102_mutex, portMAX_DELAY);
    if (heart_rate) *heart_rate = max30102_shared_data.heart_rate;
    if (spo2) *spo2 = max30102_shared_data.spo2;
    if (valid) *valid = max30102_shared_data.is_valid && max30102_shared_data.finger_detected;
    xSemaphoreGive(max30102_mutex);
}

void max30102_get_full_data(MAX30102_Data_t *data) {
    if (!max30102_mutex || !data) return;

    xSemaphoreTake(max30102_mutex, portMAX_DELAY);
    memcpy(data, &max30102_shared_data, sizeof(MAX30102_Data_t));
    xSemaphoreGive(max30102_mutex);
}

uint8_t max30102_is_finger_detected(void) {
    uint8_t result = 0;
    if (max30102_mutex) {
        xSemaphoreTake(max30102_mutex, portMAX_DELAY);
        result = max30102_shared_data.finger_detected;
        xSemaphoreGive(max30102_mutex);
    }
    return result;
}

uint8_t max30102_get_signal_quality(void) {
    uint8_t result = 0;
    if (max30102_mutex) {
        xSemaphoreTake(max30102_mutex, portMAX_DELAY);
        result = max30102_shared_data.signal_quality;
        xSemaphoreGive(max30102_mutex);
    }
    return result;
}

MAX30102_HR_Status_t max30102_get_hr_status(void) {
    MAX30102_HR_Status_t result = HR_STATUS_INVALID;
    if (max30102_mutex) {
        xSemaphoreTake(max30102_mutex, portMAX_DELAY);
        result = max30102_shared_data.hr_status;
        xSemaphoreGive(max30102_mutex);
    }
    return result;
}

MAX30102_Status_t max30102_reset(void) {
    return max30102_hw_init();
}

float max30102_read_temperature(void) {
    uint8_t temp_int, temp_frac;

    // 触发温度测量
    max30102_write_reg(MAX30102_REG_TEMP_CONFIG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(30));

    if (max30102_read_reg(MAX30102_REG_TEMP_INT, &temp_int) != HAL_OK) {
        return -999.0f;
    }
    if (max30102_read_reg(MAX30102_REG_TEMP_FRAC, &temp_frac) != HAL_OK) {
        return -999.0f;
    }

    float temperature = (float)temp_int + (float)temp_frac * 0.0625f;

    xSemaphoreTake(max30102_mutex, portMAX_DELAY);
    max30102_shared_data.temperature = temperature;
    xSemaphoreGive(max30102_mutex);

    return temperature;
}
