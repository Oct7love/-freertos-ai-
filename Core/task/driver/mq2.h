//
// Created by 13615 on 2026/1/3.
// MQ2 气体传感器底层驱动
//

#ifndef FREERTOS_TEST_MQ2_H
#define FREERTOS_TEST_MQ2_H

#include "main.h"
#include "adc.h"
#include <stdint.h>
#include <math.h>



// ==================== 硬件配置 ====================
#define MQ2_ADC_HANDLE          hadc1           // ADC句柄（需在CubeMX配置）
#define MQ2_SAMPLE_COUNT        30              // 采样次数（均值滤波）

// ==================== 电路参数 ====================
#define MQ2_VCC                 5.0f            // 传感器供电电压 (V)
#define MQ2_VREF                3.3f            // ADC参考电压 (V)
#define MQ2_ADC_RESOLUTION      4095.0f         // ADC分辨率 (12位)
#define MQ2_RL_VALUE            4700.0f         // 负载电阻RL (Ω)
#define MQ2_R0_VALUE            3000.0f         // 清洁空气中传感器电阻R0 (Ω)

// ==================== 气体检测参数 ====================
// LPG (液化石油气)
#define MQ2_GAS_A               270.0f          // 曲线系数A
#define MQ2_GAS_B               (-0.84f)        // 曲线系数B

// ==================== 报警阈值 ====================
#define MQ2_ALARM_LEVEL_SAFE    100.0f          // 安全 (ppm)
#define MQ2_ALARM_LEVEL_LOW     300.0f          // 低级报警 (ppm)
#define MQ2_ALARM_LEVEL_MID     500.0f          // 中级报警 (ppm)
#define MQ2_ALARM_LEVEL_HIGH    800.0f          // 高级报警 (ppm)

// ==================== 计算宏 ====================
// ADC原始值 → 电压 (V = ADC * Vref / 分辨率)
#define MQ2_ADC_TO_VOLTAGE(adc)     ((float)(adc) * (MQ2_VREF / MQ2_ADC_RESOLUTION))

// 电压 → 传感器电阻Rs (Rs = RL * (Vcc - V) / V)
#define MQ2_VOLTAGE_TO_RS(v)        ((MQ2_RL_VALUE * (MQ2_VCC - (v))) / (v))

// Rs → Rs/R0比值
#define MQ2_RS_TO_RATIO(rs)         ((float)(rs) / MQ2_R0_VALUE)

// Rs/R0比值 → PPM (PPM = (ratio/A)^(1/B))
#define MQ2_RATIO_TO_PPM(ratio)     (powf((ratio) / MQ2_GAS_A, 1.0f / MQ2_GAS_B))

// ==================== 数据验证宏 ====================
#define MQ2_IS_VOLTAGE_VALID(v)     ((v) > 0.01f && (v) < (MQ2_VCC - 0.01f))
#define MQ2_IS_PPM_VALID(ppm)       ((ppm) >= 0.0f && (ppm) < 10000.0f)

// ==================== 类型定义 ====================
typedef enum {
    MQ2_ALARM_SAFE = 0,     // 安全
    MQ2_ALARM_LOW,          // 低级报警
    MQ2_ALARM_MID,          // 中级报警
    MQ2_ALARM_HIGH,         // 高级报警
    MQ2_ALARM_DANGER        // 危险
} MQ2_AlarmLevel_t;

typedef struct {
    uint32_t adc_raw;           // ADC原始值
    float voltage;              // 电压 (V)
    float rs;                   // 传感器电阻 (Ω)
    float ratio;                // Rs/R0比值
    float ppm;                  // 气体浓度 (ppm)
    MQ2_AlarmLevel_t alarm;     // 报警等级
} MQ2_Data_t;

// ==================== 函数声明 ====================
// 初始化MQ2（启动ADC DMA）
void MQ2_Init(void);

// 更新传感器数据（读取ADC，计算PPM）
void MQ2_Update(MQ2_Data_t *data);

// 获取报警等级
static inline MQ2_AlarmLevel_t MQ2_CheckAlarm(float ppm) {
    if (ppm < MQ2_ALARM_LEVEL_SAFE)  return MQ2_ALARM_SAFE;
    if (ppm < MQ2_ALARM_LEVEL_LOW)   return MQ2_ALARM_LOW;
    if (ppm < MQ2_ALARM_LEVEL_MID)   return MQ2_ALARM_MID;
    if (ppm < MQ2_ALARM_LEVEL_HIGH)  return MQ2_ALARM_HIGH;
    return MQ2_ALARM_DANGER;
}

#endif //FREERTOS_TEST_MQ2_H
