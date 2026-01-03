//
// Created by 13615 on 2026/1/3.
// MQ2 气体传感器底层驱动实现
//

#include "mq2.h"

// DMA循环采样缓冲区
static uint32_t mq2_adc_buffer[MQ2_SAMPLE_COUNT];

// 初始化MQ2（启动ADC DMA循环采样）
void MQ2_Init(void) {
    // 启动ADC DMA循环采样，后台自动更新 mq2_adc_buffer
    HAL_ADC_Start_DMA(&MQ2_ADC_HANDLE, mq2_adc_buffer, MQ2_SAMPLE_COUNT);
}

// 读取ADC均值（从DMA缓冲区计算）
static uint32_t MQ2_ReadADC_Average(void) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < MQ2_SAMPLE_COUNT; i++) {
        sum += mq2_adc_buffer[i];
    }
    return (sum / MQ2_SAMPLE_COUNT);
}

// 更新传感器数据
void MQ2_Update(MQ2_Data_t *data) {
    if (!data) return;

    // 1. 读取ADC均值
    data->adc_raw = MQ2_ReadADC_Average();

    // 2. ADC → 电压
    data->voltage = MQ2_ADC_TO_VOLTAGE(data->adc_raw);

    // 3. 电压有效性检查
    if (!MQ2_IS_VOLTAGE_VALID(data->voltage)) {
        data->ppm = 0.0f;
        data->alarm = MQ2_ALARM_SAFE;
        return;
    }

    // 4. 电压 → 传感器电阻Rs
    data->rs = MQ2_VOLTAGE_TO_RS(data->voltage);

    // 5. Rs → Rs/R0比值
    data->ratio = MQ2_RS_TO_RATIO(data->rs);

    // 6. 比值 → PPM
    data->ppm = MQ2_RATIO_TO_PPM(data->ratio);

    // 7. PPM限幅
    if (!MQ2_IS_PPM_VALID(data->ppm)) {
        data->ppm = (data->ppm < 0.0f) ? 0.0f : 9999.0f;
    }

    // 8. 判断报警等级
    data->alarm = MQ2_CheckAlarm(data->ppm);
}
