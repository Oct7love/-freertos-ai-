#ifndef __OLED_ICONS_H
#define __OLED_ICONS_H

#include <stdint.h>

// 图标数据声明 (8x8点阵)
extern const uint8_t icon_home[8];    // 房屋图标 (欢迎界面)
extern const uint8_t icon_temp[8];    // 温度计图标 (DHT11)
extern const uint8_t icon_gas[8];     // 气体分子图标 (MQ2)
extern const uint8_t icon_gyro[8];    // 陀螺仪图标 (MPU6050)
extern const uint8_t icon_heart[8];   // 心跳图标 (MAX30102)
extern const uint8_t icon_gps[8];     // GPS卫星图标 (GPS)

// 图标绘制函数
void OLED_DrawIcon8x8(uint8_t x, uint8_t y, const uint8_t* icon);

#endif
