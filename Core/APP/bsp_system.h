//
// Created by 13615 on 2025/12/28.
//

#ifndef FREERTOS_TEST_BSP_SYSTEM_H
#define FREERTOS_TEST_BSP_SYSTEM_H

/* FreeRTOS 核心 */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "freertos_os2.h"
#include "freertos_mpool.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

/* HAL库头文件 */
#include "main.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "stdint.h"
#include "math.h"
#include "gpio.h"
#include "usart.h"
#include "dma.h"
#include "timers.h"
#include "tim.h"
#include "adc.h"



//driver头文件
#include "task/driver/oled.h"
#include "task/driver/oledfont.h"
#include "task/driver/delay.h"
#include "task/driver/dht11.h"
#include "task/driver/mq2.h"


//任务函数
#include "scheduler_task.h"
#include "task/uart_task.h"
#include "task/led_task.h"
#include "task/oled_task.h"
#include "task/key_task.h"
#include "task/dht11_task.h"
#include "task/mq2_task.h"
#include "ringbuffer/ringbuffer.h"
#endif //FREERTOS_TEST_BSP_SYSTEM_H