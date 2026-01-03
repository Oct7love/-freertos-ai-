//
// Created by 13615 on 2025/12/28.
//

#include "scheduler_task.h"
//消息队列句柄
// static QueueHandle_t data_queue;

//消息结构体
typedef struct {
    uint8_t sensor_id;
    uint32_t sensor_value;
}sentor_msg_t;

static SemaphoreHandle_t g_printf_mutex = NULL;//打印互斥锁

void safe_printf(const char *format, ...) {
   va_list ap;
    va_start(ap, format);
    if (g_printf_mutex) xSemaphoreTake(g_printf_mutex,portMAX_DELAY);
    vprintf(format,ap);
    if (g_printf_mutex) xSemaphoreGive(g_printf_mutex);
        va_end(ap);

}
// //生产者任务
// static void producer_task(void *arg) {
//     sentor_msg_t msg;
//     uint32_t counter = 0;
//     for (;;) {
//         //准备数据
//         msg.sensor_id = 1;
//         msg.sensor_value = counter++;
//         //发送数据到消息队列
//         if(xQueueSend(data_queue,&msg,pdMS_TO_TICKS(100)) == pdTRUE)//超时100ms
//         {
//             safe_printf("[生产者]发送:ID = %u,Value = %lu\r\n",msg.sensor_id,msg.sensor_value);
//         } else {
//             safe_printf("生产者队列满，发送失败\r\n");
//         }
//         vTaskDelay(pdMS_TO_TICKS(1000));//让出cpu
//     }
// }
//
// //消费者任务
// static void consumer_task(void *arg) {
//     sentor_msg_t msg;
//     for (;;) {
//             if (xQueueReceive(data_queue,&msg,portMAX_DELAY) == pdTRUE) {
//             safe_printf("[消费者]接收:ID = %u,Value = %lu\r\n",msg.sensor_id,msg.sensor_value);
//         }
//     }
// }



void scheduler_init(void) {
    /*创建锁*/
    g_printf_mutex = xSemaphoreCreateMutex();


    /*创建队列*/
    // data_queue = xQueueCreate(10,sizeof(sentor_msg_t));


    // /*创建任务线程*/
    // xTaskCreate(producer_task,"producer",256,NULL,2,NULL);//生产消费线程测试
    // xTaskCreate(consumer_task,"consumer",256,NULL,2,NULL);
    uart_task_init();//创建串口任务
    // led_task_init();//创建LED任务
    oled_task_init();
    key_task_init();
    dht11_task_init();   // 温湿度传感器任务
    mq2_task_init();     // MQ2气体传感器任务
}