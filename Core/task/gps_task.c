//
// Created by Claude on 2026/1/5.
// GPS 模块 FreeRTOS 任务实现
//

#include "gps_task.h"
#include "oled_task.h"
#include "esp32_task.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

// ==================== 外部声明 ====================
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern uint8_t gps_dma_buffer[256];

// ==================== 私有变量 ====================
static TaskHandle_t gps_task_handle = NULL;
static SemaphoreHandle_t gps_mutex = NULL;
static volatile uint8_t gps_running = 0;

// 共享数据（Mutex 保护）
static GPS_Data_t gps_shared_data = {0};
static GPS_Status_t gps_status = GPS_STATUS_INIT;

// 接收缓冲区（环形缓冲区）
static uint8_t gps_rx_buffer[GPS_RX_BUFFER_SIZE];
static volatile uint16_t rx_write_pos = 0;
static volatile uint16_t rx_read_pos = 0;

// 解析缓冲区
static char parse_buffer[GPS_RX_BUFFER_SIZE];

// ==================== 环形缓冲区操作 ====================
static uint16_t ringbuf_available(void) {
    return (rx_write_pos - rx_read_pos + GPS_RX_BUFFER_SIZE) % GPS_RX_BUFFER_SIZE;
}

static uint16_t ringbuf_read(uint8_t *dst, uint16_t max_len) {
    uint16_t avail = ringbuf_available();
    if (avail == 0) return 0;
    if (avail > max_len) avail = max_len;

    for (uint16_t i = 0; i < avail; i++) {
        dst[i] = gps_rx_buffer[rx_read_pos];
        rx_read_pos = (rx_read_pos + 1) % GPS_RX_BUFFER_SIZE;
    }
    return avail;
}

// ==================== NMEA 解析函数 ====================

// 解析 $GNGGA 或 $GPGGA 语句
// 格式: $GNGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
static uint8_t parse_gga(const char *sentence, GPS_Data_t *data) {
    char *fields[15] = {0};
    char buf[128];
    int field_count = 0;

    // 复制语句（strtok 会修改原字符串）
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    // 分割字段
    char *token = strtok(buf, ",");
    while (token && field_count < 15) {
        fields[field_count++] = token;
        token = strtok(NULL, ",");
    }

    // 至少需要 7 个字段
    if (field_count < 7) return 0;

    // 字段 1: UTC 时间
    if (strlen(fields[1]) >= 6) {
        snprintf(data->utc_time, sizeof(data->utc_time), "%.2s:%.2s:%.2s",
                 fields[1], fields[1] + 2, fields[1] + 4);
    }

    // 字段 6: 定位质量
    data->fix_quality = atoi(fields[6]);

    // 字段 7: 卫星数
    data->satellites = atoi(fields[7]);

    // 如果定位无效，返回
    if (data->fix_quality == 0) {
        data->is_fixed = 0;
        return 1;  // 解析成功但未定位
    }

    // 字段 2-3: 纬度
    if (strlen(fields[2]) > 0) {
        double raw_lat = atof(fields[2]);
        int lat_deg = (int)(raw_lat / 100);
        double lat_min = raw_lat - (lat_deg * 100);
        data->latitude = lat_deg + (lat_min / 60.0f);

        data->lat_dir = fields[3][0];
        if (data->lat_dir == 'S') {
            data->latitude = -data->latitude;
        }
    }

    // 字段 4-5: 经度
    if (strlen(fields[4]) > 0) {
        double raw_lon = atof(fields[4]);
        int lon_deg = (int)(raw_lon / 100);
        double lon_min = raw_lon - (lon_deg * 100);
        data->longitude = lon_deg + (lon_min / 60.0f);

        data->lon_dir = fields[5][0];
        if (data->lon_dir == 'W') {
            data->longitude = -data->longitude;
        }
    }

    // 字段 8: HDOP
    if (field_count > 8 && strlen(fields[8]) > 0) {
        data->hdop = atof(fields[8]);
    }

    // 字段 9: 海拔
    if (field_count > 9 && strlen(fields[9]) > 0) {
        data->altitude = atof(fields[9]);
    }

    data->is_fixed = 1;
    data->is_valid = 1;
    return 1;
}

// 在缓冲区中查找并解析 NMEA 语句
static void parse_nmea_buffer(const char *buffer, uint16_t len) {
    // 查找 $GNGGA 或 $GPGGA
    const char *start = strstr(buffer, "$GNGGA");
    if (!start) start = strstr(buffer, "$GPGGA");
    if (!start) return;

    // 查找语句结束（\r\n 或 *）
    const char *end = strchr(start, '\r');
    if (!end) end = strchr(start, '*');
    if (!end || end <= start) return;

    // 提取语句
    int sentence_len = end - start;
    if (sentence_len > 120) return;  // 语句过长

    char sentence[128];
    strncpy(sentence, start, sentence_len);
    sentence[sentence_len] = '\0';

    // 解析
    GPS_Data_t temp_data = {0};
    if (parse_gga(sentence, &temp_data)) {
        // 更新共享数据
        xSemaphoreTake(gps_mutex, portMAX_DELAY);
        memcpy(&gps_shared_data, &temp_data, sizeof(GPS_Data_t));
        gps_status = temp_data.is_fixed ? GPS_STATUS_FIXED : GPS_STATUS_SEARCHING;
        xSemaphoreGive(gps_mutex);
    }
}

// ==================== 任务主函数 ====================
static void gps_task(void *arg) {
    (void)arg;

    // 错峰启动
    vTaskDelay(pdMS_TO_TICKS(500));

    uart_printf_dma(&huart1, "[GPS] Task Started\r\n");

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_print_tick = 0;

    // 初始挂起，等待启动命令
    gps_running = 0;
    vTaskSuspend(NULL);

    while (1) {
        // 检查是否运行
        if (!gps_running) {
            vTaskSuspend(NULL);
            last_wake = xTaskGetTickCount();
            continue;
        }

        // 从环形缓冲区读取数据
        uint16_t avail = ringbuf_available();
        if (avail > 0) {
            if (avail > sizeof(parse_buffer) - 1) {
                avail = sizeof(parse_buffer) - 1;
            }
            uint16_t read_len = ringbuf_read((uint8_t *)parse_buffer, avail);
            parse_buffer[read_len] = '\0';

            // 解析 NMEA 数据
            parse_nmea_buffer(parse_buffer, read_len);
        }

        // 更新 OLED 显示
        GPS_Data_t data;
        gps_get_data(&data);
        oled_update_gps(data.latitude, data.longitude, data.satellites, data.is_fixed);

        // 发送数据给ESP32
        esp32_send_gps(data.latitude, data.longitude, data.satellites);

        // 定期打印状态
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_print_tick >= GPS_PRINT_INTERVAL) {
            last_print_tick = now;

            xSemaphoreTake(gps_mutex, portMAX_DELAY);
            GPS_Data_t d = gps_shared_data;
            xSemaphoreGive(gps_mutex);

            if (d.is_fixed) {
                uart_printf_dma(&huart1,
                    "[GPS] Fixed! Lat:%.6f%c Lon:%.6f%c Sat:%d Alt:%.1fm\r\n",
                    fabsf(d.latitude), d.lat_dir,
                    fabsf(d.longitude), d.lon_dir,
                    d.satellites, d.altitude);
            } else {
                uart_printf_dma(&huart1,
                    "[GPS] Searching... Satellites:%d\r\n", d.satellites);
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(GPS_PARSE_INTERVAL));
    }
}

// ==================== UART 接收回调 ====================
void gps_uart_rx_callback(uint8_t *data, uint16_t len) {
    // 将数据写入环形缓冲区（带溢出保护）
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next_pos = (rx_write_pos + 1) % GPS_RX_BUFFER_SIZE;
        
        // 缓冲区满时丢弃新数据，避免覆盖未读数据
        if (next_pos == rx_read_pos) {
            break;
        }
        
        gps_rx_buffer[rx_write_pos] = data[i];
        rx_write_pos = next_pos;
    }
}

// ==================== 公共接口 ====================
void gps_task_init(void) {
    // 创建互斥锁
    gps_mutex = xSemaphoreCreateMutex();
    if (!gps_mutex) {
        uart_printf_dma(&huart1, "[ERROR] GPS mutex failed!\r\n");
        return;
    }

    // 初始化共享数据
    memset(&gps_shared_data, 0, sizeof(gps_shared_data));
    gps_status = GPS_STATUS_INIT;

    // 创建任务
    BaseType_t ret = xTaskCreate(
        gps_task,
        "gps",
        512,  // 栈大小（字）- 增大避免溢出
        NULL,
        2,    // 优先级
        &gps_task_handle
    );

    if (ret != pdPASS) {
        uart_printf_dma(&huart1, "[ERROR] GPS task create failed!\r\n");
        return;
    }

    uart_printf_dma(&huart1, "[GPS] Init OK, Task Ready\r\n");
}

void gps_task_start(void) {
    if (gps_task_handle && !gps_running) {
        gps_running = 1;
        // 清空缓冲区
        rx_write_pos = 0;
        rx_read_pos = 0;

        // 启动 UART2 DMA 接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, gps_dma_buffer, 256);
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);  // 禁用半传输中断

        vTaskResume(gps_task_handle);
        uart_printf_dma(&huart1, "[GPS] Task Started, DMA RX Enabled\r\n");
    }
}

void gps_task_stop(void) {
    if (gps_task_handle && gps_running) {
        gps_running = 0;
        // 停止 UART2 DMA 接收
        HAL_UART_DMAStop(&huart2);
        uart_printf_dma(&huart1, "[GPS] Task Stopped\r\n");
    }
}

uint8_t gps_is_running(void) {
    return gps_running;
}

void gps_get_data(GPS_Data_t *data) {
    if (!gps_mutex || !data) return;

    xSemaphoreTake(gps_mutex, portMAX_DELAY);
    memcpy(data, &gps_shared_data, sizeof(GPS_Data_t));
    xSemaphoreGive(gps_mutex);
}

void gps_get_position(float *lat, float *lon, uint8_t *valid) {
    if (!gps_mutex) return;

    xSemaphoreTake(gps_mutex, portMAX_DELAY);
    if (lat) *lat = gps_shared_data.latitude;
    if (lon) *lon = gps_shared_data.longitude;
    if (valid) *valid = gps_shared_data.is_fixed;
    xSemaphoreGive(gps_mutex);
}

uint8_t gps_get_satellites(void) {
    if (!gps_mutex) return 0;

    xSemaphoreTake(gps_mutex, portMAX_DELAY);
    uint8_t sat = gps_shared_data.satellites;
    xSemaphoreGive(gps_mutex);
    return sat;
}

GPS_Status_t gps_get_status(void) {
    return gps_status;
}

uint8_t gps_is_fixed(void) {
    if (!gps_mutex) return 0;

    xSemaphoreTake(gps_mutex, portMAX_DELAY);
    uint8_t fixed = gps_shared_data.is_fixed;
    xSemaphoreGive(gps_mutex);
    return fixed;
}
