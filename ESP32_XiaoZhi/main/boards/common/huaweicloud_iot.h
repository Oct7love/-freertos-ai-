#ifndef __HUAWEICLOUD_IOT_H__
#define __HUAWEICLOUD_IOT_H__

#include "esp/esp_mqtt.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cJSON.h>
#include <string>
#include <memory>

// 华为云IoT连接配置
#define HUAWEI_IOT_BROKER     "00a64b9e07.st1.iotda-device.cn-south-1.myhuaweicloud.com"
#define HUAWEI_IOT_PORT       1883
#define HUAWEI_IOT_DEVICE_ID  "695cb5e3c9429d337f25cf62_smart_helmat_ai"
#define HUAWEI_IOT_SECRET     "695cb5e3c9429d337f25cf62_smart_helmat_ai"

// Topic格式
#define HUAWEI_IOT_TOPIC_REPORT "$oc/devices/" HUAWEI_IOT_DEVICE_ID "/sys/properties/report"

// 上报间隔（毫秒）
#define HUAWEI_IOT_REPORT_INTERVAL_MS  30000

class HuaweiCloudIoT {
private:
    static constexpr const char* TAG = "HuaweiCloudIoT";

    std::unique_ptr<EspMqtt> mqtt_;
    TaskHandle_t report_task_handle_ = nullptr;
    bool running_ = false;

    // 传感器数据（由STM32Controller更新）
    float latitude_ = 0.0f;
    float longitude_ = 0.0f;
    float temperature_ = 0.0f;
    int humidity_ = 0;
    int heart_rate_ = 0;
    int blood_oxygen_ = 0;
    int fall_flag_ = 0;
    int collision_flag_ = 0;

    // 生成ClientID（含时间戳）
    std::string GenerateClientId() {
        char client_id[128];
        uint32_t timestamp = (uint32_t)(esp_timer_get_time() / 1000000);
        snprintf(client_id, sizeof(client_id),
                 "%s_0_0_%u", HUAWEI_IOT_DEVICE_ID, timestamp);
        return std::string(client_id);
    }

    // 构建上报JSON
    std::string BuildReportPayload() {
        cJSON* root = cJSON_CreateObject();
        cJSON* services = cJSON_CreateArray();
        cJSON* service = cJSON_CreateObject();
        cJSON* properties = cJSON_CreateObject();

        cJSON_AddStringToObject(service, "service_id", "smart_hat");

        cJSON_AddNumberToObject(properties, "Longitude", longitude_);
        cJSON_AddNumberToObject(properties, "Latitude", latitude_);
        cJSON_AddNumberToObject(properties, "Temperature", temperature_);
        cJSON_AddNumberToObject(properties, "Humidity", humidity_);
        cJSON_AddNumberToObject(properties, "HeartRate", heart_rate_);
        cJSON_AddNumberToObject(properties, "BloodOxygen", blood_oxygen_);
        cJSON_AddNumberToObject(properties, "FallFlag", fall_flag_);
        cJSON_AddNumberToObject(properties, "CollisionFlag", collision_flag_);

        cJSON_AddItemToObject(service, "properties", properties);
        cJSON_AddItemToArray(services, service);
        cJSON_AddItemToObject(root, "services", services);

        char* json_str = cJSON_PrintUnformatted(root);
        std::string payload(json_str);
        cJSON_free(json_str);
        cJSON_Delete(root);

        return payload;
    }

    static void ReportTaskStatic(void* arg) {
        HuaweiCloudIoT* iot = (HuaweiCloudIoT*)arg;
        iot->ReportTask();
    }

    void ReportTask() {
        while (running_) {
            if (mqtt_ && mqtt_->IsConnected()) {
                std::string payload = BuildReportPayload();
                bool success = mqtt_->Publish(HUAWEI_IOT_TOPIC_REPORT, payload, 0);
                if (success) {
                    ESP_LOGI(TAG, "数据上报成功");
                } else {
                    ESP_LOGW(TAG, "数据上报失败");
                }
            } else {
                ESP_LOGW(TAG, "MQTT未连接，跳过上报");
            }
            vTaskDelay(pdMS_TO_TICKS(HUAWEI_IOT_REPORT_INTERVAL_MS));
        }
        vTaskDelete(nullptr);
    }

public:
    HuaweiCloudIoT() {
        ESP_LOGI(TAG, "HuaweiCloudIoT 初始化");
    }

    ~HuaweiCloudIoT() {
        Stop();
    }

    bool Start() {
        if (running_) {
            ESP_LOGW(TAG, "已在运行中");
            return true;
        }

        mqtt_ = std::make_unique<EspMqtt>();
        mqtt_->SetKeepAlive(120);

        mqtt_->OnConnected([this]() {
            ESP_LOGI(TAG, "华为云IoT连接成功");
        });

        mqtt_->OnDisconnected([this]() {
            ESP_LOGW(TAG, "华为云IoT连接断开");
        });

        mqtt_->OnError([](const std::string& error) {
            ESP_LOGE("HuaweiCloudIoT", "MQTT错误: %s", error.c_str());
        });

        std::string client_id = GenerateClientId();
        ESP_LOGI(TAG, "连接华为云: broker=%s, client_id=%s",
                 HUAWEI_IOT_BROKER, client_id.c_str());

        bool connected = mqtt_->Connect(
            HUAWEI_IOT_BROKER,
            HUAWEI_IOT_PORT,
            client_id,
            HUAWEI_IOT_DEVICE_ID,
            HUAWEI_IOT_SECRET
        );

        if (!connected) {
            ESP_LOGE(TAG, "连接华为云失败");
            mqtt_.reset();
            return false;
        }

        running_ = true;
        xTaskCreate(ReportTaskStatic, "huawei_iot", 4096, this, 3, &report_task_handle_);

        ESP_LOGI(TAG, "华为云IoT启动成功，上报间隔: %dms", HUAWEI_IOT_REPORT_INTERVAL_MS);
        return true;
    }

    void Stop() {
        running_ = false;
        if (mqtt_) {
            mqtt_->Disconnect();
            mqtt_.reset();
        }
        ESP_LOGI(TAG, "华为云IoT已停止");
    }

    bool IsConnected() {
        return mqtt_ && mqtt_->IsConnected();
    }

    // 更新传感器数据（由STM32Controller调用）
    void UpdateGPS(float lat, float lon) {
        latitude_ = lat;
        longitude_ = lon;
    }

    void UpdateTemperature(float temp, int humi) {
        temperature_ = temp;
        humidity_ = humi;
    }

    void UpdateHeartRate(int hr, int spo2) {
        heart_rate_ = hr;
        blood_oxygen_ = spo2;
    }

    void SetFallFlag(int flag) {
        fall_flag_ = flag;
    }

    void SetCollisionFlag(int flag) {
        collision_flag_ = flag;
    }

    // 立即上报（用于事件触发）
    bool ReportNow() {
        if (!mqtt_ || !mqtt_->IsConnected()) {
            return false;
        }
        std::string payload = BuildReportPayload();
        return mqtt_->Publish(HUAWEI_IOT_TOPIC_REPORT, payload, 0);
    }
};

#endif // __HUAWEICLOUD_IOT_H__
