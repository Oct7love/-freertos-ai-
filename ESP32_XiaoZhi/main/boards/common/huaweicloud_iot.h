#ifndef __HUAWEICLOUD_IOT_H__
#define __HUAWEICLOUD_IOT_H__

#include "board.h"
#include <mqtt.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cJSON.h>
#include <ctime>
#include <string>
#include <memory>

// 华为云IoT连接配置
#define HUAWEI_IOT_BROKER     "00a64b9e07.st1.iotda-device.cn-south-1.myhuaweicloud.com"
#define HUAWEI_IOT_PORT       8883  // MQTTS需要TLS端口
#define HUAWEI_IOT_DEVICE_ID  "695cb5e3c9429d337f25cf62_smart_helmat_ai"
#define HUAWEI_IOT_PASSWORD   "db01fd1f6fb3bdbca6474dced67fe6ff65bcf1e33802e92aef858a437f0d791b"
#define HUAWEI_IOT_CLIENT_TS  "2026010613"  // 固定时间戳（匹配密码签名）

// Topic格式
#define HUAWEI_IOT_TOPIC_REPORT "$oc/devices/" HUAWEI_IOT_DEVICE_ID "/sys/properties/report"

// 上报间隔（毫秒）
#define HUAWEI_IOT_REPORT_INTERVAL_MS  30000

class HuaweiCloudIoT {
private:
    static constexpr const char* TAG = "HuaweiCloudIoT";

    std::unique_ptr<Mqtt> mqtt_;
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

    // 生成ClientID（华为云格式：deviceId_0_0_时间戳）
    std::string GenerateClientId() {
        char client_id[128];
        // 使用固定时间戳匹配预生成的HMAC密码
        snprintf(client_id, sizeof(client_id),
                 "%s_0_0_%s", HUAWEI_IOT_DEVICE_ID, HUAWEI_IOT_CLIENT_TS);
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
        // 延迟15秒等待网络初始化完成
        ESP_LOGI(TAG, "华为云IoT：等待网络就绪（15秒）...");
        vTaskDelay(pdMS_TO_TICKS(15000));

        while (running_) {
            // 检查并建立连接
            if (!mqtt_ || !mqtt_->IsConnected()) {
                ESP_LOGI(TAG, "尝试连接华为云...");
                if (!TryConnect()) {
                    ESP_LOGW(TAG, "连接失败，30秒后重试");
                    vTaskDelay(pdMS_TO_TICKS(30000));
                    continue;
                }
            }

            // 上报数据
            std::string payload = BuildReportPayload();
            bool success = mqtt_->Publish(HUAWEI_IOT_TOPIC_REPORT, payload, 0);
            if (success) {
                ESP_LOGI(TAG, "数据上报成功");
            } else {
                ESP_LOGW(TAG, "数据上报失败");
            }

            vTaskDelay(pdMS_TO_TICKS(HUAWEI_IOT_REPORT_INTERVAL_MS));
        }
        vTaskDelete(nullptr);
    }

    bool TryConnect() {
        auto network = Board::GetInstance().GetNetwork();
        if (!network) {
            ESP_LOGW(TAG, "网络模块未初始化");
            return false;
        }

        if (!mqtt_) {
            mqtt_ = network->CreateMqtt(1);
            if (!mqtt_) {
                ESP_LOGE(TAG, "创建MQTT实例失败");
                return false;
            }

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
        }

        std::string client_id = GenerateClientId();
        ESP_LOGI(TAG, "连接华为云: %s", HUAWEI_IOT_BROKER);

        return mqtt_->Connect(
            HUAWEI_IOT_BROKER,
            HUAWEI_IOT_PORT,
            client_id,
            HUAWEI_IOT_DEVICE_ID,
            HUAWEI_IOT_PASSWORD
        );
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

        running_ = true;
        // 非阻塞启动：仅创建后台任务，连接逻辑在任务中执行
        xTaskCreate(ReportTaskStatic, "huawei_iot", 4096, this, 3, &report_task_handle_);

        ESP_LOGI(TAG, "华为云IoT任务已启动（延迟连接模式）");
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
