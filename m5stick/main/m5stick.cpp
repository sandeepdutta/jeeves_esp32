#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <math.h>
#include <complex>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "M5StickCPlus2.h"
#include <inttypes.h>  // for PRId64
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>


/* WiFi */
#define WIFI_SSID "jeeves_one"
#define WIFI_PASS "$jeeves$"

#define WIFI_CONNECTED_BIT     BIT0
#define WIFI_DISCONNECTED_BIT  BIT1

static const char* TAG = "wifi_connect";
static EventGroupHandle_t wifi_event_group;
// Semaphore to wait for the WiFi connection to be established
static SemaphoreHandle_t wifi_semaphore;
static std::atomic<bool> wifi_connected(false);

extern "C" void app_main();

class WiFiManager {
public:
    void init();
    static void connect_task(void* arg);

private:
    static void event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data);
};



void WiFiManager::event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "STA started");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Disconnected from AP");
        xEventGroupSetBits(wifi_event_group, WIFI_DISCONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        auto* event = static_cast<ip_event_got_ip_t*>(event_data);
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void WiFiManager::connect_task(void* arg) {
    while (true) {
        ESP_LOGI(TAG, "Attempting to connect to Wi-Fi...");
        esp_wifi_connect();

        wifi_connected = false;
        xSemaphoreTake(wifi_semaphore, portMAX_DELAY); // lock the semaphore
        EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                               WIFI_CONNECTED_BIT | WIFI_DISCONNECTED_BIT,
                                               pdTRUE,
                                               pdFALSE,
                                               pdMS_TO_TICKS(10000));
        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "Successfully connected");

            wifi_ap_record_t ap_info;
            esp_wifi_sta_get_ap_info(&ap_info);

            esp_netif_ip_info_t ip_info;
            esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
            esp_netif_get_ip_info(netif, &ip_info);

            char ip_str[16];
            snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
            ESP_LOGI(TAG, "Connected to %s : %s", ap_info.ssid, ip_str);
            xSemaphoreGive(wifi_semaphore); // unlock the semaphore
            wifi_connected = true;
            xEventGroupWaitBits(wifi_event_group,
                                WIFI_DISCONNECTED_BIT,
                                pdTRUE,
                                pdFALSE,
                                portMAX_DELAY);

            ESP_LOGW(TAG, "Wi-Fi disconnected");
            vTaskDelay(pdMS_TO_TICKS(2000));
        } else {
            ESP_LOGW(TAG, "Connection failed or timeout");
            esp_wifi_disconnect();
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
}

void WiFiManager::init() {
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &WiFiManager::event_handler,
                                                        nullptr,
                                                        nullptr));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &WiFiManager::event_handler,
                                                        nullptr,
                                                        nullptr));

    wifi_config_t wifi_config = {};
    std::strncpy(reinterpret_cast<char*>(wifi_config.sta.ssid), WIFI_SSID, sizeof(wifi_config.sta.ssid));
    std::strncpy(reinterpret_cast<char*>(wifi_config.sta.password), WIFI_PASS, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static constexpr const size_t record_length     = 256;
static constexpr const size_t record_samplerate = 16000;
static int16_t prev_y[record_length];
static int16_t prev_h[record_length];
static int16_t rec_data[record_length];

void mic_record() {
    const int shift = 6;
    if (StickCP2.Mic.record(rec_data, record_length, record_samplerate)) {
        int32_t w = StickCP2.Display.width();
        if (w > record_length - 1) {
            w = record_length - 1;
        }
        ESP_LOGI(TAG, "Using %d samples of %d", (int)w, record_length);
        int32_t max_y = 0;
        int32_t min_y = 0;
        float rms = 0;
        for (int32_t x = 0; x < w; ++x) {
            StickCP2.Display.writeFastVLine(x, prev_y[x], prev_h[x],
                                            TFT_BLACK);
            int32_t y1 = (rec_data[x] >> shift);
            int32_t y2 = (rec_data[x + 1] >> shift);
            if (y1 > y2) {
                int32_t tmp = y1;
                y1          = y2;
                y2          = tmp;
            }
            if (y1 > max_y) {
                max_y = y1;
            }
            if (y1 < min_y) {
                min_y = y1;
            }
            rms += y1 * y1;
            int32_t y = ((StickCP2.Display.height()) >> 1) + y1;
            int32_t h = ((StickCP2.Display.height()) >> 1) + y2 + 1 - y;
            prev_y[x] = y;
            prev_h[x] = h;
            StickCP2.Display.writeFastVLine(x, prev_y[x], prev_h[x], WHITE);
        }
        rms = sqrt(rms / w);
        ESP_LOGI(TAG, "Max: %d, Min: %d, RMS: %f", (int)max_y, (int)min_y, rms);
        StickCP2.Display.display();
        StickCP2.Display.fillCircle(70, 15, 8, RED);
        StickCP2.Display.drawString("REC", 120, 3);
    } else {
        ESP_LOGE(TAG, "Failed to record");
    }
}

void loop_task(void* arg) {
    bool update_display = true;
    bool previous_wifi_connected = false;
    while (true) {
        StickCP2.update();
        vTaskDelay(pdMS_TO_TICKS(100));
        if (!wifi_connected) {
            continue;
        }
        if (StickCP2.Mic.isEnabled()) {
            mic_record();
            ESP_LOGI(TAG, "Mic is enabled");
        } else {
            ESP_LOGI(TAG, "Mic is disabled");
        }
    }
}

extern "C" void app_main() {
    ESP_ERROR_CHECK(nvs_flash_init());
    auto cfg = M5.config();
    StickCP2.begin(cfg);
    if (!(wifi_semaphore = xSemaphoreCreateBinary())) {
        ESP_LOGE(TAG, "Failed to create wifi semaphore");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    } else {
        xSemaphoreGive(wifi_semaphore);
    }
    StickCP2.Speaker.end(); // turn speaker off
    StickCP2.Mic.begin(); // turn microphone on
    StickCP2.Display.startWrite();
    StickCP2.Display.setRotation(1);
    StickCP2.Display.setTextDatum(top_center);
    StickCP2.Display.setTextColor(WHITE);
    StickCP2.Display.setFont(&fonts::FreeSansBoldOblique12pt7b);

    static WiFiManager wifi;
    wifi.init();
    xTaskCreate(&WiFiManager::connect_task, "wifi_connect_task", 4096, nullptr, 5, nullptr);
    xTaskCreate(&loop_task, "loop_task", 4096, nullptr, 5, nullptr);

}
