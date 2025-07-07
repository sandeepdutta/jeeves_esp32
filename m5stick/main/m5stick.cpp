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
#include "driver/i2s.h"
#include "driver/i2s_std.h"
#include "driver/i2s_pdm.h"
#include "M5GFX.h"
#include <inttypes.h>  // for PRId64

/* WiFi */
#define WIFI_SSID "jeeves_one"
#define WIFI_PASS "$jeeves$"

#define WIFI_CONNECTED_BIT     BIT0
#define WIFI_DISCONNECTED_BIT  BIT1

static const char* TAG = "wifi_connect";
static const char* MIC_TAG = "microphone";
static EventGroupHandle_t wifi_event_group;

M5GFX display;

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

        display.fillScreen(TFT_BLACK);
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.setTextColor(TFT_WHITE);
        display.println("Connecting...");

        esp_wifi_connect();

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

            display.fillScreen(TFT_GREEN);
            display.setTextColor(TFT_BLACK, TFT_GREEN);
            display.setTextSize(2);
            display.setCursor(0, 0);
            display.printf("SSID:%s\nIP  :%s\n", ap_info.ssid, ip_str);

            xEventGroupWaitBits(wifi_event_group,
                                WIFI_DISCONNECTED_BIT,
                                pdTRUE,
                                pdFALSE,
                                portMAX_DELAY);

            ESP_LOGW(TAG, "Wi-Fi disconnected");
            display.fillScreen(TFT_RED);
            display.setCursor(0, 0);
            display.setTextColor(TFT_WHITE, TFT_RED);
            display.setTextSize(2);
            display.println("Disconnected");

            vTaskDelay(pdMS_TO_TICKS(2000));
        } else {
            ESP_LOGW(TAG, "Connection failed or timeout");
            display.fillScreen(TFT_RED);
            display.setCursor(0, 0);
            display.setTextColor(TFT_WHITE, TFT_RED);
            display.setTextSize(2);
            display.println("Failed");
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


bool InitI2SMicroPhone()
{
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        // .sample_rate = 44100,
        .sample_rate = 48000,
        .bits_per_sample =
            I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 1, 0)
        .communication_format =
            I2S_COMM_FORMAT_STAND_I2S, // Set the format of the communication.
#else                                      // 设置通讯格式
        .communication_format = I2S_COMM_FORMAT_I2S,
#endif
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 2,
        .dma_buf_len = 128,
    };

    i2s_pin_config_t pin_config;
#if (ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 3, 0))
    pin_config.mck_io_num = I2S_PIN_NO_CHANGE;
#endif
    pin_config.bck_io_num = I2S_PIN_NO_CHANGE;
    pin_config.ws_io_num = GPIO_NUM_0;
    pin_config.data_out_num = I2S_PIN_NO_CHANGE;
    pin_config.data_in_num = GPIO_NUM_34;

    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, &pin_config));
    ESP_ERROR_CHECK(i2s_set_clk(I2S_NUM_0, 48000, I2S_BITS_PER_SAMPLE_16BIT,
                       I2S_CHANNEL_MONO));

    ESP_LOGI(MIC_TAG, "I2S microphone initialized");
    return true;

}

extern "C" void app_main() {
    ESP_ERROR_CHECK(nvs_flash_init());

    display.begin();
    display.setRotation(1);  // Landscape
    display.setTextWrap(true);

    static WiFiManager wifi;
    wifi.init();
    InitI2SMicroPhone();
    xTaskCreate(&WiFiManager::connect_task, "wifi_connect_task", 4096, nullptr, 5, nullptr);
    
}
