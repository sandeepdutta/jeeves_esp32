#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <math.h>
#include <complex>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/stream_buffer.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_task_wdt.h"
#include "rtc_wdt.h"
#include "driver/gpio.h"
#include "M5StickCPlus2.h"
#include <inttypes.h>  // for PRId64
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include <fcntl.h>
#include <errno.h>

extern "C" void app_main();

static constexpr const size_t record_length     = CONFIG_MICROPHONE_SAMPLE_SIZE;
static constexpr const size_t record_samplerate = CONFIG_MICROPHONE_SAMPLE_RATE;
static int16_t rec_data[record_length];

// TCP socket handle
static int tcp_sock = -1;
static SemaphoreHandle_t socket_mutex;

// WiFi connection tracking
static EventGroupHandle_t wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;
static const int WIFI_FAIL_BIT = BIT1;
static int wifi_retry_count = 0;
static const int WIFI_MAX_RETRY = 5;

// Create a StreamBuffer to transfer audio data between tasks
StreamBufferHandle_t audio_stream_buffer;
static constexpr const size_t stream_buffer_size = record_length * sizeof(int16_t) * 5; // Buffer for 5 audio chunks

// Create a semaphore to synchronize initialization of the publishers.
SemaphoreHandle_t init_semaphore;

// Display variables
static constexpr const int display_width = 240;
static constexpr const int display_height = 135;
static constexpr const int bar_width = 4;
static constexpr const int bar_spacing = 2;
static constexpr const int max_bars = display_width / (bar_width + bar_spacing);
static constexpr const int bar_max_height = display_height - 45; // Leave space for text
static constexpr const int bar_y_start = 30;

// RMS calculation variables
static float rms_values[max_bars] = {0};
static int rms_index = 0;
static constexpr const float rms_alpha = 0.1f; // Smoothing factor

// Display task variables
static bool display_ready = false;
static float last_battery_level = -1.0f; // Track last battery level to avoid unnecessary updates
static SemaphoreHandle_t display_semaphore; // Semaphore for conditional display updates
static SemaphoreHandle_t mic_semaphore; // Semaphore for microphone recording

// Function to calculate RMS value from audio samples
float calculate_rms(const int16_t* samples, size_t length) {
    double sum_squares = 0.0;
    for (size_t i = 0; i < length; i++) {
        double sample = static_cast<double>(samples[i]);
        sum_squares += sample * sample;
    }
    return static_cast<float>(sqrt(sum_squares / (length)));
}

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        printf("WiFi started, attempting to connect...\n");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_retry_count < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            wifi_retry_count++;
            printf("Retry connecting to WiFi... (attempt %d/%d)\n", wifi_retry_count, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
            printf("Failed to connect to WiFi after %d attempts\n", WIFI_MAX_RETRY);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        printf("Got IP address: " IPSTR "\n", IP2STR(&event->ip_info.ip));
        wifi_retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Wake word handling function
void handle_wake_word(int32_t wake_word_value)
{
    printf("Wake word detected: %ld\n", wake_word_value);
    xSemaphoreTake(mic_semaphore, portMAX_DELAY);
    while(StickCP2.Mic.isRecording()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    StickCP2.Mic.end();
    StickCP2.Speaker.begin();
    // Play the wake word tone
    int tone_1 = 3000/2;
    int tone_2 = 4000/2;
    if (wake_word_value == 1) {
        tone_1 = 3000/2;
        tone_2 = 4000/2;
    } else {
        tone_1 = 4000/2;
        tone_2 = 4000/2;
    }
    StickCP2.Speaker.setVolume(100);
    StickCP2.Speaker.tone(tone_1, 150);
    vTaskDelay(pdMS_TO_TICKS(150));
    StickCP2.Speaker.tone(tone_2, 150);
    vTaskDelay(pdMS_TO_TICKS(150));
    StickCP2.Speaker.tone(tone_1, 150);
    vTaskDelay(pdMS_TO_TICKS(150));
    while(StickCP2.Speaker.isPlaying()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    StickCP2.Speaker.end();
    StickCP2.Mic.begin();
    xSemaphoreGive(mic_semaphore);
}

// Function to draw the bar graph
void draw_bar_graph() {
    static bool first_draw = true;
    
    if (first_draw) {
        // First time: clear entire screen and draw static elements
        StickCP2.Display.fillScreen(BLACK);
        
        // Draw title (static)
        StickCP2.Display.setTextSize(1.25f);
        StickCP2.Display.setTextColor(WHITE);
        StickCP2.Display.setCursor(5, 5);
        StickCP2.Display.println("Jeeves voice commands");
        
        first_draw = false;
    } else {
        // Clear only the dynamic areas
        // Clear bar area
        StickCP2.Display.fillRect(5, bar_y_start, display_width - 10, bar_max_height, BLACK);
        // Clear RMS text area
        StickCP2.Display.fillRect(5, display_height - 20, 100, 15, BLACK);
    }
    
    // Draw bars
    for (int i = 0; i < max_bars; i++) {
        int x = i * (bar_width + bar_spacing) + 5;
        int bar_height = static_cast<int>(rms_values[i] * bar_max_height / 6000.0f); // Normalize with smaller divisor for taller bars
        bar_height = std::min(bar_height, bar_max_height);
        
        // Color based on intensity
        uint16_t color;
        if (bar_height < bar_max_height / 3) {
            color = GREEN;
        } else if (bar_height < 2 * bar_max_height / 3) {
            color = YELLOW;
        } else {
            color = RED;
        }
        
        // Draw the bar
        StickCP2.Display.fillRect(x, bar_y_start + bar_max_height - bar_height, bar_width, bar_height, color);
    }
    
    // Draw battery level bar (only if it changed by more than 5%)
    float battery_level = StickCP2.Power.getBatteryLevel() / 100.0f; // Get battery level as 0.0 to 1.0
    
    if (abs(battery_level - last_battery_level) > 0.05f) { // 5% threshold
        // Clear battery area
        int battery_bar_width = 60;
        int battery_bar_height = 8;
        int battery_x = display_width - battery_bar_width - 5;
        int battery_y = 5;
        StickCP2.Display.fillRect(battery_x, battery_y, battery_bar_width, battery_bar_height + 10, BLACK);
        
        // Draw battery outline
        StickCP2.Display.drawRect(battery_x, battery_y, battery_bar_width, battery_bar_height, WHITE);
        
        // Draw battery level fill
        int fill_width = static_cast<int>(battery_level * (battery_bar_width - 2));
        uint16_t battery_color;
        if (battery_level > 0.5f) {
            battery_color = GREEN;
        } else if (battery_level > 0.2f) {
            battery_color = YELLOW;
        } else {
            battery_color = RED;
        }
        
        if (fill_width > 0) {
            StickCP2.Display.fillRect(battery_x + 1, battery_y + 1, fill_width, battery_bar_height - 2, battery_color);
        }
        
        // Show battery percentage
        StickCP2.Display.setTextSize(1);
        StickCP2.Display.setTextColor(WHITE);
        StickCP2.Display.setCursor(battery_x, battery_y + battery_bar_height + 2);
        StickCP2.Display.printf("%.0f%%", battery_level * 100.0f);
        
        last_battery_level = battery_level;
    }
    
    // Show current RMS value
    StickCP2.Display.setTextSize(1);
    StickCP2.Display.setTextColor(WHITE);
    StickCP2.Display.setCursor(5, display_height - 10);
    StickCP2.Display.printf("RMS: %.1f", rms_values[rms_index]);
    
    StickCP2.Display.display();
}

void display_task(void * arg) {
    printf("display_task created\n");
    
    // Wait for the display to be ready
    while (!display_ready) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    printf("Starting display updates\n");
    
    while (1) {
        // Wait for semaphore signal to update display (only when in yellow range)
        if (xSemaphoreTake(display_semaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
            draw_bar_graph();
        } else {
            // Clear bar graph area when no updates (audio too quiet)
            StickCP2.Display.fillRect(5, bar_y_start, display_width - 10, bar_max_height, BLACK);
            StickCP2.Display.display();
        }
        
        // Small delay to prevent excessive CPU usage
        vTaskDelay(pdMS_TO_TICKS(100)); // 10 FPS
    }
}

void mic_record_task(void * arg) {
    printf("mic_record task created\n");
    // Wait for the publishers to be initialized.
    xSemaphoreTake(init_semaphore, portMAX_DELAY);
    printf("Starting to record\n");
    int i = 0;
    xSemaphoreGive(mic_semaphore);
    while (1) {
        if (xSemaphoreTake(mic_semaphore, portMAX_DELAY) == pdTRUE && 
            StickCP2.Mic.record(rec_data, record_length, record_samplerate)) {
            xSemaphoreGive(mic_semaphore);
            // Calculate RMS value and update display
            float current_rms = calculate_rms(rec_data, record_length);
            // Only update bar graph if level is in yellow range (medium intensity)
            int bar_height = static_cast<int>(current_rms * bar_max_height / 10000.0f);
            bar_height = std::min(bar_height, bar_max_height);
            
            // Send the audio data to the StreamBuffer
            size_t bytes_sent = xStreamBufferSend(
                audio_stream_buffer,
                rec_data,
                record_length * sizeof(int16_t),
                portMAX_DELAY
            );
            
            if (bytes_sent == record_length * sizeof(int16_t)) {
                // Store raw RMS value without smoothing
                rms_values[rms_index] = current_rms;
                
                if (bar_height >= bar_max_height / 3 || i == 0) {
                    // Signal display task to update when in yellow range or higher or first time
                    xSemaphoreGive(display_semaphore);
                }
                
                // Shift RMS values for scrolling effect
                rms_index = (rms_index + 1) % max_bars;
                
                if (i % 100 == 0) {
                    printf("Sent %d audio chunks to stream buffer %f\n", i++, current_rms);
                } else {
                    i++;
                }
            } else {
                printf("Failed to send audio data to stream buffer, sent %zu bytes\n", bytes_sent);
            }
        } else {
            printf("Failed to record or semaphore not taken\n");
        }
        if (StickCP2.BtnB.wasClicked()) {
            printf("Button B clicked\n");
        }
        if (StickCP2.BtnA.wasHold()) {
            printf("Button A hold\n");
            StickCP2.Display.fillScreen(BLACK);
            StickCP2.Display.display();
            StickCP2.Power.deepSleep(0,true);
            esp_restart();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// TCP/IP communication task
void tcp_comm_task(void * arg)
{
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(CONFIG_SERVER_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(CONFIG_SERVER_PORT);

    uint8_t* audio_buffer = (uint8_t*)malloc(record_length * sizeof(int16_t));
    if (audio_buffer == NULL) {
        printf("Failed to allocate audio buffer\n");
        vTaskDelete(NULL);
        return;
    }

    int published_count = 0;

    while(1) {
        // Try to connect to the server
        tcp_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (tcp_sock < 0) {
            printf("Unable to create socket: errno %d\n", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        printf("Socket created, connecting to %s:%d\n", CONFIG_SERVER_IP, CONFIG_SERVER_PORT);

        int err = connect(tcp_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            printf("Socket unable to connect: errno %d\n", errno);
            close(tcp_sock);
            tcp_sock = -1;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        printf("Successfully connected\n");

        // Set socket to non-blocking mode for receiving
        int flags = fcntl(tcp_sock, F_GETFL, 0);
        fcntl(tcp_sock, F_SETFL, flags | O_NONBLOCK);

        // Signal that the connection is initialized
        xSemaphoreGive(init_semaphore);
        printf("TCP connection initialized and ready to send/receive data\n");

        // Main communication loop
        while(1) {
            StickCP2.update();

            // Try to receive audio data from the StreamBuffer
            size_t bytes_received = xStreamBufferReceive(
                audio_stream_buffer,
                audio_buffer,
                record_length * sizeof(int16_t),
                pdMS_TO_TICKS(10) // 10ms timeout - non-blocking
            );

            if (bytes_received == record_length * sizeof(int16_t)) {
                // Send the audio data over TCP
                xSemaphoreTake(socket_mutex, portMAX_DELAY);
                int sent = send(tcp_sock, audio_buffer, bytes_received, 0);
                xSemaphoreGive(socket_mutex);

                if (sent < 0) {
                    printf("Error occurred during sending: errno %d\n", errno);
                    break; // Break to reconnect
                }

                if (published_count % 100 == 0) {
                    printf("Sent %d audio chunks\n", published_count);
                }
                published_count++;
            }

            // Try to receive wake word data (non-blocking)
            int32_t wake_word_value;
            xSemaphoreTake(socket_mutex, portMAX_DELAY);
            int len = recv(tcp_sock, &wake_word_value, sizeof(int32_t), 0);
            xSemaphoreGive(socket_mutex);

            if (len > 0) {
                printf("Received wake word data: %ld\n", wake_word_value);
                handle_wake_word(wake_word_value);
            } else if (len == 0) {
                printf("Connection closed by server\n");
                break; // Break to reconnect
            } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
                printf("Error occurred during receiving: errno %d\n", errno);
                break; // Break to reconnect
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Close the socket and reconnect
        printf("Closing socket and reconnecting...\n");
        if (tcp_sock >= 0) {
            shutdown(tcp_sock, SHUT_RDWR);
            close(tcp_sock);
            tcp_sock = -1;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    free(audio_buffer);
    vTaskDelete(NULL);
}

extern "C" void app_main() {
    auto cfg = M5.config();
    StickCP2.begin(cfg);

    // Since the microphone and speaker cannot be used at the same time,
    // turn off the speaker here.
    StickCP2.Speaker.end();
    auto mic_cfg = StickCP2.Mic.config();
    mic_cfg.sample_rate = CONFIG_MICROPHONE_SAMPLE_RATE;
    mic_cfg.over_sampling = CONFIG_MICROPHONE_OVERSAMPLE;
    mic_cfg.magnification = CONFIG_MICROPHONE_GAIN;
    StickCP2.Mic.config(mic_cfg);
    StickCP2.Mic.begin();
    printf("Mic config: sample_rate: %ld, over_sampling: %d, magnification: %d\n", 
            mic_cfg.sample_rate, mic_cfg.over_sampling, mic_cfg.magnification);
    // Initialize display
    StickCP2.Display.setRotation(1); // Landscape orientation
    StickCP2.Display.fillScreen(RED); // Fill screen with red during initialization
    StickCP2.Display.setTextSize(1);
    StickCP2.Display.setTextColor(WHITE);
    StickCP2.Display.setCursor(5, 5);
    StickCP2.Display.println("Initializing Audio RMS Display...");
    StickCP2.Display.display();

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Create WiFi event group
    wifi_event_group = xEventGroupCreate();

    // Initialize network interface
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Initialize WiFi
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, CONFIG_ESP_WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, CONFIG_ESP_WIFI_PASSWORD);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    printf("WiFi initialization complete. Connecting to %s...\n", CONFIG_ESP_WIFI_SSID);

    // Wait for connection (with timeout)
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        printf("Connected to WiFi SSID: %s\n", CONFIG_ESP_WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        printf("Failed to connect to WiFi SSID: %s after %d retries\n", CONFIG_ESP_WIFI_SSID, WIFI_MAX_RETRY);
    } else {
        printf("Unexpected WiFi connection event\n");
    }

    // Create the semaphore for initialization synchronization
    init_semaphore = xSemaphoreCreateBinary();

    // Create the semaphore for display synchronization
    display_semaphore = xSemaphoreCreateBinary();

    // Create the semaphore for microphone recording
    mic_semaphore = xSemaphoreCreateBinary();

    // Create the mutex for socket access
    socket_mutex = xSemaphoreCreateMutex();

    // Create the StreamBuffer for audio data transfer
    audio_stream_buffer = xStreamBufferCreate(
        stream_buffer_size,
        record_length * sizeof(int16_t) // Trigger level - one complete audio chunk
    );

    if (audio_stream_buffer == NULL) {
        printf("Failed to create audio stream buffer\n");
        esp_restart();
    }

    printf("Audio stream buffer created with size %zu bytes\n", stream_buffer_size);

    // Create TCP communication task
    xTaskCreate(tcp_comm_task,
            "tcp_comm_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);

    // Create a task to record the audio on CORE 0.
    xTaskCreate(mic_record_task,
            "mic_record_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
            
    // Create a task to handle display updates (lower priority)
    xTaskCreate(display_task,
            "display_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO - 2, // Much lower priority than other tasks
            NULL);
            
    // Signal that display is ready
    display_ready = true;

    // Start the scheduler.
    vTaskStartScheduler();
}

