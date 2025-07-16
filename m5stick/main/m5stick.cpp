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
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rmw_microros/rmw_microros.h>
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <audio_common_msgs/msg/audio_info.h>
#include <audio_common_msgs/msg/audio_data.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

extern "C" void app_main();

static constexpr const size_t record_length     = CONFIG_MICROPHONE_SAMPLE_SIZE;
static constexpr const size_t record_samplerate = CONFIG_MICROPHONE_SAMPLE_RATE;
static int16_t rec_data[record_length];
rcl_publisher_t audio_data_publisher, audio_info_publisher;
audio_common_msgs__msg__AudioData audio_data_msg;
audio_common_msgs__msg__AudioInfo audio_info_msg;

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

// Function to calculate RMS value from audio samples
float calculate_rms(const int16_t* samples, size_t length) {
    double sum_squares = 0.0;
    for (size_t i = 0; i < length; i++) {
        double sample = static_cast<double>(samples[i]);
        sum_squares += sample * sample;
    }
    return static_cast<float>(sqrt(sum_squares / (length)));
}

// Function to draw the bar graph
void draw_bar_graph() {
    StickCP2.Display.clear();
    StickCP2.Display.setTextSize(1);
    StickCP2.Display.setTextColor(WHITE);
    StickCP2.Display.setCursor(5, 5);
    StickCP2.Display.println("Audio RMS Level");
    
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
    
    // Draw battery level bar
    float battery_level = StickCP2.Power.getBatteryLevel() / 100.0f; // Get battery level as 0.0 to 1.0
    int battery_bar_width = 60;
    int battery_bar_height = 8;
    int battery_x = display_width - battery_bar_width - 5;
    int battery_y = 5;
    
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
    
    // Show current RMS value
    StickCP2.Display.setTextSize(1);
    StickCP2.Display.setTextColor(WHITE);
    StickCP2.Display.setCursor(5, display_height - 10);
    StickCP2.Display.printf("RMS: %.1f", rms_values[rms_index]);
    
    StickCP2.Display.display();
}

void mic_record(void * arg) {
    printf("mic_record task created\n");
    // Wait for the publishers to be initialized.
    xSemaphoreTake(init_semaphore, portMAX_DELAY);
    printf("Starting to record\n");
    int i = 0;
    while (1) {
        if (StickCP2.Mic.record(rec_data, record_length, record_samplerate)) {
            // Send the audio data to the StreamBuffer
            size_t bytes_sent = xStreamBufferSend(
                audio_stream_buffer,
                rec_data,
                record_length * sizeof(int16_t),
                pdMS_TO_TICKS(100) // 100ms timeout
            );
            
            if (bytes_sent == record_length * sizeof(int16_t)) {
                // Calculate RMS value and update display
                float current_rms = calculate_rms(rec_data, record_length);
                
                // Store raw RMS value without smoothing
                rms_values[rms_index] = current_rms;
                
                // Update display every 10 samples for smooth animation
                if (i % 5 == 0) {
                    draw_bar_graph();
                }
                
                // Shift RMS values for scrolling effect
                rms_index = (rms_index + 1) % max_bars;
                
                if (i % 100 == 0) {
                    printf("Sent %d audio chunks to stream buffer\n", i++);
                } else {
                    i++;
                }
            } else {
                printf("Failed to send audio data to stream buffer, sent %zu bytes\n", bytes_sent);
            }
        } else {
            printf("Failed to record\n");
        }
    }
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

#ifdef CONFIG_MICRO_ROS_ESP_NETIF_WLAN || CONFIG_MICRO_ROS_ESP_NETIF_ENET
#ifdef CONFIG_MICRO_ROS_AGENT_DISCOVER
    // Use auto discovery.
    rcl_ret_t ret = RCL_RET_OK;
    do {
        ret = rmw_uros_discover_agent(rmw_options);
        if (ret != RCL_RET_OK) {
            printf("Failed to discover agent: %d\n", (int)ret);
            usleep(1000);
        }
    } while (ret != RCL_RET_OK);
    printf("Agent discovered\n");
#else
	// Use Static Agent IP and port.
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    printf("Agent IP and port set\n");
#endif
#endif
#endif
    // create init_options
	while (1) {
		rcl_ret_t ret = (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
		if (ret == RCL_RET_OK) {
			printf("support initialized\n");
			break;
		} else {
			printf("Failed to initialize support: %d : retrying...\n", (int)ret);
		}
	}

	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "m5StickCP2_node", "", &support));
	// Create executor.
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	unsigned int rcl_wait_timeout = 5000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    // Create publishers
    RCCHECK(rclc_publisher_init_default(
        &audio_data_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(audio_common_msgs, msg, AudioData),
        "~/audio_data"));
    // allocate memory for the message. 
    // Details: https://micro.ros.org/docs/tutorials/advanced/handling_type_memory/
    audio_data_msg.data.data = (uint8_t*)malloc(record_length * sizeof(int16_t));
    audio_data_msg.data.capacity = record_length*sizeof(int16_t);
    audio_data_msg.data.size = 0;

    RCCHECK(rclc_publisher_init_default(
        &audio_info_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(audio_common_msgs, msg, AudioInfo),
        "~/audio_info"));

    // Initialize the message.
    audio_info_msg.sample_rate = record_samplerate;
    audio_info_msg.channels = 1;
    audio_info_msg.sample_format.capacity = 4;
    audio_info_msg.sample_format.data = "PDM";
    audio_info_msg.sample_format.size = strlen("PDM");
    audio_info_msg.coding_format.capacity = 4;
    audio_info_msg.coding_format.data = "PDM";
    audio_info_msg.coding_format.size = strlen("PDM");

    // Signal that the publishers are initialized.
    xSemaphoreGive(init_semaphore);
    printf("micro_ros_task initialized and ready to publish audio data\n");

    int published_count = 0;
    while(1){
        StickCP2.update();
        
        // Try to receive audio data from the StreamBuffer
        size_t bytes_received = xStreamBufferReceive(
            audio_stream_buffer,
            audio_data_msg.data.data,
            record_length * sizeof(int16_t),
            pdMS_TO_TICKS(10) // 10ms timeout - non-blocking
        );
        
        if (bytes_received == record_length * sizeof(int16_t)) {
            // Publish the audio data
            audio_data_msg.data.size = bytes_received;
            auto ret = rcl_publish(&audio_data_publisher, &audio_data_msg, NULL);
            if (ret != RCL_RET_OK) {
                printf("Failed to publish audio data: %d\n", (int)ret);
                esp_restart();
            }
            
            // Publish audio info periodically
            if (published_count % 100 == 0) {
                RCCHECK(rcl_publish(&audio_info_publisher, &audio_info_msg, NULL));
                printf("Published %d audio chunks\n", published_count);
            }
            published_count++;
        }
        
		RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
	}

	// Free resources.
	RCCHECK(rcl_publisher_fini(&audio_data_publisher, &node));
	RCCHECK(rcl_publisher_fini(&audio_info_publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

extern "C" void app_main() {
    auto cfg = M5.config();
    StickCP2.begin(cfg);
    // Since the microphone and speaker cannot be used at the same time,
    // turn off the speaker here.
    StickCP2.Speaker.end();
    StickCP2.Mic.begin();
    
    // Initialize display
    StickCP2.Display.setRotation(1); // Landscape orientation
    StickCP2.Display.fillScreen(RED); // Fill screen with red during initialization
    StickCP2.Display.setTextSize(1);
    StickCP2.Display.setTextColor(WHITE);
    StickCP2.Display.setCursor(5, 5);
    StickCP2.Display.println("Initializing Audio RMS Display...");
    StickCP2.Display.display();

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif
    
    // Create the semaphore for initialization synchronization
    init_semaphore = xSemaphoreCreateBinary();
    
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
    
    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);

    // Create a task to record the audio on CORE 0.
    xTaskCreate(mic_record,
            "mic_record",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);

    // Start the scheduler.
    vTaskStartScheduler();
}

