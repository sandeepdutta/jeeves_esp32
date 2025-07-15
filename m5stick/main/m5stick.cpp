#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <math.h>
#include <complex>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_netif.h"
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

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

extern "C" void app_main();

static constexpr const size_t record_length     = 256;
static constexpr const size_t record_samplerate = 44100;
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
        printf("Using %d samples of %d\n", (int)w, record_length);
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
        printf("Max: %d, Min: %d, RMS: %f\n", (int)max_y, (int)min_y, rms);
        StickCP2.Display.display();
        StickCP2.Display.fillCircle(70, 15, 8, RED);
        StickCP2.Display.drawString("REC", 120, 3);
    } else {
        printf("Failed to record\n");
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

	// Static Agent IP and port can be used instead of autodisvery.
    //RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	RCCHECK(rmw_uros_discover_agent(rmw_options));
    printf("Agent discovered\n");
#endif

    // create init_options
	while (1) {
		rcl_ret_t ret = (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
		if (ret == RCL_RET_OK) {
			printf("support initialized\n");
			//connected = true;
			break;
		} else {
			printf("Failed to initialize support: %d : retrying...\n", (int)ret);
		}
	}

	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "m5StickCP2_node", "", &support));

    while(1){
		sleep(100);
	}

	// free resources
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

extern "C" void app_main() {
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}

