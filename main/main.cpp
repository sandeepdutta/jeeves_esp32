#include <string.h>
#include <stdio.h>
#include <unistd.h>
#define RMW_UXRCE_TRANSPORT_CUSTOM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/channel_float32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"
#endif
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define MAX_LEDS 16
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 send_msg;
sensor_msgs__msg__ChannelFloat32 recv_msg;
static led_strip_handle_t led_strip;
static bool save_mode = false;

typedef struct {
	int red;
	int green;
	int blue;
} led_values_t;

static led_values_t led_values[MAX_LEDS];
static led_values_t led_values_backup[MAX_LEDS];
static led_values_t flash_color;
static int flash_count = 0;
static bool flash_on = false;

enum led_state {
	LED_STATE_NORMAL,
	LED_STATE_FLASHING
};
static int led_state = LED_STATE_NORMAL;

static void set_leds_to_color(int red, int green, int blue) {
	for (size_t i = 0; i < MAX_LEDS; i++) {
		led_strip_set_pixel(led_strip, i, red, green, blue);
	}
}

// heart beat timer callback
void hb_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
		if (rcl_publish(&publisher, &send_msg, NULL) == RCL_RET_OK) {
			printf("Sent: %d\n",  (int)  send_msg.data);
			send_msg.data++;
		} else {
			printf("Failed to send heartbeat: Agent mayhave disconnect restarting\n");
			esp_restart();
		}	
	}
}

// led timer callback
void led_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (led_state == LED_STATE_FLASHING) {
		flash_count--;
		// flashing done restore original colors
		if (flash_count == 0) {
			led_state = LED_STATE_NORMAL;
			flash_on = false;
			// restore original colors
			for (size_t i = 0; i < MAX_LEDS; i++) {
				led_strip_set_pixel(led_strip, i, led_values[i].red, led_values[i].green, led_values[i].blue);
			}
		} else if (flash_on) { // Flash on turn off
			flash_on = false;
			// flash off
			led_strip_clear(led_strip);
			return;
		} else { // Flash off turn on
			// flash on
			flash_on = true;
			for (size_t i = 0; i < MAX_LEDS; i++) {
				led_strip_set_pixel(led_strip, i, flash_color.red, flash_color.green, flash_color.blue);
			}
		}
		led_strip_refresh(led_strip);
	}
}

void subscription_callback(const void * msgin)
{
	const sensor_msgs__msg__ChannelFloat32 * msg = (const sensor_msgs__msg__ChannelFloat32 *)msgin;
	printf("Received: %s %d\n", msg->name.data, msg->values.size);
	// Set individual LED states	
	if (strcmp(msg->name.data, "led") == 0) {
		printf("LED state: %f\n", msg->values.data[0]);
		//value size must be divisible by 4
		if (msg->values.size % 4 != 0) {
			printf("Invalid LED state message\n");
			return;
		}
		// if first led_number is -1, set all LEDs to color
		if ((int)msg->values.data[0] == -1) {
			set_leds_to_color(msg->values.data[1], msg->values.data[2], msg->values.data[3]);
			led_state = LED_STATE_NORMAL;
			led_strip_refresh(led_strip);
			return;
		}
		// each LED has 4 elements led_number, red, green, blue
		// led_number is the index of the LED
		// red, green, blue are the color values
		auto num_leds = msg->values.size / 4;
		for (size_t i = 0; i < num_leds; i++) {
			int led_number = msg->values.data[i*4];
			int red = msg->values.data[i*4+1];
			int green = msg->values.data[i*4+2];
			int blue = msg->values.data[i*4+3];
			// save values to led_values
			led_values[led_number].red = red;	
			led_values[led_number].green = green;
			led_values[led_number].blue = blue;
			// set pixel
			led_strip_set_pixel(led_strip, led_number, red, green, blue);
		}
		led_state = LED_STATE_NORMAL;
		led_strip_refresh(led_strip);
	} else if (strcmp(msg->name.data, "led_fb") == 0 && !save_mode) { // Set front and back LEDs to color : back LED is +8 from front 
		//value size must be divisible by 4
		if (msg->values.size % 4 != 0) {
			printf("Invalid LED state message\n");
			return;
		}
		// each LED has 4 elements led_number, red, green, blue
		// led_number is the index of the LED
		// red, green, blue are the color values
		int num_leds = msg->values.size / 4;
		for (size_t i = 0; i < num_leds; i++) {
			int led_number = msg->values.data[i*4];
			int red = msg->values.data[i*4+1];
			int green = msg->values.data[i*4+2];
			int blue = msg->values.data[i*4+3];
			// save values to led_values
			led_values[led_number].red = led_values[led_number+8].red = red;	
			led_values[led_number].green = led_values[led_number+8].green = green;
			led_values[led_number].blue = led_values[led_number+8].blue = blue;
			// set pixel
			led_strip_set_pixel(led_strip, led_number, red, green, blue); // front
			led_strip_set_pixel(led_strip, led_number+8, red, green, blue); // back
		}
		led_state = LED_STATE_NORMAL;
		led_strip_refresh(led_strip);
	} else if (strcmp(msg->name.data, "led_clear") == 0 && !save_mode) {
		set_leds_to_color(0, 0, 0);
		led_strip_clear(led_strip);
		led_state = LED_STATE_NORMAL;
	} else if (strcmp(msg->name.data, "led_backup") == 0) {
		save_mode = true;
		for (size_t i = 0; i < MAX_LEDS; i++) {
			led_values_backup[i] = led_values[i];
		}
	} else if (strcmp(msg->name.data, "led_restore") == 0) {	
		save_mode = false;
		for (size_t i = 0; i < MAX_LEDS; i++) {
			led_values[i] = led_values_backup[i];
			led_strip_set_pixel(led_strip, i, led_values[i].red, led_values[i].green, led_values[i].blue);
		}
		led_state = LED_STATE_NORMAL;
		led_strip_refresh(led_strip);
	} else if (strcmp(msg->name.data, "led_flash") == 0 && !save_mode) {
		//value size must be divisible by 4
		if (msg->values.size % 4 != 0) {
			printf("Invalid LED flash message\n");
			return;
		}
		led_state = LED_STATE_FLASHING;
		// first value number of flashes
		flash_count = (int) msg->values.data[0];
		// second value is the color
		flash_color.red = (int) msg->values.data[1];
		flash_color.green = (int) msg->values.data[2];
		flash_color.blue = (int) msg->values.data[3];
		// rest handled by led timer callback
	}
}


void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// Create init_options.
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	printf("init_options initialized\n");
	bool connected = false;
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
#else
	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif  // RMW_UXRCE_TRANSPORT_CUSTOM

	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif
	// Setup support structure.
	while (1) {
		rcl_ret_t ret = (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
		if (ret == RCL_RET_OK) {
			printf("support initialized\n");
			connected = true;
			break;
		} else {
			printf("Failed to initialize support: %d : retrying...\n", (int)ret);
		}
	}

	// Allocate memory for incoming messages
	static micro_ros_utilities_memory_conf_t conf = {0};
	conf.max_string_capacity = 50;
	conf.max_ros2_type_sequence_capacity = MAX_LEDS*4;
	conf.max_basic_type_sequence_capacity = MAX_LEDS*4;

	bool success = micro_ros_utilities_create_message_memory(
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, ChannelFloat32),
		&recv_msg,
		conf
	);
	if (!success) {
		printf("Failed to allocate memory for incoming messages\n");
		vTaskDelete(NULL);
	}
	// Create node.
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "jeeves_esp32", "", &support));

	// Create publisher.
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"~/heartbeat"));

	// Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, ChannelFloat32),
		"~/led_state"));

	// Create timers.
	// heartbeat timer 
	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 5000; // send hearbeat every 5 seconds
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		hb_timer_callback));

	// timer to handle LED state
	rcl_timer_t led_timer = rcl_get_zero_initialized_timer();
	const unsigned int led_timer_timeout = 200; // update LED state every 100ms
	RCCHECK(rclc_timer_init_default(
		&led_timer,
		&support,
		RCL_MS_TO_NS(led_timer_timeout),
		led_timer_callback));
	// Create executor.
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
	unsigned int rcl_wait_timeout = 5000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	// Add timer and subscriber to executor.
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_timer(&executor, &led_timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));

	// Spin forever.
	send_msg.data = 0;
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(100000);
	}

	// Free resources.
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

// this is the main function called by the esp-idf framework
extern "C" void app_main(void)
{
	// Setup network interface or UART transport
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#else
	static size_t uart_port = UART_NUM_1;
	rmw_uros_set_custom_transport(
		true,
		(void *) &uart_port,
		esp32_serial_open,
		esp32_serial_close,
		esp32_serial_write,
		esp32_serial_read
	);
#endif
	// Initialize LED strip
	led_strip_config_t strip_config = {
		.strip_gpio_num = 48,
		.max_leds = MAX_LEDS,
	};
	led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        //.flags.with_dma = true,
    };
	spi_config.flags.with_dma = true;
	led_strip_new_spi_device(&strip_config, &spi_config, &led_strip);
	led_strip_clear(led_strip);
	// Initialize led_values to 0
	set_leds_to_color(0, 0, 0);
	led_strip_refresh(led_strip);
    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}