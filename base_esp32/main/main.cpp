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
#include <jeeves_msgs/srv/led_set.h>
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
std_msgs__msg__Int32 send_msg;
static led_strip_handle_t led_strip;
static bool save_mode = false;

typedef struct {
	int red;
	int green;
	int blue;
} led_values_t;

static led_values_t led_values[MAX_LEDS];
static led_values_t led_values_backup[MAX_LEDS];

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

static void flash_leds(int times, int delay_ticks, int min_led, int max_led, 	int red, int green, int blue) {
	// Backup current LED state
	for (int led = min_led; led <= max_led; led++) {
		led_values_backup[led] = led_values[led];
	}
	for (int i = 0; i < times; i++) {
		// Turn on LEDs
		for (int led = min_led; led <= max_led; led++) {
			led_strip_set_pixel(led_strip, led, red, green, blue);
		}	
		led_strip_refresh(led_strip);
		vTaskDelay(delay_ticks);
		// Turn off LEDs
		for (int led = min_led; led <= max_led; led++) {
			led_strip_set_pixel(led_strip, led, 0, 0, 0);
		}
		led_strip_refresh(led_strip);
		vTaskDelay(delay_ticks);
	}
	// Restore original LED state
	for (int led = min_led; led <= max_led; led++) {
		led_values[led] = led_values_backup[led];
		led_strip_set_pixel(led_strip, led, led_values[led].red, led_values[led].green, led_values[led].blue);
	}
	led_strip_refresh(led_strip);
}

// service callback
void service_callback(const void * req, void * res){
	jeeves_msgs__srv__LedSet_Response * res_out = (jeeves_msgs__srv__LedSet_Response *)res;
	jeeves_msgs__srv__LedSet_Request * req_in = (jeeves_msgs__srv__LedSet_Request *)req;
	printf("Received LED set request %d %ld %ld %ld %ld\n", req_in->led_cmd, req_in->led_number, req_in->red, req_in->green, req_in->blue);
	int flash_delay = req_in->flash_speed / portTICK_PERIOD_MS;
	res_out->success = true;
	switch (req_in->led_cmd) {
		case jeeves_msgs__srv__LedSet_Request__LED_SET_ALL:
			set_leds_to_color(req_in->red, req_in->green, req_in->blue);
			led_strip_refresh(led_strip);
			break;
		case jeeves_msgs__srv__LedSet_Request__LED_SINGLE	:
			led_values[req_in->led_number].red = req_in->red;
			led_values[req_in->led_number].green = req_in->green;
			led_values[req_in->led_number].blue = req_in->blue;
			led_strip_set_pixel(led_strip, req_in->led_number, req_in->red, req_in->green, req_in->blue);
			led_strip_refresh(led_strip);
			break;
		case jeeves_msgs__srv__LedSet_Request__LED_FB:
			led_values[req_in->led_number].red = led_values[req_in->led_number+8].red = req_in->red;
			led_values[req_in->led_number].green = led_values[req_in->led_number+8].green = req_in->green;
			led_values[req_in->led_number].blue = led_values[req_in->led_number+8].blue = req_in->blue;
			led_strip_set_pixel(led_strip, req_in->led_number, req_in->red, req_in->green, req_in->blue);
			led_strip_set_pixel(led_strip, req_in->led_number+8, req_in->red, req_in->green, req_in->blue);
			led_strip_refresh(led_strip);
			break;
		case jeeves_msgs__srv__LedSet_Request__LED_FLASH:
			flash_leds(req_in->flash_times, flash_delay, 0, MAX_LEDS-1, req_in->red, req_in->green, req_in->blue);
			break;
		case jeeves_msgs__srv__LedSet_Request__LED_FLASH_F:
			flash_leds(req_in->flash_times, flash_delay, 0, (MAX_LEDS/2)-1, req_in->red, req_in->green, req_in->blue);
			break;
		case jeeves_msgs__srv__LedSet_Request__LED_FLASH_B:
			flash_leds(req_in->flash_times, flash_delay, MAX_LEDS/2, MAX_LEDS-1, req_in->red, req_in->green, req_in->blue);
			break;
		case jeeves_msgs__srv__LedSet_Request__LED_SAVE:
			for (int led = 0; led < MAX_LEDS; led++) {
				led_values_backup[led] = led_values[led];
			}
			save_mode = true;
			break;
		case jeeves_msgs__srv__LedSet_Request__LED_RESTORE:	
			for (int led = 0; led < MAX_LEDS; led++) {
				led_values[led] = led_values_backup[led];
				led_strip_set_pixel(led_strip, led, led_values[led].red, led_values[led].green, led_values[led].blue);
			}
			led_strip_refresh(led_strip);
			save_mode = false;
			break;	
		default:
			printf("Invalid LED command\n");
			res_out->success = false;
			break;
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
	/*
	bool success = micro_ros_utilities_create_message_memory(
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, ChannelFloat32),
		&recv_msg,
		conf
	);
	if (!success) {
		printf("Failed to allocate memory for incoming messages\n");
		vTaskDelete(NULL);
	}
	*/
	// Create node.
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "jeeves_esp32", "", &support));

	// Create publisher.
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"~/heartbeat"));

    // create service
    rcl_service_t service;
    RCCHECK(rclc_service_init_default(&service, 
			&node, 
			ROSIDL_GET_SRV_TYPE_SUPPORT(jeeves_msgs, srv, LedSet), 
			"~/led_set"));
	// Create timers.
	// heartbeat timer 
	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 5000; // send hearbeat every 5 seconds
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		hb_timer_callback));

	// Create service callback    
	jeeves_msgs__srv__LedSet_Response res;
    jeeves_msgs__srv__LedSet_Request req;
	// Create executor.
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	unsigned int rcl_wait_timeout = 5000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	// Add timer and Service to executor.
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, service_callback));

	// Spin forever.
	send_msg.data = 0;
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(1000);
	}

	// Free resources.
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
	// Initialize led_values to low RED
	set_leds_to_color(20, 0, 0);
	led_strip_refresh(led_strip);
    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}