#include "leds.h"
#include <zephyr/logging/log.h>


LOG_MODULE_DECLARE(FishIoT);



static struct gpio_callback button_cb_data;
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);



void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	// BIT(button1.pin)
	LOG_INF("Button pressed\n");
	// k_sem_give(&time_read_sem);
}

void LED_ERROR_CODE(uint8_t LED_ERROR)
{
	uint8_t led0_val = 0, led1_val = 0 , led2_val = 0,led3_val = 0;
	switch(LED_ERROR){
		//dec value: 1
		case RTC_INIT_ERROR:
			led0_val = 1;
			break;
		//dev calue: 2
		case ADC_INIT_ERROR:
			led1_val = 1;
			break;
		//dev calue: 3
		case LTE_NETWORK_CONNECT_ERROR:
			led0_val = 1;
			led1_val = 1;
			break;
		//dev calue: 4
		case RS485_INIT_ERROR:
			led2_val = 1;
			break;
		//dev calue: 5
		case AGPS_RECEIVE_ERROR:
			led0_val = 1;
			led2_val = 1;
			break;
		//dev calue: 6
		case GNSS_INIT_ERROR:
			led1_val = 1;
			led2_val = 1;
			break;
		//dev calue: 7
		case MQTT_CONNECT_ERROR:
			led0_val = 1;
			led1_val = 1;
			led2_val = 1;
			break;
		//dev calue: 8
		case MQTT_LIVE_ERROR:
			led3_val = 1;
			break;
		//dev calue: 9
		case MQTT_INPUT_ERROR:
			led0_val = 1;
			led3_val = 1;
			break;
		//dev calue: 10
		case MQTT_PUBLISH_ERROR:
			led1_val = 1;
			led3_val = 1;
			break;
		default:
			break;
	}
	gpio_pin_set_dt(&led0,led0_val);
	gpio_pin_set_dt(&led1,led1_val);
	gpio_pin_set_dt(&led2,led2_val);
	gpio_pin_set_dt(&led3,led3_val);

}


uint8_t led_button_init(void){

	uint8_t ret;

	if (!gpio_is_ready_dt(&led0)) {
		return -1;
	}

	if (!gpio_is_ready_dt(&led1)) {
		return -1;
	}

	if (!gpio_is_ready_dt(&led2)) {
		return -1;
	}
	if (!gpio_is_ready_dt(&led3)) {
		return -1;
	}

	if (!gpio_is_ready_dt(&button1)) {
		return -1;
	}

	if (!gpio_is_ready_dt(&button2)) {
		return -1;
	}


	ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return -1;
	}

	ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return -1;
	}

	ret = gpio_pin_configure_dt(&led2, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return -1;
	}
	ret = gpio_pin_configure_dt(&led3, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return -1;
	}

	/* STEP 5 - Configure the pin connected to the button to be an input pin and set its hardware specifications */
	ret = gpio_pin_configure_dt(&button1, GPIO_INPUT);
	if (ret < 0) {
		return -1;
	}
	ret = gpio_pin_configure_dt(&button2, GPIO_INPUT);
	if (ret < 0) {
		return -1;
	}

	ret = gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_TO_ACTIVE);
	ret = gpio_pin_interrupt_configure_dt(&button2, GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&button_cb_data, button_pressed, BIT(button1.pin)); 	
    gpio_init_callback(&button_cb_data, button_pressed, BIT(button2.pin)); 	

	gpio_add_callback(button1.port, &button_cb_data);
	gpio_add_callback(button2.port, &button_cb_data);

	return 0;
}


void LED_ON(uint8_t LED){
    switch(LED){
        case BOARD_LED0:
	        gpio_pin_set_dt(&led0,false);
            break;
        case BOARD_LED1:
	        gpio_pin_set_dt(&led1,false);
            break;
        case BOARD_LED2:
	        gpio_pin_set_dt(&led2,false);
            break;
        case BOARD_LED3:
            gpio_pin_set_dt(&led3,false);
            break;  
        default:
            break;
    }
}

void LED_OFF(uint8_t LED){
    switch(LED){
        case BOARD_LED0:
	        gpio_pin_set_dt(&led0,true);
            break;
        case BOARD_LED1:
	        gpio_pin_set_dt(&led1,true);
            break;
        case BOARD_LED2:
	        gpio_pin_set_dt(&led2,true);
            break;
        case BOARD_LED3:
            gpio_pin_set_dt(&led3,true);
            break;  
        default:
            break;
    }
}


void LED_TOGGLE(uint8_t LED){
    switch(LED){
        case BOARD_LED0:
	        gpio_pin_toggle_dt(&led0);
            break;
        case BOARD_LED1:
	        gpio_pin_toggle_dt(&led1);
            break;
        case BOARD_LED2:
	        gpio_pin_toggle_dt(&led2);
            break;
        case BOARD_LED3:
            gpio_pin_toggle_dt(&led3);
            break;  
        default:
            break;
    }
}
