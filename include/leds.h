#ifndef _LEDS_H_
#define _LEDS_H_
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <stdint.h>

//switches
#define SW0_NODE	DT_ALIAS(sw0)
#define SW1_NODE	DT_ALIAS(sw1)





#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define LED3_NODE DT_ALIAS(led3)



typedef enum{
    BOARD_LED0,
    BOARD_LED1,
    BOARD_LED2,
    BOARD_LED3
}boardleds;


//total 16 combinations
typedef enum{
    RTC_INIT_ERROR,
    ADC_INIT_ERROR,
    LTE_NETWORK_CONNECT_ERROR,
    RS485_INIT_ERROR,
    AGPS_RECEIVE_ERROR,
    GNSS_INIT_ERROR,
    MQTT_CONNECT_ERROR,
    MQTT_LIVE_ERROR,
    MQTT_INPUT_ERROR,
    MQTT_PUBLISH_ERROR,
    
} led_errors;
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
uint8_t led_button_init(void);
void LED_ON(uint8_t LED);
void LED_OFF(uint8_t LED);
void LED_ERROR_CODE(uint8_t LED_ERROR);
void LED_TOGGLE(uint8_t LED);
#endif