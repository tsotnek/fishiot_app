/*
* FishIoT Main Application
* MCU:nRF9160
*
* Author: Tsotne Karchava
* Created: 10.10.2023
*/


#include <stdio.h>
#include <string.h>
// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/devicetree.h>
// #include <zephyr/drivers/gpio.h>
//nrf cloud and agps
#include <net/nrf_cloud.h>
#include <net/nrf_cloud_agnss.h>
#include <zephyr/logging/log.h>
#include <date_time.h>
#include <net/nrf_cloud_rest.h>
#include <time.h>
#include <modem/modem_info.h>
#include <modem/modem_jwt.h>
//LTE
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <zephyr/net/socket.h>
#include <zephyr/drivers/i2c.h>
// #include <dk_buttons_and_leds.h>
//header file for the GNSS interface.
#include "gnss_connection.h"
// #include <nrf_modem_gnss.h>
//header file for MQTT
#include <zephyr/net/mqtt.h>
#include <zephyr/drivers/uart.h>
#include "mqtt_connection.h"
#include "rtc.h"
#include "rs485.h"
#include "leds.h"




LOG_MODULE_REGISTER(FishIoT, LOG_LEVEL_INF);
#define STACKSIZE 1024
#define THREAD0_PRIORITY 7
#define THREAD1_PRIORITY 2
#define THREAD2_PRIORITY 1



rtc_time_dec_t rtc_time;
rtc_time_bcd_t time_bcd;


uint8_t RS485_SN[14];


//MQTT
/* The mqtt client struct */
static struct mqtt_client client;
/* File descriptor */
static struct pollfd fds;




/*Helper variables to find the TTFF */
static bool first_fix = false;


//semaphores
static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(time_sem, 0, 1);
static K_SEM_DEFINE(mqtt_pub_sem, 0, 1);
static K_SEM_DEFINE(mqtt_pub_thread_start, 0, 1);
static K_SEM_DEFINE(gnss_start_sem, 0, 1);


static K_SEM_DEFINE(time_read_sem, 0, 1);
static K_SEM_DEFINE(rtc_write_fix_sem, 0, 1);
static K_SEM_DEFINE(rtc_esyn_sem, 0, 1);

extern const struct device *uart;



//Function prototypes
static int modem_configure(void);
void mqtt_thread(void);
void rtc_thread(void);
void rtc_datetime_button(void);

//handlers
void gnss_event_handler(int event);
static void lte_handler(const struct lte_lc_evt *const evt);
K_THREAD_DEFINE(mqtt_pub_id, STACKSIZE, mqtt_thread, NULL, NULL, NULL,
		THREAD0_PRIORITY, 0, 0);
K_THREAD_DEFINE(rtcthread, STACKSIZE, rtc_thread, NULL, NULL, NULL,
		THREAD1_PRIORITY, 0, 0);
K_THREAD_DEFINE(rtctimebutton, STACKSIZE, rtc_datetime_button, NULL, NULL, NULL,
		THREAD2_PRIORITY, 0, 0);

static void date_time_evt_handler(const struct date_time_evt *evt)
{
	k_sem_give(&time_sem);
}


static void lte_handler(const struct lte_lc_evt *const evt)
{
     switch (evt->type) {
     case LTE_LC_EVT_NW_REG_STATUS:
        if ((evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
            (evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING)) {
            break;
        }
		LOG_INF("Network registration status: %s",
				evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ?
				"Connected - home network" : "Connected - roaming");
		k_sem_give(&lte_connected);
        break;
	case LTE_LC_EVT_RRC_UPDATE:
		LOG_INF("RRC mode: %s", evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ?
				"Connected" : "Idle");
		if(evt->rrc_mode == LTE_LC_RRC_MODE_IDLE)
			k_sem_give(&gnss_start_sem);
		break;
	case LTE_LC_EVT_PSM_UPDATE:
		LOG_INF("PSM parameter update: TAU: %d, Active time: %d",
			evt->psm_cfg.tau, evt->psm_cfg.active_time);
		if (evt->psm_cfg.active_time == -1){
			LOG_ERR("Network rejected PSM parameters. Failed to enable PSM");
		}	
		break;
	case LTE_LC_EVT_MODEM_SLEEP_EXIT:
		LOG_INF("Modem exited from sleep.");
		break;
	case LTE_LC_EVT_MODEM_SLEEP_ENTER:
		LOG_INF("Modem entered sleep.");
		break;
     default:
             break;
     }
}

void gnss_event_handler(int event)
{
	int err;
	switch (event) {
	/*On a PVT event, confirm if PVT data is a valid fix */
	case NRF_MODEM_GNSS_EVT_PVT:
		LOG_INF("Searching...");
		/*Print satellite information */
		int num_satellites = 0;
		for (int i = 0; i < 12 ; i++) {
			if (pvt_data.sv[i].signal != 0) {
				LOG_INF("sv: %d, cn0: %d", pvt_data.sv[i].sv, pvt_data.sv[i].cn0);
				num_satellites++;
			}
		}
		LOG_INF("Number of current satellites: %d", num_satellites);
		err = nrf_modem_gnss_read(&pvt_data, sizeof(pvt_data), NRF_MODEM_GNSS_DATA_PVT);
		if (err) {
			LOG_ERR("nrf_modem_gnss_read failed, err %d", err);
			return;
		}
		if (pvt_data.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {
			LED_ON(BOARD_LED2);
			print_fix_data(&pvt_data);
			// rtc_sync_nav_second();

			/* Print the time to first fix */
			if (!first_fix) {
				LOG_INF("Time to first fix: %2.1lld s", (k_uptime_get() - gnss_start_time)/1000);
				first_fix = true;
				k_sem_give(&rtc_write_fix_sem);
			}
			else{
				k_sem_give(&rtc_esyn_sem);
			}
			
			return;
		}
		break;
	/* Log when the GNSS sleeps and wakes up */
	case NRF_MODEM_GNSS_EVT_PERIODIC_WAKEUP:
		LOG_INF("GNSS has woken up");
		break;
	case NRF_MODEM_GNSS_EVT_SLEEP_AFTER_FIX:
		
		LOG_INF("GNSS enter sleep after fix");

		k_sem_give(&mqtt_pub_thread_start);
		k_sem_give(&mqtt_pub_sem);
		
		break;
	default:
		break;
	}
}
static char str[150];
//helper function
void data_formatter(struct nrf_modem_gnss_pvt_data_frame *pvt_data){
	memset(str, 0, sizeof(str));
	strcat(str, "Latitude: ");
	char temporary[20];
	strcat(str, "\r\nLongitude: ");
	sprintf(temporary, "%.06f", pvt_data->longitude);
	strcat(str, temporary);
	
	sprintf(temporary, "%.06f", pvt_data->latitude);	
	strcat(str, temporary);


	strcat(str, "\r\nAltitude: ");
	sprintf(temporary, "%.01f", pvt_data->altitude);
	strcat(str, temporary);
}


//THREAD
void mqtt_thread(void){
	
	k_sem_take(&mqtt_pub_thread_start, K_FOREVER);
	
	int err;
	err = client_init(&client);
	if (err) {
		LOG_ERR("Failed to initialize MQTT client: %d", err);
	}
	
	for(;;){
	k_sem_take(&mqtt_pub_sem, K_FOREVER);
	printk("testing mqtt\n\n");
do_connect:

	err = mqtt_connect(&client);
	if (err) {
		LOG_ERR("Error in mqtt_connect: %d", err);
		goto do_connect;
	}

	err = fds_init(&client,&fds);
	if (err) {
		LOG_ERR("Error in fds_init: %d", err);
	}

	err = poll(&fds, 1, mqtt_keepalive_time_left(&client));
	if (err < 0) {
		LOG_ERR("Error in poll(): %d", errno);
	}
		
		
	err = mqtt_live(&client);
	if ((err != 0) && (err != -EAGAIN)) {
		LOG_ERR("Error in mqtt_live: %d", err);
	}

	if ((fds.revents & POLLIN) == POLLIN) {
		err = mqtt_input(&client);
		if (err != 0) {
			LOG_ERR("Error in mqtt_input: %d", err);
		}
	}

	if ((fds.revents & POLLERR) == POLLERR) {
		LOG_ERR("POLLERR");
	}

	if ((fds.revents & POLLNVAL) == POLLNVAL) {
		LOG_ERR("POLLNVAL");
	}
	data_formatter(&pvt_data);
	err = data_publish(&client, MQTT_QOS_1_AT_LEAST_ONCE,
		str, strlen(str));
	if (err) {
		LOG_INF("Failed to send message, %d", err);
	}

	LOG_INF("Disconnecting MQTT client");

	err = mqtt_disconnect(&client);
	if (err) {
		LOG_ERR("Could not disconnect MQTT client: %d", err);
	}
	}
}


//thread
void rtc_thread(void){


	k_sem_take(&rtc_write_fix_sem, K_FOREVER);
	LOG_INF("Writing NAV time and data");
	rtc_write_fix_data_first(&pvt_data);
	while(1){
		k_sem_take(&rtc_esyn_sem, K_FOREVER);
		rtc_sync_nav_second();
	}

}


void rtc_datetime_button(void){
	char* weekdayarr[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
	int err;
	for(;;){
		k_sem_take(&time_read_sem,K_FOREVER);
		volatile int16_t temperature = rtc_read_temp();
		printk("Temperature of RTC is %d\n", temperature);
		// err = rtc_read_time_data();
		// if (err != 0){
		// 	LOG_ERR("Failed to read date from RTC\n");
		// }
		// LOG_INF("Today is %s, Time: %d:%d:%d\n", weekdayarr[rtc_time.weekday], rtc_time.hour, \
		// 									rtc_time.minute, rtc_time.seconds);

	}
}



int main(void)
{
	int err;
	printk("Starting Application IoF...\n");
	
	if(led_button_init() != 0){
		LOG_ERR("Failed to initialize LED and Buttons");
		return 1;
	}
	
	if(rtc_init()!=0){
		LOG_ERR("Failed to initialize RTC");
		return 1;
	};

   
	// //read temperature
	volatile int16_t temperature = rtc_read_temp();
	printk("Temperature of RTC is %d\n", temperature);
	// //test for time

	// gpio_pin_toggle_dt(&led3);
	// gpio_pin_toggle_dt(&led2);
	// gpio_pin_toggle_dt(&led0);
	// gpio_pin_toggle_dt(&led1);

	// err = rs485_init();
	// if(err)
	// 	return 1;

	// printk("Enabling modem...\n");
	// err = modem_configure();
	// if (err) {
	// 	LOG_ERR("Failed to configure the modem");
	// 	return 1;
	// }
	
	// err = agps_receive_process_data();
	// if(err){
	// 	LOG_ERR("Failed to receive and process AGPS data");
	// 	return 1;
	// }

	// k_sem_take(&gnss_start_sem, K_FOREVER);
	// if (gnss_init_and_start() != 0) {
	// 	LOG_ERR("Failed to initialize and start GNSS");
	// 	return 1;
	// }	

	return 0;
}


static int modem_configure(void)
{
	int err;

	LOG_INF("Initializing modem library");

	err = nrf_modem_lib_init();
	if (err) {
		LOG_ERR("Failed to initialize the modem library, error: %d", err);
		return err;
	}
	
	/*  Store the certificate in the modem while the modem is in offline mode  */
	// err = certificate_provision();
	// if (err != 0) {
	// 	LOG_ERR("Failed to provision certificates");
	// 	return 1;
	// }

	err = lte_lc_init();
	if (err) {
		LOG_ERR("Failed to initialize LTE link controller");
		return 1;
	}

	//enable PSM
	err = lte_lc_psm_req(true);
	if (err) {
		LOG_ERR("lte_lc_psm_req, error: %d", err);
	}

	LOG_INF("Connecting to LTE network");

	lte_lc_register_handler(lte_handler);

	if (lte_lc_connect() != 0) {
		LOG_ERR("Failed to connect to LTE network");
		return 1;
	}

	LOG_INF("Connected to LTE network");
	err = date_time_update_async(date_time_evt_handler);
	if(err){
		LOG_ERR("Date time update handler failed");
	}	

	LED_ON(BOARD_LED0);

	LOG_INF("Waiting for current time");	
	/* Wait for an event from the Date Time library. */
	k_sem_take(&time_sem, K_MINUTES(10));
		
	if (!date_time_is_valid()) {
		LOG_WRN("Failed to get current time, continuing anyway");
	}
	int64_t datetime;
	datetime = k_uptime_get();
	err = date_time_uptime_to_unix_time_ms(&datetime);
	if(err){
		LOG_ERR("Failed to get Date time");
	}	
	LOG_INF("Received current time successfully!");
	return 0;
}


