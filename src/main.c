/*
* FishIoT Main Application
* MCU:nRF9160
*
* Author: Tsotne Karchava
* Created: 10.10.2023
*/

#include <stdio.h>
#include <string.h>
//nrf cloud and agps
#include <net/nrf_cloud.h>
#include <net/nrf_cloud_agnss.h>
#include <net/nrf_cloud_rest.h>
#include <zephyr/logging/log.h>
#include <date_time.h>
#include <time.h>
#include <modem/modem_info.h>
#include <modem/modem_jwt.h>
//LTE
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <zephyr/net/socket.h>
//header file for the GNSS interface.
#include "gnss_connection.h"
//header file for MQTT
#include "mqtt_connection.h"
#include "rtc.h"
#include "rs485.h"
#include "leds.h"




LOG_MODULE_REGISTER(FishIoT, LOG_LEVEL_INF);
#define STACKSIZE 1024
#define MQTT_THREAD_PRIORITY 7
#define RTC_THREAD_PRIORITY 5
#define THREAD2_PRIORITY 10
#define RS485_THREAD_PRIORITY 3

typedef enum{
	TBR_status_or_tag,
	buoy_status,
	GPS_cycle
}IoF_header_flags;

typedef struct{
	uint16_t TBRserial_and_headerflag; //TBR serial[0:13] and header flag[14:15]
	uint32_t reftimestamp; //Reference timestamp (UTC)
}IoF_header;

K_MSGQ_DEFINE(IoFHEADER_MSG, sizeof(IoF_header), 16, sizeof(uint32_t));

typedef struct{
	uint8_t secsince_timestamp;
	uint8_t code_type;
	uint16_t temperature;
	uint8_t noise_ave;
	uint8_t noise_peak;
	uint8_t freq; 
	uint8_t upper_timing_err;
}IoF_TBR_status;

K_MSGQ_DEFINE(IoFTBR_Status_MSG, sizeof(IoF_TBR_status), 16, sizeof(uint32_t));

typedef struct{
	uint8_t secsince_timestamp;
	uint8_t code_type;
	uint8_t tag_id; //allprotocols
	uint8_t tag_id_payload; //Tag ID (protocol: R04K, R64K, R01M, S64K, HS256, DS256)
 							//or Tag payload (protocol: S256)
 							//or Not used (protocol: R256)
	uint8_t tag_id_payload_r01m; //Tag ID (protocol: R01M)
							//or Tag payload (protocol: S64K, HS256, DS256)
							//or Not used (protocol: R256, R04K, R64K, S256)
	uint16_t SNR_milliseconds  //SNR[0:3], rest is milliseconds
}IoF_TBR_tag;

K_MSGQ_DEFINE(IoFTBR_TAG_MSG, sizeof(IoF_TBR_tag), 16, sizeof(uint32_t));


//MQTT
/* The mqtt client struct */
static struct mqtt_client client;
/* File descriptor */
static struct pollfd fds;

//semaphores
static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(time_sem, 0, 1);
static K_SEM_DEFINE(mqtt_pub_sem, 0, 1);
static K_SEM_DEFINE(mqtt_pub_thread_start, 0, 1);


static K_SEM_DEFINE(gnss_start_sem, 0, 1);


static K_SEM_DEFINE(time_read_sem, 0, 1);
static K_SEM_DEFINE(rtc_write_fix_sem, 0, 1);
static K_SEM_DEFINE(rtc_esyn_sem, 0, 1);


static K_SEM_DEFINE(mqtt_pub_done_sem, 0, 1);
extern const struct device *uart;



//Function prototypes
static int modem_configure(void);

//threads
void mqtt_thread(void);
void rtc_thread(void);
void rs485_thread(void);
void rtc_datetime_button(void);

//handlers
static void lte_handler(const struct lte_lc_evt *const evt);


//thread definitions
K_THREAD_DEFINE(mqtt_pub_id, STACKSIZE, mqtt_thread, NULL, NULL, NULL,
		MQTT_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(rtcthread, STACKSIZE, rtc_thread, NULL, NULL, NULL,
		RTC_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(rtctimebutton, STACKSIZE, rtc_datetime_button, NULL, NULL, NULL,
		THREAD2_PRIORITY, 0, 0);
K_THREAD_DEFINE(rs485thread, STACKSIZE, rs485_thread, NULL, NULL, NULL,
		RS485_THREAD_PRIORITY,0, 0);


static void date_time_evt_handler(const struct date_time_evt *evt)
{
	k_sem_give(&time_sem);
}

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	// BIT(button1.pin)
	LOG_INF("Button pressed\n");
	k_sem_give(&time_read_sem);
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

		// k_sem_give(&mqtt_pub_thread_start);
		// k_sem_give(&mqtt_pub_sem);
		
		break;
	default:
		break;
	}
}


//THREAD
//TODO: convert received strings into structs

void rs485_thread(void){
	//wait until the message was received on RS485
	char *word = "TBR Sensor";
	for(;;){
		k_sem_take(&uart_rec_sem, K_FOREVER);
		IoF_header header;
		uint8_t strport[10];
		header.TBRserial_and_headerflag = TBSN | (TBR_status_or_tag << 14);
		strncpy(strport, rx_buf[9],10);
		header.reftimestamp = atoi(strport);
		k_msgq_put(&IoFHEADER_MSG,&header,K_FOREVER);
		if(strstr(rx_buf,word)){
			//log message
			
			LOG_INF("Received LOG Message on TBR: %s", rx_buf);
		}
		else{
			//tag detection
			LOG_INF("Detected FISH on TBR: %s", rx_buf);
		}

		k_sem_give(&mqtt_pub_thread_start);
		k_sem_give(&mqtt_pub_sem);
		k_sem_take(&mqtt_pub_done_sem, K_FOREVER);
		memset((void *) rx_buf, 0, sizeof(rx_buf)/sizeof(char));
	}
}

uint8_t mqttmessageformat[15] = {0};
//helper function
void formatmessage(void){
	IoF_header iofheader;
	k_msgq_get(&IoFHEADER_MSG, &iofheader, K_FOREVER);

	mqttmessageformat[0] = (uint8_t)iofheader.TBRserial_and_headerflag;
	mqttmessageformat[1] = (uint8_t)iofheader.TBRserial_and_headerflag>>8;

	mqttmessageformat[2] = (uint8_t)iofheader.reftimestamp;
	mqttmessageformat[3] = (uint8_t)iofheader.reftimestamp>>8;
	mqttmessageformat[4] = (uint8_t)iofheader.reftimestamp>>16;
	mqttmessageformat[5] = (uint8_t)iofheader.reftimestamp>>24;


	switch(iofheader.TBRserial_and_headerflag & 0xC0){
		case TBR_status_or_tag:
			//check which message was received
			IoF_TBR_status status;
			k_msgq_peek(&IoFTBR_Status_MSG, &status);
			if(status.temperature != 0)
			{
				k_msgq_get(&IoFTBR_Status_MSG, &status, K_FOREVER);
				mqttmessageformat[6] = status.secsince_timestamp;
				mqttmessageformat[7] = status.code_type;
				mqttmessageformat[8] = (uint8_t)status.temperature;
				mqttmessageformat[9] = (uint8_t)status.temperature>>8;
				mqttmessageformat[10] = status.noise_ave;
				mqttmessageformat[11] = status.noise_peak;
				mqttmessageformat[12] = status.freq;
				mqttmessageformat[13] = status.upper_timing_err;
			}
			else{
				IoF_TBR_tag tag;
				k_msgq_get(&IoFTBR_TAG_MSG, &tag, K_FOREVER);
				mqttmessageformat[6] = tag.secsince_timestamp;
				mqttmessageformat[7] = tag.code_type;
				mqttmessageformat[8] = tag.tag_id;
				switch(tag.code_type){ //depeending on code type adjust payload
					case 0: //protocol s256
						mqttmessageformat[9] = tag.tag_id_payload;
						mqttmessageformat[10] = (uint8_t)tag.SNR_milliseconds;
						mqttmessageformat[11] = (uint8_t)tag.SNR_milliseconds>>8;						
						break;

					default:
						break;
				}


			}
			break;

		
		default: 
			break;
	}
}

void mqtt_thread(void){
	
	k_sem_take(&mqtt_pub_thread_start, K_FOREVER);
	
	int err;
	err = client_init(&client);
	if (err) {
		LOG_ERR("Failed to initialize MQTT client: %d", err);
	}
	
	for(;;){
	k_sem_take(&mqtt_pub_sem, K_FOREVER);
	formatmessage();
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
	// data_formatter(&pvt_data);
	err = data_publish(&client, MQTT_QOS_1_AT_LEAST_ONCE,
		mqttmessageformat, strlen(mqttmessageformat));
	if (err) {
		LOG_INF("Failed to send message, %d", err);
	}
	memset((void *) mqttmessageformat, 0, sizeof(mqttmessageformat)/sizeof(uint8_t));
	LOG_INF("Disconnecting MQTT client");

	err = mqtt_disconnect(&client);
	if (err) {
		LOG_ERR("Could not disconnect MQTT client: %d", err);
	}
	k_sem_give(&mqtt_pub_done_sem);

	}
}


//thread
void rtc_thread(void){

	k_sem_take(&rtc_write_fix_sem, K_FOREVER);
	LOG_INF("Writing NAV time and data");
	//write time from GNSS to RTC
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
		err = rtc_read_time_data();
		if (err != 0){
			LOG_ERR("Failed to read date from RTC\n");
		}
		LOG_INF("Today is %s, Time: %d:%d:%d\n", weekdayarr[rtc_time.weekday], rtc_time.hour, \
											rtc_time.minute, rtc_time.seconds);

	}
}



int main(void)
{
	int err;
	printk("Starting Application IoF...\n");
	k_sched_lock();
	
	if(led_button_init() != 0){
		LOG_ERR("Failed to initialize LED and Buttons");
		return 1;
	}
	
	if(rtc_init()!=0){
		LOG_ERR("Failed to initialize RTC");
		return 1;
	};

	//read temperature
	volatile int16_t temperature = rtc_read_temp();
	printk("Temperature of RTC is %d\n", temperature);

	char* weekdayarr[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

	err = rtc_read_time_data();
	if (err != 0){
		LOG_ERR("Failed to read date from RTC\n");
	}
	LOG_INF("Today is %s, Time: %d:%d:%d\n", weekdayarr[rtc_time.weekday], rtc_time.hour, \
											rtc_time.minute, rtc_time.seconds);

	err = rs485_init();
	if(err)
		return 1;

	k_sched_unlock();
	printk("Enabling modem...\n");
	err = modem_configure();
	if (err) {
		LOG_ERR("Failed to configure the modem");
		return 1;
	}
	
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
	k_sem_take(&time_sem, K_MINUTES(1));
		
	if (!date_time_is_valid()) {
		LOG_WRN("Failed to get current time, continuing anyway");
		//date time couldn't be receieved under 1 minute or it is wrong.
	}
	else{
	/*TIME*/
	//if Time is valid then update TBR live enoch time.
	uint64_t unix_time_ms;
	/*Read current time and put in container */
	err = date_time_now(&unix_time_ms);
	//write enoch time to TBRLive
	err = rs485_updatetime(unix_time_ms);	
	}

	LOG_INF("Received current time successfully!");
	return 0;
}


