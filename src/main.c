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
//header file for the GNSS interface.
#include "gnss_connection.h"
//header file for MQTT
// #include "mqtt_connection.h"
#include "rtc.h"
#include "rs485.h"
#include "leds.h"
#include "adc.h"
#include "mqtt_task.h"

LOG_MODULE_REGISTER(FishIoT, LOG_LEVEL_INF);
#define STACKSIZE 1024
#define MQTT_THREAD_PRIORITY 7
#define RTC_THREAD_PRIORITY 4
#define RS485_THREAD_PRIORITY 3
#define GNSS_THREAD_PRIORITY 5
#define THREAD2_PRIORITY 10


K_MSGQ_DEFINE(IoFHEADER_MSG, sizeof(IoF_header), 16, sizeof(uint32_t));


K_MSGQ_DEFINE(IoFTBR_Status_MSG, sizeof(IoF_TBR_status), 16, sizeof(uint32_t));


K_MSGQ_DEFINE(IoFTBR_TAG_MSG, sizeof(IoF_TBR_tag), 16, sizeof(uint32_t));

//semaphores
static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(time_sem, 0, 1);

static K_SEM_DEFINE(gnss_start_sem, 0, 1);


static K_SEM_DEFINE(time_read_sem, 0, 1);
static K_SEM_DEFINE(rtc_write_fix_sem, 0, 1);
static K_SEM_DEFINE(rtc_esyn_sem, 0, 1);


static K_SEM_DEFINE(gnss_thread_sem, 0 , 1);

extern const struct device *uart;




//Function prototypes
static int modem_configure(void);

//threads
void rtc_thread(void);
void rs485_thread(void);
void gnss_thread(void);
void rtc_datetime_button(void);

//handlers
static void lte_handler(const struct lte_lc_evt *const evt);


//thread definitions
K_THREAD_DEFINE(mqtt_pub_id, STACKSIZE, mqtt_thread, NULL, NULL, NULL,
		MQTT_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(rtcthread, STACKSIZE, rtc_thread, NULL, NULL, NULL,
		RTC_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(rs485thread, STACKSIZE, rs485_thread, NULL, NULL, NULL,
		RS485_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(gnssthread, STACKSIZE, gnss_thread, NULL, NULL, NULL,
		GNSS_THREAD_PRIORITY, 0, 0);

K_THREAD_DEFINE(rtctimebutton, STACKSIZE, rtc_datetime_button, NULL, NULL, NULL,
		THREAD2_PRIORITY, 0, 0);


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
			print_fix_data(&pvt_data);

			/* Print the time to first fix */
			if (!first_fix) {
				LOG_INF("Time to first fix: %2.1lld s", (k_uptime_get() - gnss_start_time)/1000);
				first_fix = true;
				k_sem_give(&rtc_write_fix_sem);
			}
			else{
				k_sem_give(&rtc_esyn_sem);
			}
			
			k_sem_give(&gnss_thread_sem);
			return;
		}
		break;
	/* Log when the GNSS sleeps and wakes up */
	case NRF_MODEM_GNSS_EVT_PERIODIC_WAKEUP:
		LOG_INF("GNSS has woken up");
		break;
	case NRF_MODEM_GNSS_EVT_SLEEP_AFTER_FIX:
		LOG_INF("GNSS enter sleep after fix");		
		break;
	default:
		break;
	}
}


//THREAD
void gnss_thread(void){
	for(;;){
		//wait until the GNSS fix occurs
		k_sem_take(&gnss_thread_sem, K_FOREVER);
		IoF_header header;
		header.TBRserial_and_headerflag = TBSN | (buoy_status << 14);
		struct tm ti={
			.tm_isdst = -1,
			.tm_sec = pvt_data.datetime.seconds,
			.tm_min = pvt_data.datetime.minute,
			.tm_hour = pvt_data.datetime.hour,
			.tm_mday = pvt_data.datetime.day,
			.tm_mon = pvt_data.datetime.month,
			.tm_mon = pvt_data.datetime.month,
			.tm_year = pvt_data.datetime.year
		};
	
		time_t t_of_day = mktime(&ti);
		header.reftimestamp = (uint32_t)t_of_day;

		
		IoF_bouy_status bouy;
		// bouy.batvolatge_airtemp_lon = ((uint16_t)(pvt_data.longitude&3)<<14) | (rtc_read_temp()<<7) | (uint16_t)(adc_read_voltage()/100);
		

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



//TODO: convert received strings into structs

void code_type_function(IoF_TBR_tag* tag, char * protocol, int frequency){
	uint8_t protocol_int = 0;
	if((strncmp(protocol,(char *)"R256", 4) == 0) || (strncmp(protocol, (char *)"r256",4) == 0)){
		protocol_int = R256;
	}
	else if((strncmp(protocol, (char *)"R04K", 4) == 0) || (strncmp(protocol, (char *)"r04k", 4) == 0 ) ||
			(strncmp(protocol, (char *)"R04k", 4) == 0) || (strncmp(protocol, (char *)"r04K", 4) == 0 )) {
		protocol_int = R04K;
	}
	else if((strncmp(protocol, (char *)"R64K", 4) == 0) || (strncmp(protocol, (char *)"r64k", 4) == 0) ||
			(strncmp(protocol, (char *)"R64k", 4) == 0) || (strncmp(protocol, (char *)"r64K", 4) == 0)) {
		protocol_int = R64K;
	}
	else if((strncmp(protocol,(char *)"S256", 4) == 0) || (strncmp(protocol, (char *)"s256", 4) == 0)) {
		protocol_int = S256;
	}
	else if((strncmp(protocol,(char *)"R01M", 4) == 0) || (strncmp(protocol, (char *)"r01m", 4)==0) ||
			(strncmp(protocol,(char *)"R01m", 4) == 0) || (strncmp(protocol, (char *)"r01M", 4)==0)) {
		protocol_int = R01M;
	}
	else if((strncmp(protocol,(char *)"S64K", 4) == 0) || (strncmp(protocol, (char *)"s64k", 4) == 0) ||
			(strncmp(protocol,(char *)"S64k", 4) == 0) || (strncmp(protocol, (char *)"s64K", 4) == 0)) {
		protocol_int = S64K;
	}
	else if((strncmp(protocol,(char *)"HS256", 4) == 0) || (strncmp(protocol, (char *)"hs256", 4) == 0) ||
			(strncmp(protocol,(char *)"Hs256", 4) == 0) || (strncmp(protocol, (char *)"hS256", 4) == 0)) {
		protocol_int = HS256;
	}
	else if((strncmp(protocol,(char *)"DS256", 4) == 0) || (strncmp(protocol, (char *)"ds256", 4) == 0) ||
			(strncmp(protocol,(char *)"Ds256", 4) == 0) || (strncmp(protocol, (char *)"dS256", 4) == 0)) {
		protocol_int = DS256;
	}
	else protocol_int = 0xFE;

	tag->protocol = protocol_int;

	int diff_freq = frequency-69;
	if (frequency > 69) {
		protocol_int = protocol_int+(16*diff_freq);
	}
	else if (frequency<69) {
		protocol_int = (uint8_t)protocol_int+(16*(diff_freq-1));
	}
	else {		//in case of 69 KHz do not change code type
	}

	tag->code_type = protocol_int;

}

void rs485_thread(void){
	//wait until the message was received on RS485
	char *word = "TBR Sensor";
	for(;;){
		k_sem_take(&uart_rec_sem, K_FOREVER);

		uint8_t rx_buf_temp[60];
		strcpy(rx_buf_temp,rx_buf);

		IoF_header header;
		header.TBRserial_and_headerflag = TBSN | (TBR_status_or_tag << 14);


		const char s[2]= ",";
		char *token;
		char arr[9][25]; 
		/* get the first token */
		token = strtok(rx_buf, s);
		int i =0;
		/* walk through other tokens */
		while( token != NULL ) {
			//   printf( " %s\n", token );
			strcpy(arr[i],token);
			i++;
			token = strtok(NULL, s);
		}
		header.reftimestamp = atoi(arr[1]);
		
		if(strstr(rx_buf_temp,word)){
			//log message
			header.Tbr_message_type = 255; //log message
			k_msgq_put(&IoFHEADER_MSG, &header, K_FOREVER);
			IoF_TBR_status status;
			status.secsince_timestamp = 0;
			status.code_type = 255; //code type is fixed to 255 in the TB status frame
			status.temperature = atoi(arr[3]);
			status.noise_ave = atoi(arr[4]);
			status.noise_peak = atoi(arr[5]);
			status.snr_detection = atoi(arr[6]);
			status.upper_timing_err = 0xcc;
			k_msgq_put(&IoFTBR_Status_MSG, &status, K_FOREVER);
			LOG_INF("Received LOG Message on TBR: %s", rx_buf_temp);
		}
		else{
			//tag detection
			header.Tbr_message_type = 0; //tag message
			k_msgq_put(&IoFHEADER_MSG, &header, K_FOREVER);
			IoF_TBR_tag tag;
			tag.secsince_timestamp = 0;
			
			code_type_function(&tag, arr[3], atoi(arr[7])); //write code_type
			tag.tag_id = atoi(arr[4]);
			tag.tag_payload = atoi(arr[5]);
			tag.SNR_milliseconds = (((uint16_t)(atoi(arr[2])&0xFFF))<<4) | (uint16_t)atoi(arr[6]);
			k_msgq_put(&IoFTBR_TAG_MSG, &tag, K_FOREVER);
			LOG_INF("Detected FISH on TBR: %s", rx_buf_temp);
		}

		k_sem_give(&mqtt_pub_thread_start);
		k_sem_give(&mqtt_pub_sem);
		k_sem_take(&mqtt_pub_done_sem, K_FOREVER);
		memset((void *) rx_buf, 0, sizeof(rx_buf)/sizeof(char));
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

	if(adc_init()!=0){
		LOG_ERR("Failed to initialize ADC");
		return 1;
	}

	LOG_INF("Battery voltage is: %d mV", adc_read_voltage());
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


