#include "gnss_task.h"
#include "gnss_connection.h"
#include <zephyr/logging/log.h>
#include <time.h>
#include "leds.h"
#include "structs.h"
#include "rtc.h"
#include "adc.h"
#include <stdlib.h>



LOG_MODULE_DECLARE(FishIoT);

extern struct k_sem mqtt_lock_mutex; 
extern struct k_sem gnss_thread_sem; 
// extern struct k_sem rtc_write_fix_sem; 
extern struct k_sem rtc_esyn_sem; 
extern struct k_sem gnss_start_sem; 
extern struct k_sem mqtt_pub_sem; 

extern struct k_msgq IoFHEADER_MSG; 
extern struct k_msgq IoFBuoy_Status_MSG; 

extern uint16_t TBSN; //tbr serial number

static int num_satellites = 0;
static int old_num_sattelites = 0;
extern k_tid_t rtc_thread_tid;


void gnss_event_handler(int event)
{
	int err;
	switch (event) {
	/*On a PVT event, confirm if PVT data is a valid fix */
	case NRF_MODEM_GNSS_EVT_PVT:
		LOG_INF("Searching...");
		/*Print satellite information */
		
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
			old_num_sattelites = num_satellites;
			/* Print the time to first fix */
			if (!first_fix) {
				LOG_INF("Time to first fix: %2.1lld s", (k_uptime_get() - gnss_start_time)/1000);
				first_fix = true;
				k_thread_start(rtc_thread_tid);
				// k_sem_give(&rtc_write_fix_sem);
			}
			else{
				k_sem_give(&rtc_esyn_sem);
			}
			
			k_sem_give(&gnss_thread_sem);

			return;
		}
		num_satellites=0;
		break;
	/* Log when the GNSS sleeps and wakes up */
	case NRF_MODEM_GNSS_EVT_PERIODIC_WAKEUP:
		k_sem_take(&mqtt_lock_mutex,K_NO_WAIT);
		LOG_INF("GNSS has woken up");

		// k_mutex_lock(&mqtt_lock_mutex,K_MSEC(100));

		break;
	case NRF_MODEM_GNSS_EVT_SLEEP_AFTER_FIX:
		LOG_INF("GNSS enter sleep after fix");
		// k_mutex_unlock(&mqtt_lock_mutex);
		k_sem_give(&mqtt_lock_mutex);

		break;
	default:
		break;
	}
}



uint8_t gnss_task_agps_process_start(void){

	uint8_t err;

	err = agps_receive_process_data();
	if(err){
		LOG_ERR("GNSS_THREAD: Failed to receive and process AGPS data");
		LED_ERROR_CODE(AGPS_RECEIVE_ERROR);
	}

	k_sem_take(&gnss_start_sem, K_FOREVER);
	if (gnss_init_and_start() != 0) {
		LOG_ERR("GNSS_THREAD: Failed to initialize and start GNSS");
		LED_ERROR_CODE(GNSS_INIT_ERROR);
		return 1;
	}	

	return 0;
}


//Latitude:  55.743375 should be 5544.6025N
//Longitude:37.66139 should be 3739.6834E
static void convert_to_nmea(IoF_bouy_status* bouy){
	float latitude = (float)pvt_data.latitude;  //scale down to float
	float longitude = (float)pvt_data.longitude; 

	// uint32_t lat_int = (uint32_t)latitude; //take int parts
	// uint32_t long_int = (uint32_t)longitude;

	// uint32_t lat_int_min =  (uint32_t)((latitude - (float)lat_int)*60); //44.6025
	// uint32_t lat_int_sec = (uint32_t)((((latitude - (float)lat_int)*60) - (float)lat_int_min)*60);

	// uint32_t long_int_min =  (uint32_t)((longitude - (float)long_int)*60);
	// uint32_t long_int_sec = (uint32_t)((((longitude - (float)long_int)*60) - (float)long_int_min)*60);

	// lat_int = (lat_int * 10000) + (lat_int_min * 100) + lat_int_sec;
	// long_int = (long_int * 10000) + (long_int_min * 100) + long_int_sec;

	// LOG_INF("GNSS_THREAD: NEMA Values are for LAT: %d      For LONG: %d\n", lat_int, long_int);
	// bouy->longitude = long_int * 1000;
	// bouy->latitude = lat_int * 1000;

	bouy->longitude = (uint32_t)(longitude * 10000000);
	bouy->latitude = (uint32_t)(latitude * 10000000);
}

//THREAD
void gnss_thread(void *, void *, void *){
	for(;;){
		//wait until the GNSS fix occurs
		k_sem_take(&gnss_thread_sem, K_FOREVER);
		int err = rtc_read_time_data();
		if (err != 0){
			LOG_ERR("GNSS_THREAD: Failed to read date from RTC\n");
		}
		LOG_INF("GNSS_THREAD: Time from RTC: %d:%d:%d.%d\n", rtc_time.hour, rtc_time.minute, rtc_time.seconds,rtc_time.sec100);
	
		IoF_header header;
		header.TBRserial = TBSN;
		header.headerflag = buoy_status;
		struct tm ti={
			.tm_isdst 	= -1,
			.tm_sec 	= pvt_data.datetime.seconds,
			.tm_min 	= pvt_data.datetime.minute,
			.tm_hour 	= pvt_data.datetime.hour,
			.tm_mday 	= pvt_data.datetime.day,
			.tm_mon 	= pvt_data.datetime.month - 1,
			.tm_year 	= pvt_data.datetime.year - 1900
		};
	
		time_t t_of_day = mktime(&ti);
		header.reftimestamp = (uint32_t)t_of_day;

		struct tm rtctime_struct={
			.tm_isdst 	= -1,
			.tm_sec 	= rtc_time.seconds,
			.tm_min 	= rtc_time.minute,
			.tm_hour 	= rtc_time.hour,
			.tm_mday 	= rtc_time.day,
			.tm_mon 	= rtc_time.month - 1,
			.tm_year 	= rtc_time.year + 2000 - 1900
		};
	
		time_t rtc_time_utc = mktime(&rtctime_struct);

		LOG_INF("GNSS_THREAD: UTC TIMESTAMP FOR GNSS IS: %d\n", header.reftimestamp);
		LOG_INF("GNSS_THREAD: UTC TIMESTAMP FOR RTC IS: %d\n", (uint32_t)rtc_time_utc);

		IoF_bouy_status bouy;
		bouy.batvoltage = (uint8_t)((adc_read_voltage()*100)/BAT_VOLTAGE_NOMINAL_MV);
		bouy.airtemp = rtc_read_temp();
		convert_to_nmea(&bouy);
		bouy.fix = 3; //3D fix to be compatible with IoF format
		if(old_num_sattelites == 0)
			bouy.num_of_sattelites = bouy.num_of_sattelites;
		else
			bouy.num_of_sattelites = old_num_sattelites;
		LOG_INF("Value of PDOP is %f", pvt_data.pdop);
		bouy.PDOP = (uint8_t)(pvt_data.pdop*10); //multiply by 10, because then node-red multiplies by 0.1
		
		
		if(k_msgq_num_used_get(&IoFHEADER_MSG) >= 16){
			//purge oldest value
			LOG_INF("GNSS_THREAD: IoFHeaderMSG queue is full... purging\n");
			if(k_msgq_get(&IoFHEADER_MSG, NULL, K_FOREVER) != 0){
				LOG_INF("GNSS_THREAD: IoFHeaderMSG queue couldn't be purged\n");
			}
		}
		if(k_msgq_put(&IoFHEADER_MSG, &header, K_FOREVER)!=0){
			LOG_INF("GNSS_THREAD: Message couldn't be placed in IoFHeaderMSG queue\n");
		}

		if(k_msgq_num_used_get(&IoFBuoy_Status_MSG) >= 16){
			//purge oldest value
			LOG_INF("GNSS_THREAD: IoFBuoy_Status_MSG queue is full... purging\n");
			if(k_msgq_get(&IoFBuoy_Status_MSG, NULL, K_FOREVER) != 0){
				LOG_INF("GNSS_THREAD: IoFBuoy_Status_MSG queue couldn't be purged\n");
			}
		}
		if(k_msgq_put(&IoFBuoy_Status_MSG, &bouy, K_FOREVER)!=0){
			LOG_INF("GNSS_THREAD: Message couldn't be placed in IoFBuoy_Status_MSG queue\n");
		}
		// if(k_msgq_num_used_get(&IoFHEADER_MSG)>5)
			k_sem_give(&mqtt_pub_sem);

	}
}