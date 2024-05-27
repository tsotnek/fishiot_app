/*
* FishIoT Main Application
* MCU:nRF9160
*
* Author: Tsotne Karchava
* Created: 10.10.2023
*/

#include <zephyr/logging/log.h>
#include <date_time.h>
#include <zephyr/sys/reboot.h>
//LTE
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
//header file for MQTT
// #include "rtc.h"
#include "leds.h"
#include "structs.h"
#include "rs485_task.h"
#include "mqtt_task.h"
#include "gnss_task.h"
#include "rtc_task.h"
#include "tbr_sync_task.h"

#define STACKSIZE 2048
#define MQTT_THREAD_PRIORITY 9
#define RTC_THREAD_PRIORITY 7
#define RS485_THREAD_PRIORITY 6
#define GNSS_THREAD_PRIORITY 8
#define TBR_SYNC_THREAD_PRIORITY 5

bool RTC_TIME_SET = false;


LOG_MODULE_REGISTER(FishIoT, LOG_LEVEL_INF);

K_MSGQ_DEFINE(IoFHEADER_MSG, sizeof(IoF_header), 32, sizeof(uint32_t));
K_MSGQ_DEFINE(IoFTBR_Status_MSG, sizeof(IoF_TBR_status), 16, sizeof(uint32_t));
K_MSGQ_DEFINE(IoFTBR_TAG_MSG, sizeof(IoF_TBR_tag), 16, sizeof(uint32_t));
K_MSGQ_DEFINE(IoFBuoy_Status_MSG, sizeof(IoF_bouy_status), 16, sizeof(uint32_t));


//semaphores
static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(time_sem, 0, 1);
static K_SEM_DEFINE(time_read_sem, 0, 1);

K_SEM_DEFINE(gnss_start_sem, 0, 1);

// K_SEM_DEFINE(rtc_write_fix_sem, 0, 1);
K_SEM_DEFINE(rtc_esyn_sem, 0, 1);

K_SEM_DEFINE(gnss_thread_sem, 0 , 1);

K_SEM_DEFINE(mqtt_pub_sem, 0, 1);
K_SEM_DEFINE(mqtt_pub_thread_start, 0, 1);
K_SEM_DEFINE(mqtt_pub_done_sem, 0, 1);


K_SEM_DEFINE(mqtt_lock_mutex, 0, 1);

uint16_t TBSN; //tbr serial number

//Function prototypes
static int modem_configure(void);

//handlers
static void lte_handler(const struct lte_lc_evt *const evt);

//thread definitions
K_THREAD_STACK_DEFINE(mqtt_thread_id, STACKSIZE);
K_THREAD_STACK_DEFINE(rtc_thread_id, STACKSIZE);
K_THREAD_STACK_DEFINE(rs485_thread_id, STACKSIZE);
K_THREAD_STACK_DEFINE(gnss_thread_id, STACKSIZE);
K_THREAD_STACK_DEFINE(tbr_sync_thread_id, STACKSIZE/2);



struct k_thread mqtt_pub_thread_struct;
struct k_thread rtc_thread_struct;
struct k_thread rs485_thread_struct;
struct k_thread gnss_thread_struct;
struct k_thread tbr_sync_thread_struct;


k_tid_t rtc_thread_tid;
k_tid_t gnss_thread_tid;
k_tid_t mqtt_thread_tid;
k_tid_t tbr_sync_thread_tid;


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


static int modem_configure(void)
{
	int err;

	LOG_INF("Initializing modem library");

	err = nrf_modem_lib_init();
	if (err) {
		LOG_ERR("Failed to initialize the modem library, error: %d", err);
		return err;
	}
	
	#if(CONFIG_MQTT_LIB_TLS)
	/*  Store the certificate in the modem while the modem is in offline mode  */
	err = certificate_provision();
	if (err != 0) {
		LOG_ERR("Failed to provision certificates");
		return 1;
	}
	#endif

	/* lte_lc_init deprecated in >= v2.6.0 */
	#if NCS_VERSION_NUMBER < 0x20600
	err = lte_lc_init();
	if (err) {
		LOG_ERR("Failed to initialize LTE link control library, error: %d", err);
		return err;
	}
	#endif

	//enable PSM
	err = lte_lc_psm_req(true);
	if (err) {
		LOG_ERR("lte_lc_psm_req, error: %d", err);
	}

	LOG_INF("Connecting to LTE network");

	lte_lc_register_handler(lte_handler);

	if (lte_lc_connect() != 0) {
		LED_ERROR_CODE(LTE_NETWORK_CONNECT_ERROR);
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

	LOG_INF("Received current time successfully!");
	return 0;
}


int main(void)
{
	int err;
	LOG_INF("Starting Application IoF...\n");
	// k_mutex_lock(&mqtt_lock_mutex, K_FOREVER);	
	if(led_button_init() != 0){
		LOG_ERR("Failed to initialize LED and Buttons");
		return 1;
	}
	
	if(rtc_init()!=0){
		LOG_ERR("Failed to initialize RTC\n");
		LED_ERROR_CODE(RTC_INIT_ERROR);
		LOG_INF("Waiting for 1 minute before restarting the chip\n");
		k_busy_wait(1*1000*1000*60);
		sys_reboot(SYS_REBOOT_COLD);
		return 1;
	};


	
	if(adc_init()!=0){
		LOG_ERR("Failed to initialize ADC");
		LED_ERROR_CODE(ADC_INIT_ERROR);
		LOG_INF("Waiting for 1 minute before restarting the chip\n");
		k_busy_wait(1*1000*1000*60);
		sys_reboot(SYS_REBOOT_COLD);
		return 1;
	}

	LOG_INF("Enabling modem...\n");
	err = modem_configure();
	if (err) {
		LOG_ERR("Failed to configure the modem");
		LOG_INF("Waiting for 1 minute before restarting the chip\n");
		k_busy_wait(1*1000*1000*60);
		sys_reboot(SYS_REBOOT_WARM);
		return 1;
	}

	// k_sem_give(&mqtt_pub_thread_start);
	mqtt_thread_tid = k_thread_create(&mqtt_pub_thread_struct, mqtt_thread_id, K_THREAD_STACK_SIZEOF(mqtt_thread_id), mqtt_thread, NULL,NULL,NULL, MQTT_THREAD_PRIORITY, 0, K_NO_WAIT);

	err = rs485_init();
	if(err){
		LOG_ERR("Failed to initialize RS485 - TBRLIVE");
		LED_ERROR_CODE(RS485_INIT_ERROR);
		LOG_INF("Waiting for 1 minute before restarting the chip\n");
		k_busy_wait(1*1000*1000*60);
		sys_reboot(SYS_REBOOT_COLD);
		return 1;
	}
	
	//if Time is valid then update TBR live enoch time.
	uint64_t unix_time_ms;
	/*Read current time and put in container */
	err = date_time_now(&unix_time_ms);
	//write enoch time to TBRLive
	err = rs485_updatetime(unix_time_ms);
	if(err != 0){
		LOG_ERR("Couldn't update RS485 time...\n");
	}	
	
	err = gnss_task_agps_process_start();
	if (err != 0)
	{
		LOG_INF("Waiting for 1 minute before restarting the chip\n");
		k_busy_wait(1*1000*1000*60);
		sys_reboot(SYS_REBOOT_COLD);
	}
	
	rtc_thread_tid = k_thread_create(&rtc_thread_struct, rtc_thread_id, K_THREAD_STACK_SIZEOF(rtc_thread_id), rtc_thread, NULL,NULL,NULL, RTC_THREAD_PRIORITY, 0, K_FOREVER);

	// rtc_thread_tid = k_thread_create(&rtc_thread_struct, rtc_thread_id, K_THREAD_STACK_SIZEOF(rtc_thread_id), rtc_thread, NULL,NULL,NULL, RTC_THREAD_PRIORITY, 0, K_FOREVER);
	gnss_thread_tid = k_thread_create(&gnss_thread_struct, gnss_thread_id, K_THREAD_STACK_SIZEOF(gnss_thread_id), gnss_thread, NULL,NULL,NULL, GNSS_THREAD_PRIORITY, 0, K_FOREVER);

	k_tid_t rs485_thread_tid = k_thread_create(&rs485_thread_struct, rs485_thread_id, K_THREAD_STACK_SIZEOF(rs485_thread_id), rs485_thread, NULL,NULL,NULL, RS485_THREAD_PRIORITY, 0, K_NO_WAIT);
	tbr_sync_thread_tid = k_thread_create(&tbr_sync_thread_struct, tbr_sync_thread_id, K_THREAD_STACK_SIZEOF(tbr_sync_thread_id), tbr_sync_thread, NULL,NULL,NULL, TBR_SYNC_THREAD_PRIORITY, 0, K_NO_WAIT);
	return 0;
}


