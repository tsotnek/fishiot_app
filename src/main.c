/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/mqtt.h>
#include <net/nrf_cloud.h>
#include <net/nrf_cloud_agps.h>
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <date_time.h>

#include "mqtt_connection.h"
//header file for the GNSS interface.
// #include <nrf_modem_gnss.h>

struct nrf_cloud_init_param nrf_cloud_struct;

//PVT data frame variables,
// static struct nrf_modem_gnss_pvt_data_frame pvt_data;

/*Helper variables to find the TTFF */
// static int64_t gnss_start_time;
// static bool first_fix = false;

/* The mqtt client struct */
static struct mqtt_client client;
/* File descriptor */
static struct pollfd fds;

static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(mqtt_pub_sem, 0, 1);
static K_SEM_DEFINE(start_gnss, 0, 1);
// static K_SEM_DEFINE(start_gnss2, 1, 1);
static K_SEM_DEFINE(time_sem, 0, 1);

LOG_MODULE_REGISTER(FishIoT, LOG_LEVEL_INF);

extern bool mqtt_connack_bool;

#define STACKSIZE 1024
#define MQTT_PUB_PRIORITY 7

K_THREAD_STACK_DEFINE(my_stack_area, STACKSIZE);

struct k_thread mqtt_pub_thread;


//Function prototypes
static void lte_handler(const struct lte_lc_evt *const evt);
static int modem_configure(void);

// static void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt_data);
// static int gnss_init_and_start(void);
// static void gnss_event_handler(int event);
static void nrf_cloud_handler(const struct nrf_cloud_evt *evt);
// // static void nrf_cloud_event_handler_t(const struct nrf_cloud_evt *evt);
static int cloud_setup_and_agps_request(void);
//Threads

// bool onetime = false;
// void mqtt_pub_thread_function(void)
// {
// 	if(onetime == false){
// 	int err;
// 	err = cloud_setup_and_agps_request();
// 	if (err) {
// 		LOG_ERR("Failed to setup NRF cloud and request AGPS data");
// 	}
// 	onetime == true;
// 	}
// }
void mqtt_pub_thread_function(void)
{

	
	// k_sem_take(&mqtt_pub_sem, K_FOREVER);
	int err;
	
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

	
	while (!mqtt_connack_bool) {
		err = poll(&fds, 1, mqtt_keepalive_time_left(&client));
		if (err < 0) {
			LOG_ERR("Error in poll(): %d", errno);
			break;
		}

		err = mqtt_live(&client);
		if ((err != 0) && (err != -EAGAIN)) {
			LOG_ERR("Error in mqtt_live: %d", err);
			break;
		}

		if ((fds.revents & POLLIN) == POLLIN) {
			err = mqtt_input(&client);
			if (err != 0) {
				LOG_ERR("Error in mqtt_input: %d", err);
				break;
			}
		}

	}
	mqtt_connack_bool = false;
	err = data_publish(&client, MQTT_QOS_1_AT_LEAST_ONCE,
				   CONFIG_BUTTON_EVENT_PUBLISH_MSG, sizeof(CONFIG_BUTTON_EVENT_PUBLISH_MSG)-1);
		if (err) {
			LOG_INF("Failed to send message, %d", err);
					}

	LOG_INF("Disconnecting MQTT client");

	err = mqtt_disconnect(&client);
	if (err) {
		LOG_ERR("Could not disconnect MQTT client: %d", err);
	}
	k_sleep(K_SECONDS(5*60));
	goto do_connect;
}


int cloud_connected; 
static void nrf_cloud_handler(const struct nrf_cloud_evt *evt)
{
	switch(evt->type)
	{
		case NRF_CLOUD_EVT_TRANSPORT_CONNECTED:
			LOG_INF("Connected to cloud successfully");
			k_sem_give(&start_gnss);
			break;
		case NRF_CLOUD_EVT_TRANSPORT_CONNECTING:
			LOG_INF("Connecting to cloud...");
			break;
		case NRF_CLOUD_EVT_USER_ASSOCIATION_REQUEST:
			LOG_INF("There was a request from nRF Cloud to associate the device with a user on the nRF Cloud.");
			break;
		case NRF_CLOUD_EVT_USER_ASSOCIATED:
			LOG_INF("The device is successfully associated with a user.");
			break;
		case NRF_CLOUD_EVT_READY:
			LOG_INF("NRF_CLOUD_EVT_READY");
			break;
		case NRF_CLOUD_EVT_RX_DATA_GENERAL:
			LOG_INF("The device received non-specific data from the cloud.");
			break;
		case NRF_CLOUD_EVT_RX_DATA_LOCATION:
			LOG_INF("The device received location data from the cloud and no response callback was registered.");
			// int err = nrf_cloud_agps_process((evt->data).ptr, (evt->data).len);
			// if(err){
			// 	LOG_ERR("Error ooccured %d", err);
			// }
			break;
		case NRF_CLOUD_EVT_RX_DATA_SHADOW:
			LOG_INF("The device received shadow related data from the cloud.");
			break;
		case NRF_CLOUD_EVT_PINGRESP:
			LOG_INF("The device has received a ping response from the cloud.");
			break;
		case NRF_CLOUD_EVT_SENSOR_DATA_ACK:
			LOG_INF("The data sent to the cloud was acknowledged.");
			break;
		case NRF_CLOUD_EVT_TRANSPORT_DISCONNECTED:
			LOG_INF("The transport was disconnected. The status field in the event struct will be populated with a nrf_cloud_disconnect_status value.");
			break;



		case NRF_CLOUD_EVT_ERROR:
			LOG_INF("Cloud evt failed");
			// cloud_connected = 2;
			break;
		
		default:
			LOG_INF("Some unhandled event in NRF cloud handler occured, type: %d", evt->type);
			break;
			
	}
}




int main(void)
{
	int err;
	printk("Enabling modem...\n");
	// err = nrf_modem_lib_init();
	// if (err) {
	// 	printk("Failed to initialize the modem library, error: %d\n", err);
	// 	return err;
	// }

	// lte_lc_init();
	// lte_lc_register_handler(lte_handler);

	// /* Enable PSM. */
	// lte_lc_psm_req(true);
	// lte_lc_connect();

	// k_sem_take(&lte_connected, K_FOREVER);

	// char ID[100];



	// if(err)
	// {
	// 	LOG_ERR("Could not connect to nrf CLOUD");
	// 	// goto retry_connect;
	// }
	// // while(!cloud_connected);
	// // if(cloud_connected==2)
	// // 	goto retry_connect;
	// k_sem_take(&start_gnss, K_FOREVER);
	// err = nrf_cloud_agps_request_all();
	// if (err)
	// {
	// 	LOG_ERR("Could not request AGPS data");
	// }

	if (dk_leds_init() != 0) {
		LOG_ERR("Failed to initialize the LED library");
	}

	err = modem_configure();
	if (err) {
		LOG_ERR("Failed to configure the modem");
	}
	
	
	nrf_cloud_struct.event_handler = &nrf_cloud_handler;
	err = nrf_cloud_init(&nrf_cloud_struct);
	if(err){
		printk("Could not initialize nrf CLOUD\n");
		return 1;
	}

	err = nrf_cloud_connect();
	// err = client_init(&client);
	// if (err) {
	// 	LOG_ERR("Failed to initialize MQTT client: %d", err);
	// }
	// k_thread_create(&mqtt_pub_thread, my_stack_area,
    //                              K_THREAD_STACK_SIZEOF(my_stack_area),
    //                              (void *)&mqtt_pub_thread_function,
    //                              NULL, NULL, NULL,
    //                              MQTT_PUB_PRIORITY, 0, K_SECONDS(30));
	
	k_sem_take(&start_gnss,K_FOREVER);
	// err = nrf_cloud_agps_request_all();
	// if (err) {
	// 	LOG_ERR("Failed to access AGNSS data");
	// }

	int idname[100];
	err = nrf_cloud_client_id_get(idname,NRF_CLOUD_CLIENT_ID_MAX_LEN);
	if (err)
	{
		LOG_ERR("Error getting ID from nrf cloud");
	}

	LOG_INF("The Retrieved ID IS: %s",idname);

	// if (gnss_init_and_start() != 0) {
	// 	LOG_ERR("Failed to initialize and start GNSS");
	// 	return 0;
	// }

	

	

	/* This is never reached */
	return 0;
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
		break;
	case LTE_LC_EVT_PSM_UPDATE:
		LOG_INF("PSM parameter update: TAU: %d, Active time: %d",
			evt->psm_cfg.tau, evt->psm_cfg.active_time);
		if (evt->psm_cfg.active_time == -1){
			LOG_ERR("Network rejected PSM parameters. Failed to enable PSM");
		
	}
     default:
             break;
     }
}

static void date_time_evt_handler(const struct date_time_evt *evt)
{
	k_sem_give(&time_sem);
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


	
	if (lte_lc_init() != 0) {
		LOG_ERR("Failed to initialize LTE link controller");
		return -1;
	}


	//enable PSM
	err = lte_lc_psm_req(true);
	if (err) {
		LOG_ERR("lte_lc_psm_req, error: %d", err);
	}

	LOG_INF("Connecting to LTE network");

	if (lte_lc_connect() != 0) {
		LOG_ERR("Failed to connect to LTE network");
		return -1;
	}

	LOG_INF("Connected to LTE network");

	// k_sem_take(&lte_connected, K_FOREVER);
	dk_set_led_on(DK_LED2);
	if (IS_ENABLED(CONFIG_DATE_TIME)) {
		LOG_INF("Waiting for current time");

		/* Wait for an event from the Date Time library. */
		// k_sem_take(&time_sem, K_MINUTES(10));

		if (!date_time_is_valid()) {
			LOG_WRN("Failed to get current time, continuing anyway");
		}
	}

	return 0;
}


// static void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
// {
// 	LOG_INF("Latitude:       %.06f", pvt_data->latitude);
// 	LOG_INF("Longitude:      %.06f", pvt_data->longitude);
// 	LOG_INF("Altitude:       %.01f m", pvt_data->altitude);
// 	LOG_INF("Time (UTC):     %02u:%02u:%02u.%03u",
// 	       pvt_data->datetime.hour,
// 	       pvt_data->datetime.minute,
// 	       pvt_data->datetime.seconds,
// 	       pvt_data->datetime.ms);
// }


// static void gnss_event_handler(int event)
// {
// 	int err;
// 	switch (event) {
// 	/* STEP 7 - On a PVT event, confirm if PVT data is a valid fix */
// 	case NRF_MODEM_GNSS_EVT_PVT:
// 		LOG_INF("Searching...");
// 		/* STEP 15 - Print satellite information */
// 		int num_satellites = 0;
// 		for (int i = 0; i < 12 ; i++) {
// 			if (pvt_data.sv[i].signal != 0) {
// 				LOG_INF("sv: %d, cn0: %d", pvt_data.sv[i].sv, pvt_data.sv[i].cn0);
// 				num_satellites++;
// 			}
// 		}
// 		LOG_INF("Number of current satellites: %d", num_satellites);
// 		err = nrf_modem_gnss_read(&pvt_data, sizeof(pvt_data), NRF_MODEM_GNSS_DATA_PVT);
// 		if (err) {
// 			LOG_ERR("nrf_modem_gnss_read failed, err %d", err);
// 			return;
// 		}
// 		if (pvt_data.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {
// 			dk_set_led_on(DK_LED1);
// 			print_fix_data(&pvt_data);
// 			/* STEP 12.3 - Print the time to first fix */
// 			if (!first_fix) {
// 				LOG_INF("Time to first fix: %2.1lld s", (k_uptime_get() - gnss_start_time)/1000);
// 				first_fix = true;
// 			}
// 			return;
// 		}
// 		break;
// 	/* STEP 7.2 - Log when the GNSS sleeps and wakes up */
// 	case NRF_MODEM_GNSS_EVT_PERIODIC_WAKEUP:
// 		LOG_INF("GNSS has woken up");
// 		break;
// 	case NRF_MODEM_GNSS_EVT_SLEEP_AFTER_FIX:
// 		LOG_INF("GNSS enter sleep after fix");
// 		break;
// 	default:
// 		break;
// 	}
// }



// static int gnss_init_and_start(void)
// {

// 	k_sem_take(&start_gnss2, K_FOREVER);
// 	k_sem_take(&start_gnss, K_FOREVER);
// 	/* STEP 4 - Set the modem mode to normal */
// 	if (lte_lc_func_mode_set(LTE_LC_FUNC_MODE_NORMAL) != 0) {
// 		LOG_ERR("Failed to activate GNSS functional mode");
// 		return -1;
// 	}

// 	if (nrf_modem_gnss_event_handler_set(gnss_event_handler) != 0) {
// 		LOG_ERR("Failed to set GNSS event handler");
// 		return -1;
// 	}

// 	if (nrf_modem_gnss_fix_interval_set(CONFIG_GNSS_PERIODIC_INTERVAL) != 0) {
// 		LOG_ERR("Failed to set GNSS fix interval");
// 		return -1;
// 	}

// 	if (nrf_modem_gnss_fix_retry_set(CONFIG_GNSS_PERIODIC_TIMEOUT) != 0) {
// 		LOG_ERR("Failed to set GNSS fix retry");
// 		return -1;
// 	}

// 	LOG_INF("Starting GNSS");
// 	if (nrf_modem_gnss_start() != 0) {
// 		LOG_ERR("Failed to start GNSS");
// 		return -1;
// 	}

// 	gnss_start_time = k_uptime_get();

// 	return 0;
// }