/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
// #include <zephyr/net/mqtt.h>
#include <net/nrf_cloud.h>
#include <net/nrf_cloud_agps.h>
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <modem/modem_info.h>
#include <modem/modem_jwt.h>
#include <date_time.h>
#include <net/nrf_cloud_rest.h>
#include <time.h>
#include "mqtt_connection.h"
//header file for the GNSS interface.
#include <nrf_modem_gnss.h>

//PVT data frame variables,
static struct nrf_modem_gnss_pvt_data_frame pvt_data;

/*Helper variables to find the TTFF */
static int64_t gnss_start_time;
static bool first_fix = false;

static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(time_sem, 0, 1);

LOG_MODULE_REGISTER(FishIoT, LOG_LEVEL_INF);

static char recv_buffer[2048];
static char agps_data_buf[3500];
static char jwt_buf[600];


//Function prototypes
static void lte_handler(const struct lte_lc_evt *const evt);
static int modem_configure(void);

static void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt_data);
static int gnss_init_and_start(void);
static void gnss_event_handler(int event);

// static void nrf_cloud_handler(const struct nrf_cloud_evt *evt)
// {
// 	switch(evt->type)
// 	{
// 		case NRF_CLOUD_EVT_TRANSPORT_CONNECTED:
// 			LOG_INF("Connected to cloud successfully");
// 			k_sem_give(&start_gnss);
// 			break;
// 		case NRF_CLOUD_EVT_TRANSPORT_CONNECTING:
// 			LOG_INF("Connecting to cloud...");
// 			break;
// 		case NRF_CLOUD_EVT_USER_ASSOCIATION_REQUEST:
// 			LOG_INF("There was a request from nRF Cloud to associate the device with a user on the nRF Cloud.");
// 			break;
// 		case NRF_CLOUD_EVT_USER_ASSOCIATED:
// 			LOG_INF("The device is successfully associated with a user.");
// 			break;
// 		case NRF_CLOUD_EVT_READY:
// 			LOG_INF("NRF_CLOUD_EVT_READY");
// 			break;
// 		case NRF_CLOUD_EVT_RX_DATA_GENERAL:
// 			LOG_INF("The device received non-specific data from the cloud.");
// 			break;
// 		case NRF_CLOUD_EVT_RX_DATA_LOCATION:
// 			LOG_INF("The device received location data from the cloud and no response callback was registered.");
// 			// int err = nrf_cloud_agps_process((evt->data).ptr, (evt->data).len);
// 			// if(err){
// 			// 	LOG_ERR("Error ooccured %d", err);
// 			// }
// 			break;
// 		case NRF_CLOUD_EVT_RX_DATA_SHADOW:
// 			LOG_INF("The device received shadow related data from the cloud.");
// 			break;
// 		case NRF_CLOUD_EVT_PINGRESP:
// 			LOG_INF("The device has received a ping response from the cloud.");
// 			break;
// 		case NRF_CLOUD_EVT_SENSOR_DATA_ACK:
// 			LOG_INF("The data sent to the cloud was acknowledged.");
// 			break;
// 		case NRF_CLOUD_EVT_TRANSPORT_DISCONNECTED:
// 			LOG_INF("The transport was disconnected. The status field in the event struct will be populated with a nrf_cloud_disconnect_status value.");
// 			break;
// 		case NRF_CLOUD_EVT_ERROR:
// 			LOG_INF("Cloud evt failed");
// 			// cloud_connected = 2;
// 			break;
// 		default:
// 			LOG_INF("Some unhandled event in NRF cloud handler occured, type: %d", evt->type);
// 			break;
// 	}
// }


int main(void)
{
	int err;
	printk("Enabling modem...\n");
	
	if (dk_leds_init() != 0) {
		LOG_ERR("Failed to initialize the LED library");
	}

	err = modem_configure();
	if (err) {
		LOG_ERR("Failed to configure the modem");
	}
	

	struct nrf_cloud_rest_agps_request nrf_agps_requst_struct = {
		.type = NRF_CLOUD_REST_AGPS_REQ_ASSISTANCE,
		.agps_req = NULL,
		.net_info = NULL,
	};

		err = nrf_cloud_jwt_generate(0, jwt_buf, sizeof(jwt_buf));
	if (err) {
		LOG_ERR("Failed to generate JWT, error: %d", err);
	}

	struct nrf_cloud_rest_context nrf_rest_struct = {
		.connect_socket = -1,
		.keep_alive = false,
		.timeout_ms = NRF_CLOUD_REST_TIMEOUT_NONE,
		.auth = jwt_buf,
		.rx_buf = recv_buffer,
		.rx_buf_len = sizeof(recv_buffer),
		.fragment_size = 0, /* Defaults to CONFIG_NRF_CLOUD_REST_FRAGMENT_SIZE when 0 */
		.status = 0,
		.response = NULL,
		.response_len = 0,
		.total_response_len = 0
	};
	struct nrf_cloud_rest_agps_result result = {
		.buf = agps_data_buf,
		.buf_sz = sizeof(agps_data_buf),
		.agps_sz = 0
	};

	err = nrf_cloud_rest_agps_data_get(&nrf_rest_struct, &nrf_agps_requst_struct, &result);
	if(err)
	{
		LOG_ERR("Unsuccessful rest data get, error %d", err);
	}
	LOG_INF("%s",recv_buffer);

	err = nrf_cloud_agps_process(result.buf, result.agps_sz);
	if(err)
	{
		LOG_ERR("Unable to parse the the AGPS");
	}
	LOG_INF("Successfully parsed AGPS");

	if (gnss_init_and_start() != 0) {
		LOG_ERR("Failed to initialize and start GNSS");
		return 0;
	}

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

	// date_time_register_handler(date_time_evt_handler);
	
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

	lte_lc_register_handler(lte_handler);

	if (lte_lc_connect() != 0) {
		LOG_ERR("Failed to connect to LTE network");
		return -1;
	}

	LOG_INF("Connected to LTE network");
	err = date_time_update_async(date_time_evt_handler);
	if(err){
		LOG_ERR("Date time update handler failed");
	}	
	dk_set_led_on(DK_LED2);
	LOG_INF("Waiting for current time");
	// time_t unix_time;
	// struct tm * timeinfo;
	// if(date_time_now(&unix_time)==0){
	// 	timeinfo = localtime(&unix_time);
	// 	if(date_time_set(&timeinfo)==0)
	// 	{
	// 		k_sem_give(&time_sem);
	// 	}
	// 	else
	// 		LOG_ERR("Time set failed");
	// }
	
	
	
	/* Wait for an event from the Date Time library. */
	k_sem_take(&time_sem, K_MINUTES(10));

	if (!date_time_is_valid()) {
		LOG_WRN("Failed to get current time, continuing anyway");
	}
	return 0;
}


static void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	LOG_INF("Latitude:       %.06f", pvt_data->latitude);
	LOG_INF("Longitude:      %.06f", pvt_data->longitude);
	LOG_INF("Altitude:       %.01f m", pvt_data->altitude);
	LOG_INF("Time (UTC):     %02u:%02u:%02u.%03u",
	       pvt_data->datetime.hour,
	       pvt_data->datetime.minute,
	       pvt_data->datetime.seconds,
	       pvt_data->datetime.ms);
}


static void gnss_event_handler(int event)
{
	int err;
	switch (event) {
	/* STEP 7 - On a PVT event, confirm if PVT data is a valid fix */
	case NRF_MODEM_GNSS_EVT_PVT:
		LOG_INF("Searching...");
		/* STEP 15 - Print satellite information */
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
			dk_set_led_on(DK_LED1);
			print_fix_data(&pvt_data);
			/* STEP 12.3 - Print the time to first fix */
			if (!first_fix) {
				LOG_INF("Time to first fix: %2.1lld s", (k_uptime_get() - gnss_start_time)/1000);
				first_fix = true;
			}
			return;
		}
		break;
	/* STEP 7.2 - Log when the GNSS sleeps and wakes up */
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



static int gnss_init_and_start(void)
{

	/* STEP 4 - Set the modem mode to normal */
	if (lte_lc_func_mode_set(LTE_LC_FUNC_MODE_NORMAL) != 0) {
		LOG_ERR("Failed to activate GNSS functional mode");
		return -1;
	}

	if (nrf_modem_gnss_event_handler_set(gnss_event_handler) != 0) {
		LOG_ERR("Failed to set GNSS event handler");
		return -1;
	}

	if (nrf_modem_gnss_fix_interval_set(CONFIG_GNSS_PERIODIC_INTERVAL) != 0) {
		LOG_ERR("Failed to set GNSS fix interval");
		return -1;
	}

	if (nrf_modem_gnss_fix_retry_set(CONFIG_GNSS_PERIODIC_TIMEOUT) != 0) {
		LOG_ERR("Failed to set GNSS fix retry");
		return -1;
	}

	LOG_INF("Starting GNSS");
	if (nrf_modem_gnss_start() != 0) {
		LOG_ERR("Failed to start GNSS");
		return -1;
	}

	gnss_start_time = k_uptime_get();

	return 0;
}