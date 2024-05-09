#include <zephyr/logging/log.h>
#include <nrf_modem_gnss.h>
#include <net/nrf_cloud.h>
#include <net/nrf_cloud_agnss.h>
#include <net/nrf_cloud_rest.h>
#include <modem/modem_info.h>
#include <modem/modem_jwt.h>
#include "gnss_connection.h"

#define PPS


LOG_MODULE_DECLARE(FishIoT);


//GNSS
int64_t gnss_start_time;
//PVT data frame variables,
struct nrf_modem_gnss_pvt_data_frame pvt_data;

static char recv_buffer[2048];
static char agps_data_buf[3500];
static char jwt_buf[600];

// int64_t gnss_start_time;


void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
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


int agps_receive_process_data(void){
    
    int err;

    struct nrf_cloud_rest_agnss_request nrf_agps_requst_struct = {
		.type = NRF_CLOUD_REST_AGNSS_REQ_ASSISTANCE,
		.agnss_req = NULL,
		.net_info = NULL,
	};

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
	struct nrf_cloud_rest_agnss_result result = {
		.buf = agps_data_buf,
		.buf_sz = sizeof(agps_data_buf),
		.agnss_sz = 0
	};




    err = nrf_cloud_jwt_generate(0, jwt_buf, sizeof(jwt_buf));
	if (err) {
		LOG_ERR("Failed to generate JWT, error: %d", err);
        return 1;
	}
	LOG_INF("Generated JWT successfully!");

	LOG_INF("Requesting AGPS Data...");
	err = nrf_cloud_rest_agnss_data_get(&nrf_rest_struct, &nrf_agps_requst_struct, &result);
	if(err)
	{
		LOG_ERR("Unsuccessful rest data get, error %d", err);
        return 1;
	}
	LOG_INF("%s",recv_buffer);

	LOG_INF("Starting to process AGPS data...");
	err = nrf_cloud_agnss_process(result.buf, result.agnss_sz);
	if(err)
	{
		LOG_ERR("Unable to parse the the AGPS");
        return 1;
	}
	LOG_INF("Successfully parsed AGPS");
    return 0;
}


int gnss_init_and_start(void)
{

	/* Set the modem mode to normal */
	if (lte_lc_func_mode_set(LTE_LC_FUNC_MODE_NORMAL) != 0) {
		LOG_ERR("Failed to activate GNSS functional mode");
		return 1;
	}

	if (nrf_modem_gnss_event_handler_set(gnss_event_handler) != 0) {
		LOG_ERR("Failed to set GNSS event handler");
		return 1;
	}

	if (nrf_modem_gnss_fix_interval_set(CONFIG_GNSS_PERIODIC_INTERVAL) != 0) {
		LOG_ERR("Failed to set GNSS fix interval");
		return 1;
	}

	if (nrf_modem_gnss_fix_retry_set(CONFIG_GNSS_PERIODIC_TIMEOUT) != 0) {
		LOG_ERR("Failed to set GNSS fix retry");
		return 1;
	}

#ifdef PPS
	struct nrf_modem_gnss_1pps_config pps_config = {
    .pulse_interval = 1,
    .pulse_width = 100,
    .apply_start_time = false
	};

	int err = nrf_modem_gnss_1pps_enable(&pps_config);
	if(err){
		LOG_INF("Failed to enable 1pps");
	}
#endif

	LOG_INF("Starting GNSS");
	if (nrf_modem_gnss_start() != 0) {
		LOG_ERR("Failed to start GNSS");
		return 1;
	}

	gnss_start_time = k_uptime_get();

	return 0;
}