#ifndef _GNSSCONNECTION_H_
#define _GNSSCONNECTION_H_

#include <stdint.h>
#include <nrf_modem_gnss.h>

//GNSS
extern int64_t gnss_start_time;
//PVT data frame variables,
extern struct nrf_modem_gnss_pvt_data_frame pvt_data;
extern bool first_fix;

void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt_data);
int agps_receive_process_data(void);
int gnss_init_and_start(void);
void gnss_event_handler(int event);
void data_formatter(struct nrf_modem_gnss_pvt_data_frame *pvt_data);

#endif