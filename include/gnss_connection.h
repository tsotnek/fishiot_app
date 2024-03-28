#ifndef _GNSSCONNECTION_H_
#define _GNSSCONNECTION_H_

void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt_data);
int agps_receive_process_data(void);
int gnss_init_and_start(void);
void gnss_event_handler(int event);

#endif