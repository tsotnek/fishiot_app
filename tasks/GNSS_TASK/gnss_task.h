#ifndef _GNSS_TASK_H_
#define _GNSS_TASK_H_
#include <stdint.h>

extern uint8_t first_fix;
extern struct nrf_modem_gnss_pvt_data_frame pvt_data;
extern uint8_t adc_init(void);

void gnss_event_handler(int event);
void gnss_thread(void *, void *, void *);
void gnss_start_period_thread(void *, void *, void *);
uint8_t gnss_task_agps_process_start(void);

#endif