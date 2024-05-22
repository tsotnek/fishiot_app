#ifndef _RTC_TASK_H_
#define _RTC_TASK_H_
#include <stdint.h>

extern uint8_t rtc_init(void);
extern int16_t rtc_read_temp(void);
extern uint8_t rtc_read_time_data(void);

void rtc_thread(void);



#endif
