#include "tbr_sync_task.h"
#include <zephyr/logging/log.h>
#include "leds.h"
#include "rs485.h"
#include "rtc.h"
#include <time.h>
#include <stdlib.h>

LOG_MODULE_DECLARE(FishIoT);

extern bool RTC_TIME_SET;

static uint64_t tbr_sync_rtc_read_time(void){
    rtc_read_time_data();
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
    return (uint64_t) rtc_time_utc;
}

void tbr_sync_thread(void *, void *, void *){
    char rx_buffer[MSG_SIZE];
	const char s[2]= ",";
    char *token;
    for(;;){
        k_sem_take(&tbr_sync_task_sem, K_FOREVER);
        strcpy(rx_buffer, rx_buf);
        token = strtok(rx_buffer, s); //garbage
        uint64_t tbr_timestamp = atoi(strtok(NULL,s)); //this gives timestamp

        if(RTC_TIME_SET){
            //read time from RTC
            uint64_t rtc_timestamp = tbr_sync_rtc_read_time();

            LOG_INF("TBR_SYNC_TASK: value for tbr_timestamp is: %lld, value for rtc_timestamp is: %lld\n", tbr_timestamp, rtc_timestamp);
            int diff = rtc_timestamp-tbr_timestamp;
            if(diff >= 1){
                if(rs485_updatetime(rtc_timestamp+diff)!= 0){
                    LOG_INF("TBR_SYNC_TASK: RS485 UPDATE TIME FAILED\n");
                }
            }
            else if(diff < 0){
                if(rs485_updatetime(rtc_timestamp)!= 0){
                    LOG_INF("TBR_SYNC_TASK: RS485 UPDATE TIME FAILED\n");
                }
            }
        }
		// memset((void *) rx_buf, 0, sizeof(rx_buf)/sizeof(char));

        }
}
