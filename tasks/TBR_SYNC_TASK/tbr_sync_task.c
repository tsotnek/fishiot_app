#include "tbr_sync_task.h"
#include <zephyr/logging/log.h>
#include "leds.h"
#include "rs485.h"
#include "rtc.h"
#include <time.h>
#include <stdlib.h>

LOG_MODULE_DECLARE(FishIoT);

extern bool RTC_TIME_SET;

static uint32_t tbr_sync_rtc_read_time(void){
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
    return (uint32_t) rtc_time_utc;
}

void tbr_sync_thread(void *, void *, void *){
    char rx_buffer[MSG_SIZE];
	const char s[2]= ",";
    char *token;
    const char *word = "TBR Sensor";
    int cntr = 0;
    uint32_t rtc_timestamp = tbr_sync_rtc_read_time();
    k_sleep(K_MSEC(1000-rtc_time.sec100));
    rs485_updatetime(rtc_timestamp+1);
    for(;;){
        k_sem_take(&tbr_sync_task_sem, K_FOREVER);
        

        if(RTC_TIME_SET){
            //read time from RTC
			// LOG_INF("uptime seconds in TBRThread before reading rtc: %lld", k_uptime_get());

            // uint32_t rtc_timestamp = tbr_sync_rtc_read_time();
			// // LOG_INF("uptime seconds in TBRThread after reading rtc: %lld", k_uptime_get());
            // if(rs485_readtbrtime_commandmode(rtc_timestamp)!= 0){
            //     LOG_ERR("TBR_SYNC_THREAD: Couldn't syncrhonize time...\n");
            // }
            cntr++;
            strcpy(rx_buffer, rx_buf);
            token = strtok(rx_buffer, s); //garbage
            uint32_t tbr_timestamp = atoi(strtok(NULL, s)); 
            rtc_timestamp = tbr_sync_rtc_read_time(); //-10 since tblive sends messages after 10.5s delay
            if(cntr>50){
                k_sleep(K_MSEC(1000-rtc_time.sec100));
                rs485_updatetime(rtc_timestamp+1);
                cntr = 0;
            }
            
           
            
            LOG_INF("TBR_SYNC_TASK: value for tbr_timestamp is: %d, value for rtc_timestamp is: %d\n", tbr_timestamp, rtc_timestamp);
            
            // int diff = rtc_timestamp-tbr_timestamp;
            // k_sleep(K_MSEC(9900));
            
            
        }

        }
}
