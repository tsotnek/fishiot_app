#include "rtc_task.h"
#include "leds.h"
#include "rtc.h"
#include <zephyr/logging/log.h>


LOG_MODULE_DECLARE(FishIoT);

extern struct nrf_modem_gnss_pvt_data_frame pvt_data;


extern struct k_sem rtc_write_fix_sem; 
extern struct k_sem rtc_esyn_sem; 

//thread
void rtc_thread(void){

	k_sem_take(&rtc_write_fix_sem, K_FOREVER);
	LOG_INF("Writing NAV time and data");
	//write time from GNSS to RTC
	rtc_write_fix_data_first(&pvt_data);
	while(1){
		k_sem_take(&rtc_esyn_sem, K_FOREVER);
		rtc_sync_nav_second();
	}

}
