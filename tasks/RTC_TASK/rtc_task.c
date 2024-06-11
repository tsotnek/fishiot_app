#include "rtc_task.h"
#include "leds.h"
#include "rtc.h"
#include <zephyr/logging/log.h>


LOG_MODULE_DECLARE(FishIoT);

extern bool RTC_TIME_SET;
extern struct nrf_modem_gnss_pvt_data_frame pvt_data;
extern k_tid_t tbr_sync_thread_tid;

extern struct k_msgq IoFHEADER_MSG; 
extern struct k_sem mqtt_pub_sem; 

#define NINTPINLOW DT_ALIAS(nintpinlow)
#define TBRPPSpindriver DT_ALIAS(tbrppspin)


extern struct k_sem gnss_thread_sem; 



static struct gpio_callback nInt_cb_data;
static const struct gpio_dt_spec nintpin = GPIO_DT_SPEC_GET(NINTPINLOW, gpios);

static const struct gpio_dt_spec tbr_pps_pin = GPIO_DT_SPEC_GET(TBRPPSpindriver, gpios);



struct k_timer my_timer;
void my_expiry_function(struct k_timer *timer_id){
	gpio_pin_set_dt(&tbr_pps_pin,0);
	k_timer_stop(&my_timer);

}

void nint_interrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	//interrupt occured
	//toggle the PPS
	gpio_pin_set_dt(&tbr_pps_pin,1);
	k_timer_start(&my_timer, K_MSEC(100), K_FOREVER);

	// LOG_INF("interrupt occured\n");
	// rtc_i2c_write(RTC_Status, 2);
}


// extern struct k_sem rtc_write_fix_sem; 
extern struct k_sem rtc_esyn_sem; 

extern k_tid_t gnss_thread_tid;

static uint8_t rtc_init_interrupt(void){

	uint8_t ret;

	//initialize interrupt pin
	if (!gpio_is_ready_dt(&nintpin)) {
		LOG_ERR("RTC_THREAD: Error initalizing interrupt pin\n");
		return 1;
	}

	ret = gpio_pin_configure_dt(&nintpin, GPIO_INPUT);
	if (ret < 0) {
		return 1;
	}

	ret = gpio_pin_interrupt_configure_dt(&nintpin, GPIO_INT_EDGE_TO_ACTIVE);

	
    gpio_init_callback(&nInt_cb_data, nint_interrupt, BIT(nintpin.pin)); 	

	gpio_add_callback(nintpin.port, &nInt_cb_data);

	//initialize TBR pps output
	if (!gpio_is_ready_dt(&tbr_pps_pin)) {
		return -1;
	}

	ret = gpio_pin_configure_dt(&tbr_pps_pin, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return -1;
	}
	//USEL bit is set to 0 - interrupt on second update
	rtc_i2c_write(RTC_Status, 2);
	//SET UIE bit to 1 to enable interrupt
	rtc_i2c_write(RTC_Control_2, RTC_Control_2_UIE);

	k_timer_init(&my_timer, my_expiry_function, NULL);

	return 0;
}
//thread
void rtc_thread(void *, void *, void *){


	
	rtc_init_interrupt();
	
	// k_sem_take(&rtc_write_fix_sem, K_FOREVER);
	LOG_INF("RTC_THREAD: Writing NAV time and data");
	//write time from GNSS to RTC
	k_sleep(K_MSEC(110)); //Ensure GPS tp is low
	rtc_sync_nav_second();	
	rtc_write_fix_data_first(&pvt_data);
	rtc_sync_nav_second();	
	RTC_TIME_SET = true;
	k_thread_start(tbr_sync_thread_tid);
	k_thread_start(gnss_thread_tid);
	k_sem_give(&gnss_thread_sem);

	while(1)
	{
		k_sem_take(&rtc_esyn_sem, K_FOREVER);
		k_sleep(K_MSEC(110)); //Ensure GPS tp is low
		if(rtc_sync_nav_second()!=0)
		{
			if(k_msgq_num_used_get(&IoFHEADER_MSG) != 0)
				k_sem_give(&mqtt_pub_sem);
			else
				{
					LOG_ERR("RTC_THREAD: Restarting due to RTC...\n");
					k_sleep(K_MSEC(500));
					NVIC_SystemReset();
				}
		}
		else{
			if(rtc_read_time_data()!=0){
				LOG_ERR("RTC_THREAD: Reading of time from RTC failed...\n");
			}
			if(pvt_data.datetime.seconds != rtc_time.seconds){
				k_sleep(K_MSEC(110)); //Ensure GPS tp is low
				rtc_sync_nav_second();
				rtc_write_fix_data_first(&pvt_data);
				rtc_sync_nav_second();
			}
		}
		k_sem_give(&gnss_thread_sem);

	}
}
