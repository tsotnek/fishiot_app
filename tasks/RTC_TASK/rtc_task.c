#include "rtc_task.h"
#include "leds.h"
#include "rtc.h"
#include <zephyr/logging/log.h>


LOG_MODULE_DECLARE(FishIoT);

extern bool RTC_TIME_SET;
extern struct nrf_modem_gnss_pvt_data_frame pvt_data;


#define NINTPINLOW DT_ALIAS(nintpinlow)
#define TBRPPSpindriver DT_ALIAS(tbrppspin)


static struct gpio_callback nInt_cb_data;
static const struct gpio_dt_spec nintpin = GPIO_DT_SPEC_GET(NINTPINLOW, gpios);

static const struct gpio_dt_spec tbr_pps_pin = GPIO_DT_SPEC_GET(TBRPPSpindriver, gpios);


void nint_interrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	//interrupt occured
	//toggle the PPS
	gpio_pin_toggle_dt(&tbr_pps_pin);
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

	ret = gpio_pin_interrupt_configure_dt(&nintpin, GPIO_INT_EDGE_BOTH);

	
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
	return 0;
}
//thread
void rtc_thread(void *, void *, void *){


	// struct nrf_modem_gnss_pvt_data_frame pvt_dataaa;
	// pvt_dataaa.datetime.year = 2024;
	// pvt_dataaa.datetime.month = 5;
	// pvt_dataaa.datetime.day = 26;
	// pvt_dataaa.datetime.hour = 18;
	// pvt_dataaa.datetime.minute = 39;
	// pvt_dataaa.datetime.seconds = 30;
	// pvt_dataaa.datetime.ms = 30;

	// rtc_write_fix_data_first(&pvt_dataaa);

	rtc_init_interrupt();
	
	// k_sem_take(&rtc_write_fix_sem, K_FOREVER);
	LOG_INF("RTC_THREAD: Writing NAV time and data");
	//write time from GNSS to RTC
	rtc_write_fix_data_first(&pvt_data);
	RTC_TIME_SET = true;
	k_thread_start(gnss_thread_tid);
	while(1)
	{
		k_sem_take(&rtc_esyn_sem, K_FOREVER);
		rtc_sync_nav_second();		
	}
}
