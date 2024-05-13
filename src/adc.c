#include "adc.h"
#include "leds.h"
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(FishIoT);


static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));


int16_t buf;
struct adc_sequence sequence = {
	.buffer = &buf,
	/* buffer size in bytes, not number of samples */
	.buffer_size = sizeof(buf),
	//Optional
	//.calibrate = true,
};

uint8_t adc_init(void){

    int err;

    if (!adc_is_ready_dt(&adc_channel)) {
		LOG_ERR("ADC controller devivce %s not ready", adc_channel.dev->name);
		return 1;
	}

	err = adc_channel_setup_dt(&adc_channel);
	if (err < 0) {
		LOG_ERR("Could not setup channel #%d (%d)", 0, err);
		return 1;
	}

	err = adc_sequence_init_dt(&adc_channel, &sequence);
	if (err < 0) {
		LOG_ERR("Could not initalize sequnce");
		return 1;
	}
    return 0;
}


uint32_t adc_read_voltage(void)
{
    int err;
    err = adc_read(adc_channel.dev, &sequence);
    if (err < 0) {
        LOG_ERR("Could not read (%d)", err);
        // return 1;
    }

    uint32_t val_mv = (uint32_t)buf;
    /* Convert raw value to mV*/
    err = adc_raw_to_millivolts_dt(&adc_channel, &val_mv);
    val_mv *= 3.4;
   
    return val_mv;

}