#ifndef _ADC_H_
#define _ADC_H_
#include <stdint.h>

uint8_t adc_init(void);
uint32_t adc_read_voltage(void);

#endif