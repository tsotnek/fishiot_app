#ifndef _RS485_H_
#define _RS485_H_
#include <stdint.h>
#include <zephyr/drivers/uart.h>

uint8_t rs485_init(void);
uint8_t rs485_transmit(void);



#endif