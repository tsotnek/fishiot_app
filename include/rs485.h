#ifndef _RS485_H_
#define _RS485_H_
#include <stdint.h>
#include <zephyr/drivers/uart.h>

#define MSG_SIZE 60

extern char rx_buf[MSG_SIZE];
extern struct k_sem uart_rec_sem; 
extern uint16_t TBSN; //tbr serial number


uint8_t rs485_init(void);
uint8_t rs485_transmit(void);
uint8_t rs485_extractserialnnumber(void);
uint8_t rs485_updatetime(uint64_t unix_time_ms);
void print_uart(char *buf);

#endif