#ifndef _RS485_H_
#define _RS485_H_
#include <stdint.h>
#define MSG_SIZE 60

extern struct k_msgq uart_msgq;
extern struct k_sem uart_rec_sem; 
extern struct k_sem tbr_sync_task_sem;

extern char rx_buf[MSG_SIZE];

typedef enum
{
    R256,
    R04K,
    R64K,
    S256,
    R01M,
    S64K,
    HS256,
    DS256

} TBR_protocols;

uint8_t rs485_rounduptime(void);

uint8_t rs485_init(void);
uint8_t rs485_transmit(void);
uint8_t rs485_extractserialnnumber(void);
uint8_t rs485_updatetime(uint64_t unix_time_ms);
void print_uart(char *buf);

#endif