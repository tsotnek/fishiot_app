#ifndef _RS485_TASK_H_
#define _RS485_TASK_H_
#include <stdint.h>
extern uint8_t rs485_init(void);
extern uint8_t rs485_updatetime(uint64_t unix_time_ms);
void rs485_thread(void);

#endif
