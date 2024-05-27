#include "rs485.h"
#include "leds.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


K_SEM_DEFINE(uart_rec_sem, 0, 1);
K_SEM_DEFINE(tbr_sync_task_sem, 0 , 1);
/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);



#define RXLOWpin DT_ALIAS(relowpin)
#define TXHIGHpin DT_ALIAS(dehighpin)

const struct device *uart_dev= DEVICE_DT_GET(DT_NODELABEL(uart2));
static const struct gpio_dt_spec RXpin = GPIO_DT_SPEC_GET(RXLOWpin, gpios);
static const struct gpio_dt_spec TXPin = GPIO_DT_SPEC_GET(TXHIGHpin, gpios);

/* receive buffer used in UART ISR callback */
char rx_buf[MSG_SIZE];
static int rx_buf_pos;

extern uint16_t TBSN; //tbr serial number
static bool enablerecieve; 
extern bool RTC_TIME_SET;

uint8_t rs485_rounduptime(void){
	char *msg_to_send="(+)\r\n";
	print_uart(msg_to_send);
	return 0;
}
uint8_t rs485_updatetime(uint64_t unix_time_ms){
	if(!RTC_TIME_SET)
		unix_time_ms /= 1000; //unix time stamp
	uint64_t time = unix_time_ms;
	uint16_t digitSum = 0;
	uint32_t digit = 0;
	for(uint8_t i = 0; i < 9; i++){
		time /= 10;
		digit = time % 10;
		if ((i % 2 ) == 0){
			digit *= 2;
		}
		if (digit > 9 ){
			digit -= 9;
		}
		digitSum += digit;
	}

	uint8_t luhnsCheckDigit = (digitSum * 9) % 10;

	char* ack_msg = "ack02";
	char rsupdatemessage[17];
	sprintf(rsupdatemessage, "(+)%d%d\r\n", (int)unix_time_ms/10, luhnsCheckDigit);
	if(!RTC_TIME_SET)
		uart_irq_rx_enable(uart_dev);
	print_uart(rsupdatemessage);
	k_busy_wait(100*1000);
	return strcmp(rx_buf, ack_msg);
}

uint8_t rs485_extractserialnnumber(void){
	if(rx_buf[0]!='S'){
		memset((void *) rx_buf, 0, sizeof(rx_buf)/sizeof(char));
		return 1;
	}

	uint8_t RS485_SN[10];
    
    uint8_t i=0;
	while(rx_buf[i]!='>'){
		RS485_SN[i] = rx_buf[i+4];
		i++;
	}

	TBSN = atoi(RS485_SN);

	memset((void *) rx_buf, 0, sizeof(rx_buf)/sizeof(char));
	return 0;
}


/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		return;
	}
	// unsigned int irqkey = irq_lock();

	/* read until FIFO empty */
	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			//place it in queue only if its LOG or Tag detection
			if(strlen(rx_buf)>16 && enablerecieve){
				k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);
				k_sem_give(&tbr_sync_task_sem);
				k_sem_give(&uart_rec_sem);
			}
			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
			//check if the size of messages is greater than 5
				
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			if ((c != '\n') || (c != '\r')){
				rx_buf[rx_buf_pos++] = c;
			}
		}
		/* else: characters beyond buffer size are dropped */
	}
	// irq_unlock(irqkey);

	
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);
	gpio_pin_set_dt(&RXpin, 1);
	gpio_pin_set_dt(&TXPin, 1);
	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
	gpio_pin_set_dt(&TXPin, 0);
	gpio_pin_set_dt(&RXpin, 0);
}

uint8_t rs485_init(void)
{
	enablerecieve = false;
	if (!gpio_is_ready_dt(&RXpin)) {
		return -1;
	}
	if (!gpio_is_ready_dt(&TXPin)) {
		return -1;
	}

    int ret = gpio_pin_configure_dt(&RXpin, GPIO_OUTPUT);
	if (ret < 0) {
		return -1;
	}
	ret = gpio_pin_configure_dt(&TXPin, GPIO_OUTPUT);
	if (ret < 0) {
		return -1;
	}

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return 0;
	}

	
	gpio_pin_set_dt(&TXPin, 0);
	gpio_pin_set_dt(&RXpin, 0);

	/* configure interrupt and callback to receive data */
	ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}
	int retryattempts = 0;
retry:
	if(retryattempts>10){
		printk("Something wrong with TBRLive...\n");
		return 1;
	}
	uart_irq_rx_enable(uart_dev);
	print_uart("?\r\n");

	k_busy_wait(100*1000);

	uart_irq_rx_disable(uart_dev);

	if(rs485_extractserialnnumber()!=0){
		printk("Error in extraction of serial number, retrying...\n");
		k_busy_wait(1*1000*1000);
		retryattempts++;
		goto retry;
	}
	printk("TBR Serial number is %d\n", TBSN);
	enablerecieve = true;
	return 0;
}


