#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "rs485.h"
#include "mqtt_task.h"
#include "leds.h"
#include "structs.h"
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/sys_heap.h>

LOG_MODULE_DECLARE(FishIoT);

extern struct k_msgq IoFHEADER_MSG; 
extern struct k_msgq IoFTBR_Status_MSG; 
extern struct k_msgq IoFTBR_TAG_MSG; 
extern struct k_msgq IoFBuoy_Status_MSG; 

extern struct k_sem mqtt_pub_thread_start; 
extern struct k_sem mqtt_pub_sem; 
extern struct k_sem mqtt_pub_done_sem; 
extern struct k_sem mqtt_lock_mutex; 
extern struct k_sem gnss_lock_mutex;


extern struct k_sem modem_fault_sem;
extern bool modem_fault;
extern struct k_sem block_mqtt_on_fault_sem;
//MQTT
/* The mqtt client struct */
static struct mqtt_client client;
/* File descriptor */
static struct pollfd fds;


uint8_t mqttmessageformat[15] = {0};
uint8_t mqttmessagelen;

extern struct k_heap _system_heap;
struct sys_memory_stats stats;
void rtos_hal_print_heap_info(void) 
{
    sys_heap_runtime_stats_get(&_system_heap.heap, &stats);

    LOG_INF("\nINFO: Allocated Heap = %zu\n", stats.allocated_bytes);
    LOG_INF("INFO: Free Heap = %zu\n", stats.free_bytes);
    LOG_INF("INFO: Max Allocated Heap = %zu\n", stats.max_allocated_bytes);

    return;
}
//helper function
static uint8_t formatmessage(void){
	if(k_msgq_num_used_get(&IoFHEADER_MSG)==0){
		return 0;
	}
	IoF_header iofheader;
	if(k_msgq_get(&IoFHEADER_MSG, &iofheader, K_FOREVER)!=0){
		LOG_ERR("MQTT_TASK: Couldn't get message from IoFHEADER_MSG queue\n");
		return 0;
	}

	mqttmessageformat[0] = (uint8_t)(iofheader.TBRserial >> 6);
	mqttmessageformat[1] = (uint8_t)(((iofheader.TBRserial & 0x3f) << 2) | iofheader.headerflag);

	mqttmessageformat[2] = (uint8_t)(iofheader.reftimestamp>>24);
	mqttmessageformat[3] = (uint8_t)(iofheader.reftimestamp>>16);
	mqttmessageformat[4] = (uint8_t)(iofheader.reftimestamp>>8);
	mqttmessageformat[5] = (uint8_t)(iofheader.reftimestamp);
	mqttmessagelen = 6;

	switch(iofheader.headerflag){
		case TBR_status_or_tag:
			//check which message was received
			
			if(iofheader.Tbr_message_type == 255)
			{
				IoF_TBR_status status;
				if(k_msgq_get(&IoFTBR_Status_MSG, &status, K_FOREVER)!=0){
					LOG_ERR("MQTT_TASK: Couldn't get message from IoFTBR_Status_MSG queue\n");
				}
				mqttmessageformat[6] = status.secsince_timestamp;
				mqttmessageformat[7] = status.code_type;
				mqttmessageformat[8] = (uint8_t)(status.temperature>>8);
				mqttmessageformat[9] = (uint8_t)status.temperature;
				mqttmessageformat[10] = status.noise_ave;
				mqttmessageformat[11] = status.noise_peak;
				mqttmessageformat[12] = status.snr_detection;
				mqttmessageformat[13] = status.upper_timing_err;
				mqttmessagelen = 14;
				LOG_INF("Transmitting TBR Status\n");

			}
			else{
				IoF_TBR_tag tag;
				if(k_msgq_get(&IoFTBR_TAG_MSG, &tag, K_FOREVER)!=0){
					LOG_ERR("MQTT_TASK: Couldn't get message from IoFTBR_TAG_MSG queue\n");
				}
				mqttmessageformat[6] = tag.secsince_timestamp;
				mqttmessageformat[7] = tag.code_type;
				
				switch(tag.protocol){ //depeending on code type adjust payload
					case S256: //protocol s256
						mqttmessageformat[8] = (uint8_t)tag.tag_id;
						mqttmessageformat[9] = tag.tag_payload;
						mqttmessageformat[10] = (uint8_t)(((tag.SNR & 0x3f)<<2) | ((tag.milliseconds >> 8)&0x03));
						mqttmessageformat[11] = (uint8_t)(tag.milliseconds & 0xff);
						mqttmessagelen = 12;						
						break;
					case S64K:
					case HS256:
					case DS256:
						mqttmessageformat[8] = (uint8_t)(tag.tag_id >> 8);
						mqttmessageformat[9] = (uint8_t)tag.tag_id;
						mqttmessageformat[10] = tag.tag_payload;
						mqttmessageformat[11] = (uint8_t)(((tag.SNR & 0x3f)<<2) | ((tag.milliseconds >> 8)&0x03));
						mqttmessageformat[12] = (uint8_t)(tag.milliseconds & 0xff);
						mqttmessagelen = 13;
						break;
					case R01M:
						mqttmessageformat[8] = (uint8_t)(tag.tag_id >> 16);
						mqttmessageformat[9] = (uint8_t)(tag.tag_id >> 8);
						mqttmessageformat[10] = (uint8_t)tag.tag_id;
						mqttmessageformat[11] = (uint8_t)(((tag.SNR & 0x3f)<<2) | ((tag.milliseconds >> 8)&0x03));
						mqttmessageformat[12] = (uint8_t)(tag.milliseconds & 0xff);
						mqttmessagelen = 13;
						break;
					case R04K:
					case R64K:
						mqttmessageformat[8] = (uint8_t)(tag.tag_id >> 8);
						mqttmessageformat[9] = (uint8_t)tag.tag_id;
						mqttmessageformat[10] = (uint8_t)(((tag.SNR & 0x3f)<<2) | ((tag.milliseconds >> 8)&0x03));
						mqttmessageformat[11] = (uint8_t)(tag.milliseconds & 0xff);
						mqttmessagelen = 12;						
						break;
					case R256:
						mqttmessageformat[8] = (uint8_t)tag.tag_id;
						mqttmessageformat[9] = (uint8_t)(((tag.SNR & 0x3f)<<2) | ((tag.milliseconds >> 8)&0x03));
						mqttmessageformat[10] = (uint8_t)(tag.milliseconds & 0xff);
						mqttmessagelen = 11;						
						break;
					default:
						break;
				}
				LOG_INF("Transmitting Tag Status\n");

			}
			break;


		case buoy_status:

			IoF_bouy_status bouy;
			if(k_msgq_get(&IoFBuoy_Status_MSG, &bouy, K_FOREVER)!=0){
				LOG_ERR("MQTT_TASK: Couldn't get message from IoFBuoy_Status_MSG queue\n");
			}
			mqttmessageformat[6] = (uint8_t)(((bouy.batvoltage & 0x7F) << 1) | ((bouy.airtemp >> 6) & 0b01));
			mqttmessageformat[7] = (uint8_t)(((bouy.airtemp & 0x3f) << 2) | ((bouy.longitude >> 25) & 0b11));
			mqttmessageformat[8] = (uint8_t)((bouy.longitude >> 17) & 0xff);
			mqttmessageformat[9] = (uint8_t)((bouy.longitude >> 9) & 0xff);
			mqttmessageformat[10] = (uint8_t)((bouy.longitude >> 1) & 0xff);
			mqttmessageformat[11] = (uint8_t)((bouy.PDOP & 0xfe) | ((bouy.latitude >> 29) & 0x01));
			mqttmessageformat[12] = (uint8_t)((bouy.latitude >> 21) & 0xff);
			mqttmessageformat[13] = (uint8_t)((bouy.latitude >> 13) & 0xff);
			mqttmessageformat[14] = (uint8_t)((bouy.latitude >> 5) & 0xff);
			mqttmessageformat[15] = (uint8_t)(((bouy.fix & 0x07) << 5) | (bouy.num_of_sattelites & 0x1f));
			mqttmessagelen = 16;
			LOG_INF("Transmitting Bouy Status\n");

			break;

		
		default: 
			break;
	}
	return 1;
}

void mqtt_thread(void *, void *, void *){
	
	// k_sem_take(&mqtt_pub_thread_start, K_FOREVER);
	
	int err;
	err = client_init(&client);
	if (err) {
		LOG_ERR("Failed to initialize MQTT client: %d", err);
	}
	int connect_attempt = 0;
	int total_connect_attempt = 0;
	for(;;){
	k_sem_take(&mqtt_pub_sem, K_FOREVER);
	k_sem_take(&mqtt_lock_mutex,K_FOREVER);
	k_sem_take(&gnss_lock_mutex,K_NO_WAIT);
	// thread_analyzer_print();
do_connect:
	if (connect_attempt > 0) {
		LOG_INF("Reconnecting in %d seconds...",
			CONFIG_MQTT_RECONNECT_DELAY_S);
		// irq_unlock(irqkey);
		total_connect_attempt++;
		k_sleep(K_SECONDS(CONFIG_MQTT_RECONNECT_DELAY_S));
		connect_attempt = 0;
		if(modem_fault)
		{	
			k_sem_take(&block_mqtt_on_fault_sem, K_FOREVER);
			modem_fault=false;
		}
		else if(total_connect_attempt>=3)
		{
			k_sem_give(&modem_fault_sem);
			k_sleep(K_SECONDS(CONFIG_MQTT_RECONNECT_DELAY_S));
			k_sem_take(&block_mqtt_on_fault_sem, K_FOREVER);
			total_connect_attempt=0;
		}
	}
	// irqkey = irq_lock();
	
	err = mqtt_connect(&client);
	if (err) {
		LOG_ERR("Error in mqtt_connect: %d", err);
		LED_ERROR_CODE(MQTT_CONNECT_ERROR);
		connect_attempt++;
		goto do_connect;
	}
	// rtos_hal_print_heap_info();

	err = fds_init(&client,&fds);
	if (err) {
		LOG_ERR("Error in fds_init: %d", err);
	}

	err = poll(&fds, 1, mqtt_keepalive_time_left(&client));
	if (err < 0) {
		LOG_ERR("Error in poll(): %d", errno);
		connect_attempt++;
		err = mqtt_disconnect(&client);
		if (err) {
			LOG_ERR("Could not disconnect MQTT client: %d", err);
		}
		goto do_connect;

	}	
	
	err = mqtt_live(&client);
	if ((err != 0) && (err != -EAGAIN)) {
		LED_ERROR_CODE(MQTT_LIVE_ERROR);
		LOG_ERR("Error in mqtt_live: %d", err);
		connect_attempt++;
		err = mqtt_disconnect(&client);
		if (err) {
			LOG_ERR("Could not disconnect MQTT client: %d", err);
		}
		goto do_connect;

	}

	if ((fds.revents & POLLIN) == POLLIN) {
		err = mqtt_input(&client);
		if (err != 0) {
			LED_ERROR_CODE(MQTT_INPUT_ERROR);
			LOG_ERR("Error in mqtt_input: %d", err);
			connect_attempt++;
			err = mqtt_disconnect(&client);
			if (err) {
				LOG_ERR("Could not disconnect MQTT client: %d", err);
			}
			goto do_connect;
		}
	}

	if ((fds.revents & POLLERR) == POLLERR) {
		connect_attempt++;
		LOG_ERR("POLLERR");
		err = mqtt_disconnect(&client);
			if (err) {
				LOG_ERR("Could not disconnect MQTT client: %d", err);
			}

		goto do_connect;
	}

	if ((fds.revents & POLLNVAL) == POLLNVAL) {
		connect_attempt++;
		LOG_ERR("POLLNVAL");
		err = mqtt_disconnect(&client);
			if (err) {
				LOG_ERR("Could not disconnect MQTT client: %d", err);
			}
		goto do_connect;
	}
	while(formatmessage()){
		
		err = data_publish(&client, MQTT_QOS_1_AT_LEAST_ONCE,mqttmessageformat, mqttmessagelen);
		if (err) {
			LED_ERROR_CODE(MQTT_PUBLISH_ERROR);
			LOG_INF("Failed to send message, %d", err);
			connect_attempt++;
			break;
		}
		memset((void *) mqttmessageformat, 0, sizeof(mqttmessageformat)/sizeof(uint8_t));
		memset((void *) &mqttmessagelen, 0, sizeof(uint8_t));
	
	}

	LOG_INF("Disconnecting MQTT client");
	err = mqtt_disconnect(&client);
	if (err) {
		LOG_ERR("Could not disconnect MQTT client: %d", err);
	}
	// rtos_hal_print_heap_info();
	// k_sem_give(&mqtt_pub_done_sem);
	k_sem_give(&mqtt_lock_mutex);
	// irq_unlock(irqkey);
	}
}

