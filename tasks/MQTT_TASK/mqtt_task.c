#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "mqtt_connection.h"
#include "rs485.h"
#include "mqtt_task.h"
#include "leds.h"
#include "structs.h"

LOG_MODULE_DECLARE(FishIoT);

extern struct k_msgq IoFHEADER_MSG; 
extern struct k_msgq IoFTBR_Status_MSG; 
extern struct k_msgq IoFTBR_TAG_MSG; 
extern struct k_msgq IoFBuoy_Status_MSG; 

extern struct k_sem mqtt_pub_thread_start; 
extern struct k_sem mqtt_pub_sem; 
extern struct k_sem mqtt_pub_done_sem; 
extern struct k_sem mqtt_lock_mutex; 

void mqtt_thread(void);


//MQTT
/* The mqtt client struct */
static struct mqtt_client client;
/* File descriptor */
static struct pollfd fds;


uint8_t mqttmessageformat[15] = {0};
uint8_t mqttmessagelen;
//helper function
static void formatmessage(void){
	IoF_header iofheader;
	if(k_msgq_get(&IoFHEADER_MSG, &iofheader, K_FOREVER)!=0){
		LOG_ERR("MQTT_TASK: Couldn't get message from IoFHEADER_MSG queue\n");
	}

	mqttmessageformat[0] = (uint8_t)iofheader.TBRserial_and_headerflag;
	mqttmessageformat[1] = (uint8_t)(iofheader.TBRserial_and_headerflag>>8);

	mqttmessageformat[2] = (uint8_t)iofheader.reftimestamp;
	mqttmessageformat[3] = (uint8_t)(iofheader.reftimestamp>>8);
	mqttmessageformat[4] = (uint8_t)(iofheader.reftimestamp>>16);
	mqttmessageformat[5] = (uint8_t)(iofheader.reftimestamp>>24);
	mqttmessagelen = 6;

	switch((iofheader.TBRserial_and_headerflag & 0xC000)>>14){
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
				mqttmessageformat[8] = (uint8_t)status.temperature;
				mqttmessageformat[9] = (uint8_t)(status.temperature>>8);
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
				mqttmessageformat[8] = (uint8_t)tag.tag_id;
				switch(tag.protocol){ //depeending on code type adjust payload
					case S256: //protocol s256
						mqttmessageformat[9] = tag.tag_payload;
						mqttmessageformat[10] = (uint8_t)tag.SNR_milliseconds;
						mqttmessageformat[11] = (uint8_t)(tag.SNR_milliseconds>>8);
						mqttmessagelen = 12;						
						break;
					case S64K:
					case HS256:
					case DS256:
						mqttmessageformat[9] = (uint8_t)(tag.tag_id >> 8);
						mqttmessageformat[10] = tag.tag_payload;
						mqttmessageformat[11] = (uint8_t)tag.SNR_milliseconds;
						mqttmessageformat[12] = (uint8_t)(tag.SNR_milliseconds>>8);
						mqttmessagelen = 13;
						break;
					case R01M:
						mqttmessageformat[9] = (uint8_t)(tag.tag_id >> 8);
						mqttmessageformat[10] = (uint8_t)(tag.tag_id >> 16);
						mqttmessageformat[11] = (uint8_t)tag.SNR_milliseconds;
						mqttmessageformat[12] = (uint8_t)(tag.SNR_milliseconds>>8);
						mqttmessagelen = 13;
						break;
					case R04K:
					case R64K:
						mqttmessageformat[9] = (uint8_t)(tag.tag_id >> 8);
						mqttmessageformat[10] = (uint8_t)tag.SNR_milliseconds;
						mqttmessageformat[11] = (uint8_t)(tag.SNR_milliseconds>>8);
						mqttmessagelen = 12;						
						break;
					case R256:
						mqttmessageformat[9] = (uint8_t)tag.SNR_milliseconds;
						mqttmessageformat[10] = (uint8_t)(tag.SNR_milliseconds>>8);
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
			mqttmessageformat[6] = (uint8_t)bouy.batvolatge_airtemp_lon;
			mqttmessageformat[7] = (uint8_t)(bouy.batvolatge_airtemp_lon>>8);
			mqttmessageformat[8] = (uint8_t)bouy.longitude;
			mqttmessageformat[9] = (uint8_t)(bouy.longitude>>8);
			mqttmessageformat[10] = bouy.longitude_cont;
			mqttmessageformat[11] = bouy.PDOP_lat;
			mqttmessageformat[12] = (uint8_t)bouy.latitude;
			mqttmessageformat[13] = (uint8_t)(bouy.latitude>>8);
			mqttmessageformat[14] = bouy.latitude_cont;
			mqttmessageformat[15] = bouy.fix_num_of_satelites;
			mqttmessagelen = 16;
			LOG_INF("Transmitting Bouy Status\n");

			break;

		
		default: 
			break;
	}
}

void mqtt_thread(void){
	
	k_sem_take(&mqtt_pub_thread_start, K_FOREVER);
	
	int err;
	err = client_init(&client);
	if (err) {
		LOG_ERR("Failed to initialize MQTT client: %d", err);
	}
	
	for(;;){
	k_sem_take(&mqtt_pub_sem, K_FOREVER);
	k_sem_take(&mqtt_lock_mutex,K_FOREVER);
	unsigned int irqkey = irq_lock();
	formatmessage();
do_connect:

	err = mqtt_connect(&client);
	if (err) {
		LOG_ERR("Error in mqtt_connect: %d", err);
		LED_ERROR_CODE(MQTT_CONNECT_ERROR);
		goto do_connect;
	}

	err = fds_init(&client,&fds);
	if (err) {
		LOG_ERR("Error in fds_init: %d", err);
	}

	err = poll(&fds, 1, mqtt_keepalive_time_left(&client));
	if (err < 0) {
		LOG_ERR("Error in poll(): %d", errno);
	}
		
		
	err = mqtt_live(&client);
	if ((err != 0) && (err != -EAGAIN)) {
		LED_ERROR_CODE(MQTT_LIVE_ERROR);
		LOG_ERR("Error in mqtt_live: %d", err);
	}

	if ((fds.revents & POLLIN) == POLLIN) {
		err = mqtt_input(&client);
		if (err != 0) {
			LED_ERROR_CODE(MQTT_INPUT_ERROR);
			LOG_ERR("Error in mqtt_input: %d", err);
		}
	}

	if ((fds.revents & POLLERR) == POLLERR) {
		LOG_ERR("POLLERR");
	}

	if ((fds.revents & POLLNVAL) == POLLNVAL) {
		LOG_ERR("POLLNVAL");
	}
	// data_formatter(&pvt_data);
	err = data_publish(&client, MQTT_QOS_1_AT_LEAST_ONCE,
		mqttmessageformat, mqttmessagelen);
	if (err) {
		LED_ERROR_CODE(MQTT_PUBLISH_ERROR);
		LOG_INF("Failed to send message, %d", err);
	}
	memset((void *) mqttmessageformat, 0, sizeof(mqttmessageformat)/sizeof(uint8_t));
	memset((void *) &mqttmessagelen, 0, sizeof(uint8_t));

	LOG_INF("Disconnecting MQTT client");

	err = mqtt_disconnect(&client);
	if (err) {
		LOG_ERR("Could not disconnect MQTT client: %d", err);
	}
	// k_sem_give(&mqtt_pub_done_sem);
	k_sem_give(&mqtt_lock_mutex);
	irq_unlock(irqkey);
	}
}

