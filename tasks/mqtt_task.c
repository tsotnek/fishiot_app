#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "mqtt_connection.h"
#include "rs485.h"
#include "mqtt_task.h"

LOG_MODULE_DECLARE(FishIoT);

extern struct k_msgq IoFHEADER_MSG; 
extern struct k_msgq IoFTBR_Status_MSG; 
extern struct k_msgq IoFTBR_TAG_MSG; 

K_SEM_DEFINE(mqtt_pub_sem, 0, 1);
K_SEM_DEFINE(mqtt_pub_thread_start, 0, 1);
K_SEM_DEFINE(mqtt_pub_done_sem, 0, 1);

void mqtt_thread(void);


//MQTT
/* The mqtt client struct */
static struct mqtt_client client;
/* File descriptor */
static struct pollfd fds;


uint8_t mqttmessageformat[15] = {0};
uint8_t mqttmessagelen;
//helper function
void formatmessage(void){
	IoF_header iofheader;
	k_msgq_get(&IoFHEADER_MSG, &iofheader, K_FOREVER);

	mqttmessageformat[0] = (uint8_t)iofheader.TBRserial_and_headerflag;
	mqttmessageformat[1] = (uint8_t)(iofheader.TBRserial_and_headerflag>>8);

	mqttmessageformat[2] = (uint8_t)iofheader.reftimestamp;
	mqttmessageformat[3] = (uint8_t)(iofheader.reftimestamp>>8);
	mqttmessageformat[4] = (uint8_t)(iofheader.reftimestamp>>16);
	mqttmessageformat[5] = (uint8_t)(iofheader.reftimestamp>>24);
	mqttmessagelen = 6;

	printk("Value of tbrserial_headerflag is: %d", iofheader.TBRserial_and_headerflag);
	switch(iofheader.TBRserial_and_headerflag & 0xC000){
		case TBR_status_or_tag:
			//check which message was received
			
			if(iofheader.Tbr_message_type == 255)
			{
				IoF_TBR_status status;
				k_msgq_get(&IoFTBR_Status_MSG, &status, K_FOREVER);
				mqttmessageformat[6] = status.secsince_timestamp;
				mqttmessageformat[7] = status.code_type;
				mqttmessageformat[8] = (uint8_t)status.temperature;
				mqttmessageformat[9] = (uint8_t)(status.temperature>>8);
				mqttmessageformat[10] = status.noise_ave;
				mqttmessageformat[11] = status.noise_peak;
				mqttmessageformat[12] = status.snr_detection;
				mqttmessageformat[13] = status.upper_timing_err;
				mqttmessagelen = 14;
			}
			else{
				IoF_TBR_tag tag;
				k_msgq_get(&IoFTBR_TAG_MSG, &tag, K_FOREVER);
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
			}
			break;


		case buoy_status:

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
	formatmessage();
do_connect:

	err = mqtt_connect(&client);
	if (err) {
		LOG_ERR("Error in mqtt_connect: %d", err);
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
		LOG_ERR("Error in mqtt_live: %d", err);
	}

	if ((fds.revents & POLLIN) == POLLIN) {
		err = mqtt_input(&client);
		if (err != 0) {
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
		LOG_INF("Failed to send message, %d", err);
	}
	memset((void *) mqttmessageformat, 0, sizeof(mqttmessageformat)/sizeof(uint8_t));
	memset((void *) &mqttmessagelen, 0, sizeof(uint8_t));

	LOG_INF("Disconnecting MQTT client");

	err = mqtt_disconnect(&client);
	if (err) {
		LOG_ERR("Could not disconnect MQTT client: %d", err);
	}
	k_sem_give(&mqtt_pub_done_sem);

	}
}

