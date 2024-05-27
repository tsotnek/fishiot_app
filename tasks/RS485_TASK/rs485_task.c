#include "rs485_task.h"
#include "rs485.h"
#include "leds.h"
#include "structs.h"
#include <zephyr/logging/log.h>
#include <stdlib.h>

LOG_MODULE_DECLARE(FishIoT);



extern struct k_msgq IoFHEADER_MSG; 
extern struct k_msgq IoFTBR_Status_MSG; 
extern struct k_msgq IoFTBR_TAG_MSG; 

extern struct k_sem mqtt_pub_sem; 


extern uint16_t TBSN; //tbr serial number



static void code_type_function(IoF_TBR_tag* tag, char * protocol, int frequency){
	uint8_t protocol_int = 0;
	if((strncmp(protocol,(char *)"R256", 4) == 0) || (strncmp(protocol, (char *)"r256",4) == 0)){
		protocol_int = R256;
	}
	else if((strncmp(protocol, (char *)"R04K", 4) == 0) || (strncmp(protocol, (char *)"r04k", 4) == 0 ) ||
			(strncmp(protocol, (char *)"R04k", 4) == 0) || (strncmp(protocol, (char *)"r04K", 4) == 0 )) {
		protocol_int = R04K;
	}
	else if((strncmp(protocol, (char *)"R64K", 4) == 0) || (strncmp(protocol, (char *)"r64k", 4) == 0) ||
			(strncmp(protocol, (char *)"R64k", 4) == 0) || (strncmp(protocol, (char *)"r64K", 4) == 0)) {
		protocol_int = R64K;
	}
	else if((strncmp(protocol,(char *)"S256", 4) == 0) || (strncmp(protocol, (char *)"s256", 4) == 0)) {
		protocol_int = S256;
	}
	else if((strncmp(protocol,(char *)"R01M", 4) == 0) || (strncmp(protocol, (char *)"r01m", 4)==0) ||
			(strncmp(protocol,(char *)"R01m", 4) == 0) || (strncmp(protocol, (char *)"r01M", 4)==0)) {
		protocol_int = R01M;
	}
	else if((strncmp(protocol,(char *)"S64K", 4) == 0) || (strncmp(protocol, (char *)"s64k", 4) == 0) ||
			(strncmp(protocol,(char *)"S64k", 4) == 0) || (strncmp(protocol, (char *)"s64K", 4) == 0)) {
		protocol_int = S64K;
	}
	else if((strncmp(protocol,(char *)"HS256", 4) == 0) || (strncmp(protocol, (char *)"hs256", 4) == 0) ||
			(strncmp(protocol,(char *)"Hs256", 4) == 0) || (strncmp(protocol, (char *)"hS256", 4) == 0)) {
		protocol_int = HS256;
	}
	else if((strncmp(protocol,(char *)"DS256", 4) == 0) || (strncmp(protocol, (char *)"ds256", 4) == 0) ||
			(strncmp(protocol,(char *)"Ds256", 4) == 0) || (strncmp(protocol, (char *)"dS256", 4) == 0)) {
		protocol_int = DS256;
	}
	else protocol_int = 0xFE;

	tag->protocol = protocol_int;

	int diff_freq = frequency-69;
	if (frequency > 69) {
		protocol_int = protocol_int+(16*diff_freq);
	}
	else if (frequency<69) {
		protocol_int = (uint8_t)protocol_int+(16*(diff_freq-1));
	}
	else {		//in case of 69 KHz do not change code type
	}

	tag->code_type = protocol_int;

}



void rs485_thread(void *, void *, void *){
	//wait until the message was received on RS485
	const char *word = "TBR Sensor";
	const char s[2]= ",";
	char *token;
	char arr[9][25]; 
	uint8_t rx_buf_temp[60];
	uint8_t rx_cpy[60];
	for(;;){
		k_sem_take(&uart_rec_sem, K_FOREVER);

		if(k_msgq_get(&uart_msgq, rx_buf_temp, K_FOREVER)!=0){
			LOG_ERR("RS485_THREAD: Couldn't get receieved UART Message!\n");
		}
		strcpy(rx_cpy, rx_buf_temp);
		IoF_header header;
		header.TBRserial = TBSN;
		header.headerflag = TBR_status_or_tag;
		
		/* get the first token */
		token = strtok(rx_buf_temp, s);
		int i =0;
		/* walk through other tokens */
		while( token != NULL ) {
			strcpy(arr[i],token);
			i++;
			token = strtok(NULL, s);
		}
		header.reftimestamp = atoi(arr[1]);

		
		if(strstr(rx_cpy,word)){
			//log message
			header.Tbr_message_type = 255; //log message
			//check the number of messages in queue
			if(k_msgq_num_used_get(&IoFHEADER_MSG) >= 16){
				//purge oldest value
				LOG_INF("RS485_THREAD: IoFHeaderMSG queue is full... purging\n");
				if(k_msgq_get(&IoFHEADER_MSG, NULL, K_FOREVER) != 0){
					LOG_INF("RS485_THREAD: IoFHeaderMSG queue couldn't be purged\n");
				}
			}
			if(k_msgq_put(&IoFHEADER_MSG, &header, K_FOREVER)!=0){
				LOG_INF("RS485_THREAD: Message couldn't be placed in IoFHeaderMSG queue\n");
			}
			IoF_TBR_status status;
			status.secsince_timestamp = 0;
			status.code_type = 255; //code type is fixed to 255 in the TB status frame
			status.temperature = atoi(arr[3]);
			status.noise_ave = atoi(arr[4]);
			status.noise_peak = atoi(arr[5]);
			status.snr_detection = atoi(arr[6]);
			status.upper_timing_err = 0xcc;//reserved for upper accuracy limit
			//check the number of messages in queue
			if(k_msgq_num_used_get(&IoFTBR_Status_MSG) >= 16){
				//purge oldest value
				LOG_INF("RS485_THREAD: IoFTBR_Status_MSG queue is full... purging\n");
				if(k_msgq_get(&IoFTBR_Status_MSG, NULL, K_FOREVER) != 0){
					LOG_INF("RS485_THREAD: IoFTBR_Status_MSG queue couldn't be purged\n");
				}
			}
			if(k_msgq_put(&IoFTBR_Status_MSG, &status, K_FOREVER)!=0){
				LOG_INF("RS485_THREAD: Message couldn't be placed in IoFTBR_Status_MSG queue\n");
			}
			LOG_INF("Received LOG Message on TBR: %s", rx_cpy);
		}
		else{
			//tag detection
			header.Tbr_message_type = 0; //tag message
			
			if(k_msgq_num_used_get(&IoFHEADER_MSG) >= 16){
				//purge oldest value
				LOG_INF("RS485_THREAD: IoFHeaderMSG queue is full... purging\n");
				if(k_msgq_get(&IoFHEADER_MSG, NULL, K_FOREVER) != 0){
					LOG_INF("RS485_THREAD: IoFHeaderMSG queue couldn't be purged\n");
				}
			}
			if(k_msgq_put(&IoFHEADER_MSG, &header, K_FOREVER)!=0){
				LOG_INF("RS485_THREAD: Message couldn't be placed in IoFHeaderMSG queue\n");
			}
			IoF_TBR_tag tag;
			tag.secsince_timestamp = 0;
			
			code_type_function(&tag, arr[3], atoi(arr[7])); //write code_type
			tag.tag_id = atoi(arr[4]);
			tag.tag_payload = atoi(arr[5]);
			tag.SNR = (uint8_t)atoi(arr[6]);
			tag.milliseconds = (uint16_t)atoi(arr[2]);
			
			if(k_msgq_num_used_get(&IoFTBR_TAG_MSG) >= 16){
				//purge oldest value
				LOG_INF("RS485_THREAD: IoFTBR_TAG_MSG queue is full... purging\n");
				if(k_msgq_get(&IoFTBR_TAG_MSG, NULL, K_FOREVER) != 0){
					LOG_INF("RS485_THREAD: IoFTBR_TAG_MSG queue couldn't be purged\n");
				}
			}
			if(k_msgq_put(&IoFTBR_TAG_MSG, &tag, K_FOREVER)!=0){
				LOG_INF("RS485_THREAD: Message couldn't be placed in IoFTBR_TAG_MSG queue\n");
			}
			LOG_INF("Detected FISH on TBR: %s", rx_cpy);
		}
		k_sem_give(&mqtt_pub_sem);
	}
}