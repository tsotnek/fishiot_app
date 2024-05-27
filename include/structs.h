#ifndef _SEMAPHORES_STRUCTS_H_
#define _SEMAPHORES_STRUCTS_H_
#include <stdint.h>


#define BAT_VOLTAGE_NOMINAL_MV 3600
typedef enum{
	TBR_status_or_tag,
	buoy_status,
	GPS_cycle
} IoF_header_flags;

typedef struct{
	uint16_t TBRserial; //TBR serial[0:13] and header flag[14:15]
	uint8_t headerflag;
	uint32_t reftimestamp; //Reference timestamp (UTC)
	uint8_t Tbr_message_type;
} IoF_header;


typedef struct{
	uint8_t secsince_timestamp;
	uint8_t code_type;
	uint16_t temperature;
	uint8_t noise_ave;
	uint8_t noise_peak;
	uint8_t snr_detection; 
	uint8_t upper_timing_err;
} IoF_TBR_status;

typedef struct{
	uint8_t secsince_timestamp;
	uint8_t code_type;
	uint32_t tag_id; //allprotocols
	uint8_t tag_payload; //Tag ID (protocol: R04K, R64K, R01M, S64K, HS256, DS256)
 							//or Tag payload (protocol: S256)
 							//or Not used (protocol: R256)
	uint8_t SNR;
	uint16_t milliseconds;

    uint8_t protocol;
	
} IoF_TBR_tag;

typedef struct{
	uint8_t batvoltage;
	uint8_t airtemp;
	uint32_t longitude;
	uint8_t PDOP; //pdop [8:14], lat[15]
	uint32_t latitude;
	uint8_t fix;
	uint8_t num_of_sattelites;
}IoF_bouy_status;


#endif