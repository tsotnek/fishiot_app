#ifndef _DEVICES_HEADER_RTC_H_
#define _DEVICES_HEADER_RTC_H_

#include <stdint.h>
#include <stdbool.h>
#include <nrf_modem_gnss.h>
#include <zephyr/drivers/i2c.h>

#define RTC_EEPROM_PMU      0xC0
#define RTC_EEPROM_Offset   0xC1
#define RTC_EEPROM_Clkout_1 0xC2
#define RTC_EEPROM_Clkout_2 0xC3

#define RTC_100_Sec 0x00
#define RTC_Sec     0x01
#define RTC_Min     0x02
#define RTC_Hours   0x03
#define RTC_Weekday 0x04
#define RTC_Date    0x05
#define RTC_Month   0x06
#define RTC_Year    0x07
#define RTC_Timer_Value_0   0x0B
#define RTC_Timer_Value_1   0x0C
#define RTC_Status          0x0D
#define RTC_TEMP_LSB        0x0E
#define RTC_TEMP_MSB        0x0F
#define RTC_Control_1       0x10
#define RTC_Control_2       0x11
#define RTC_Control_3       0x12
#define RTC_TS_Control       0x13
#define RTC_Clock_Int_Mask  0x14
#define RTC_EVI_Control     0x15
#define RTC_TS_EVI_Count    0x26
#define RTC_TS_EVI_100_Sec  0x27
#define RTC_TS_EVI_Sec      0x28
#define RTC_TS_EVI_Min      0x29
#define RTC_TS_EVI_Hours    0x2A
#define RTC_TS_EVI_Date     0x2B
#define RTC_TS_EVI_Month    0x2C
#define RTC_TS_EVI_Year     0x2D

#define RTC_Status_THF      (1 << 7)
#define RTC_Status_TLF      (1 << 6)
#define RTC_Status_UF       (1 << 5)
#define RTC_Status_TF       (1 << 4)
#define RTC_Status_AF       (1 << 3)
#define RTC_Status_EVF      (1 << 2)
#define RTC_Status_PORF     (1 << 1)
#define RTC_Status_VLF      (1 << 0)

#define RTC_Control_1_GP0      (1 << 5)
#define RTC_Control_1_USEL     (1 << 4)
#define RTC_Control_1_TE       (1 << 3)
#define RTC_Control_1_EERD     (1 << 2)
#define RTC_Control_1_TD_4096  (0 << 0)
#define RTC_Control_1_TD_64    (1 << 0)
#define RTC_Control_1_TD_1     (2 << 0)
#define RTC_Control_1_TD_160   (3 << 0)

#define RTC_Control_2_CLKIE    (1 << 6)
#define RTC_Control_2_UIE      (1 << 5)
#define RTC_Control_2_TIE      (1 << 4)
#define RTC_Control_2_AIE      (1 << 3)
#define RTC_Control_2_EIE      (1 << 2)
#define RTC_Control_2_GP1      (1 << 1)
#define RTC_Control_2_STOP     (1 << 0)

#define RTC_Control_3_BSIE     (1 << 4)
#define RTC_Control_3_THE      (1 << 3)
#define RTC_Control_3_TLE      (1 << 2)
#define RTC_Control_3_THIE     (1 << 1)
#define RTC_Control_3_TLIE     (1 << 0)

#define RTC_TS_Control_EVR      (1 << 5)
#define RTC_TS_Control_THR      (1 << 4)
#define RTC_TS_Control_TLR      (1 << 3)
#define RTC_TS_Control_EVOW     (1 << 2)
#define RTC_TS_Control_THOW     (1 << 1)
#define RTC_TS_Control_TLOW     (1 << 0)

#define RTC_Clock_Int_Mask_CLKD     (1 << 7)
#define RTC_Clock_Int_Mask_INTDE    (1 << 6)
#define RTC_Clock_Int_Mask_CEIE     (1 << 5)
#define RTC_Clock_Int_Mask_CAIE     (1 << 4)
#define RTC_Clock_Int_Mask_CTIE     (1 << 3)
#define RTC_Clock_Int_Mask_CUIE     (1 << 2)
#define RTC_Clock_Int_Mask_CTHIE    (1 << 1)
#define RTC_Clock_Int_Mask_CTLIE    (1 << 0)

#define RTC_EVI_Control_CLKDE    (1 << 7)
#define RTC_EVI_Control_EHL      (1 << 6)
#define RTC_EVI_Control_ET_None  (0 << 4)
#define RTC_EVI_Control_ET_256   (1 << 4)
#define RTC_EVI_Control_ET_64    (2 << 4)
#define RTC_EVI_Control_ET_8     (3 << 4)
#define RTC_EVI_Control_ESYN     (1 << 0)

#define RTC_EEPROM_PMU_NCLKE    (1 << 6)
#define RTC_EEPROM_PMU_BSM_SOD  (0 << 4)
#define RTC_EEPROM_PMU_BSM_DSM  (1 << 4)
#define RTC_EEPROM_PMU_BSM_LSM  (2 << 4)
#define RTC_EEPROM_PMU_TCR_06   (0 << 2)
#define RTC_EEPROM_PMU_TCR_2    (1 << 2)
#define RTC_EEPROM_PMU_TCR_7    (2 << 2)
#define RTC_EEPROM_PMU_TCR_12   (3 << 2)
#define RTC_EEPROM_PMU_TCM_Off  (0 << 0)
#define RTC_EEPROM_PMU_TCM_175  (1 << 0)
#define RTC_EEPROM_PMU_TCM_3    (2 << 0)
#define RTC_EEPROM_PMU_TCM_45   (3 << 0)

#define RTC_EEPROM_Offset_PORIE    (1 << 7)
#define RTC_EEPROM_Offset_VLIE     (1 << 6)
#define RTC_EEPROM_Offset_Offset   (1 << 0)

#define RTC_EEPROM_Clkout_2_OS      (1 << 7)
#define RTC_EEPROM_Clkout_2_FD_32k  (0 << 5)
#define RTC_EEPROM_Clkout_2_FD_1k   (1 << 5)
#define RTC_EEPROM_Clkout_2_FD_64   (2 << 5)
#define RTC_EEPROM_Clkout_2_FD_1    (3 << 5)

#define RTC_ENABLE_CLKOUT 1

extern rtc_time_dec_t rtc_time;
extern rtc_time_bcd_t time_bcd;

//months
typedef enum{
JANUARY   = 0,
FEBRUARY  = 3,
MARCH     = 3,
APRIL     = 6,
MAY       = 1,
JUNE      = 4,
JULY      = 6,
AUGUST    = 2,
SEPTEMBER = 5,
OCTOBER   = 0, 
NOVEMBER  = 3,
DECEMBER  = 5
} rtc_months;
//weekdays
typedef enum{
  SUNDAY,
  MONDAY,
  TUESDAY,
  WEDNESDAY,
  THURSDAY,
  FRIDAY,
  SATURDAY
} rtc_weekdays;


typedef struct {
  bool    valid_time;

  uint8_t  year;
  uint8_t  month;
  uint8_t  day;
  uint8_t  weekday;
  uint8_t  hour;
  uint8_t  min;
  uint8_t  sec;
  uint8_t  sec100;

} rtc_time_bcd_t;

typedef struct {
  bool    valid_time;

  uint8_t  year;
  uint8_t  month;
  uint8_t  day;
  uint8_t  weekday;
  uint8_t  hour;
  uint8_t  minute;
  uint8_t  seconds;
  uint8_t  sec100;

} rtc_time_dec_t;

// rtc_time_bcd_t test_time;

// rtc_time_bcd_t gps_time_last_sync_bcd;
// // nav_data_t gps_fix_last_sync;
// uint8_t rtc_convert_time_from_gps_finished;
// uint32_t rtc_4096_signal_count;
// uint8_t rtc_start_sync_with_gps;
// uint8_t rtc_second_offset_to_gps;

uint8_t rtc_init(void);
int16_t rtc_read_temp(void);

uint8_t rtc_i2c_write(uint8_t address, uint8_t data);
uint8_t rtc_i2c_write_datetime(rtc_time_bcd_t datetime);
uint8_t rtc_i2c_read(uint8_t address, uint8_t num_bytes);
// void rtc_read_than_write(uint8_t address, uint8_t data, uint8_t positive);
void rtc_reset_eeprom();
void rtc_reset();

// void rtc_sync();
// void rtc_convert_nav_data_into_bcd_format(nav_data_t nav_data);
// void rtc_synchronize_seconds();
uint8_t rtc_read_time_data(void);
// void rtc_write_time_data(rtc_time_bcd_t time_data, uint8_t bytes, uint8_t offset);
// void rtc_write_time_data_stop(rtc_time_bcd_t time_data, uint8_t bytes);
// void rtc_write_time_data_esyn(rtc_time_bcd_t time_data, uint8_t bytes);

void rtc_evi_init();
// void rtc_generate_evi_interrupt();
// void rtc_clear_time_stamp_evi();
// rtc_time_bcd_t rtc_read_time_stamp_evi();

// rtc_time_dec_t rtc_convert_time_struct_bcd_to_decimal(rtc_time_bcd_t time_bcd);
// rtc_time_bcd_t rtc_convert_time_struct_decimal_to_bcd(rtc_time_dec_t time_dec);
uint8_t rtc_convert_bcd_to_decimal(uint8_t time_bcd);
uint8_t rtc_convert_decimal_to_bcd(uint8_t time_dec);

uint8_t rtc_convert_nav_data_into_bcd_format(rtc_time_dec_t datetime);

uint8_t rtc_write_fix_data_first(struct nrf_modem_gnss_pvt_data_frame *pvt_data);
uint8_t rtc_sync_nav_second(void);


#endif
