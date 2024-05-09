/*
 * rtc.c
 *
 *  Created on: 20. September 2023
 *      Author: Janne Marie List
 *
 */

#include "rtc.h"
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
/* STEP 3 - Include the header file of the I2C API */
#include <zephyr/drivers/i2c.h>
/* STEP 4.1 - Include the header file of printk() */
#include <zephyr/sys/printk.h>
// #include <nrf_modem_gnss.h>

//I2C RV3032
#define I2C0_NODE DT_NODELABEL(rv3032)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);



//global variables
uint8_t i2c_ReceiveBuffer[10];
extern rtc_time_dec_t rtc_time;
extern rtc_time_bcd_t time_bcd;

uint8_t rtc_init(void){

  // Setup I2C protocol
  if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
		return EXIT_FAILURE;
	}
  // Reset all registers to zero
  rtc_reset();

  // RTC Clockout pin
  // GPIO_PinModeSet(RTC_PORT, RTC_CLKOUT, gpioModeInputPull, 1);
  // GPIO_IntConfig(RTC_PORT, RTC_CLKOUT, true, false, true);        // Interrupt on rising edge
  rtc_i2c_write(RTC_EEPROM_Clkout_2, RTC_EEPROM_Clkout_2_FD_1);   // set CLKOUT to 1 Hz

  // Setup synchronization with GPS time
  // rtc_convert_time_from_gps_finished = 0;
  // rtc_start_sync_with_gps = 0;
  // rtc_second_offset_to_gps = 0;

  // Enable Interrupt Clockout pin
  // NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  // NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  return EXIT_SUCCESS;
}

// Write data to register given by address
uint8_t rtc_i2c_write(uint8_t address, uint8_t data){
  
  uint8_t i2c_TransmitBuffer[2] = {address, data};

  int ret = i2c_reg_write_byte_dt(&dev_i2c, address, data);

  if(ret != 0){
    printk("Failed to write to I2C device address %x at reg. %x \n\r", dev_i2c.addr,i2c_TransmitBuffer[0]);
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}


uint8_t rtc_write_fix_data_first(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
  int ret;
  rtc_time_dec_t datetime;
  datetime.year     = pvt_data->datetime.year - 2000;
  datetime.month    = pvt_data->datetime.month;
  datetime.day      = pvt_data->datetime.day;
  datetime.hour     = pvt_data->datetime.hour;
  datetime.minute   = pvt_data->datetime.minute;
  datetime.seconds  = pvt_data->datetime.seconds + 1;  //add 1 second
	datetime.sec100   = pvt_data->datetime.ms;

  //convert into bcd
  ret = rtc_convert_nav_data_into_bcd_format(datetime);
  if(ret != 0){
    printk("Error while converting NAV data into BCD format\n");
    return EXIT_FAILURE;
  }
  //write bcd format date/time to RTC
  ret = rtc_i2c_write_datetime(time_bcd);
  if(ret != 0){
    printk("Error while writing NAV date/time into RTC\n");
    return EXIT_FAILURE;
  }

  printk("Sucessfully wrote NAV date/time into RTC!\n");
  return ret;
}

uint8_t rtc_sync_nav_second(void){
  //Enable ESYN bit 15h – EVI Control register
  int ret = rtc_i2c_write(RTC_EVI_Control, 1);
  if(ret != 0){
    printk("Enabling ESYN bit for synchronization of RTC failed!\n");
    return EXIT_FAILURE;
  }
  printk("Enabled ESYN bit for sync of RTC successfully!\n");
  return EXIT_SUCCESS;
}

uint8_t rtc_i2c_write_datetime(rtc_time_bcd_t datetime){

  uint8_t timearr[] = { datetime.sec, datetime.min, datetime.hour, \
                        datetime.weekday, datetime.day, datetime.month, datetime.year};



  int ret = i2c_burst_write_dt(&dev_i2c, RTC_Sec, timearr, sizeof(timearr));
  if(ret != 0){
    printk("Failed to write to I2C device address %x at reg. %x \n\r", dev_i2c.addr,timearr[0]);
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

// Read registers starting at address
uint8_t rtc_i2c_read(uint8_t address, uint8_t num_bytes){

  uint8_t i2c_ReceiveBuffer_byte;


  int ret = i2c_reg_read_byte_dt(&dev_i2c, address, &i2c_ReceiveBuffer_byte);
  if(ret != 0){
    printk("Failed to read from I2C device address %x at reg. %x \n\r", dev_i2c.addr,address);
    return EXIT_FAILURE;
  }

  return i2c_ReceiveBuffer_byte;
}


uint8_t rtc_i2c_read_multiple(uint8_t address, uint8_t num_bytes){

  
  memset((void *)i2c_ReceiveBuffer, 0, sizeof(i2c_ReceiveBuffer));

  int ret = i2c_burst_read_dt(&dev_i2c, RTC_Sec, i2c_ReceiveBuffer, num_bytes);

  if(ret != 0){
    printk("Failed to read from I2C device address %x at reg. %x \n\r", dev_i2c.addr,address);
    return EXIT_FAILURE;
  } 

  return EXIT_SUCCESS;
}

// // Toggle bits given by data (not overwrite entire byte)
// void rtc_read_than_write(uint8_t address, uint8_t data, uint8_t positive){
//   rtc_i2c_read(address, 1);
//   uint8_t current_state = i2c_ReceiveBuffer[0];
//   if (positive){
//       rtc_i2c_write(address, current_state | data);
//   }else{
//       rtc_i2c_write(address, current_state & ~data);
//   }
// }

void rtc_reset_eeprom(){
  rtc_i2c_write(RTC_EEPROM_PMU, 0x00);
  rtc_i2c_write(RTC_EEPROM_Offset, 0x00);
  rtc_i2c_write(RTC_EEPROM_Clkout_1, 0x00);
  rtc_i2c_write(RTC_EEPROM_Clkout_2, 0x00);
}

void rtc_reset(){
  // Reset all registers
  for (uint8_t i = 0x00; i < 0x2D; i++){
      rtc_i2c_write(i, 0x00);
  }

  // Reset EEPROM
  rtc_reset_eeprom();
}

// GPS nav data is in decimal format, the RTC stores data in BCD format
uint8_t rtc_convert_nav_data_into_bcd_format(rtc_time_dec_t datetime){

  uint8_t montharr[] = {JANUARY, FEBRUARY, MARCH, APRIL, MAY, JUNE, \
                      JULY, AUGUST, SEPTEMBER, OCTOBER, NOVEMBER, DECEMBER};
  uint8_t yy = datetime.year;
  time_bcd.year    = rtc_convert_decimal_to_bcd(yy);
  time_bcd.month   = rtc_convert_decimal_to_bcd(datetime.month);
  time_bcd.day     = rtc_convert_decimal_to_bcd(datetime.day);
  //(Year Code + Month Code + Century Code + Date Number - Leap Year Code) mod 7
  //TODO: needs leap year calculation
  time_bcd.weekday = (((yy + (yy/4)) % 7) +  montharr[datetime.month-1] + 6 + datetime.day) % 7;
  time_bcd.hour    = rtc_convert_decimal_to_bcd(datetime.hour);
  time_bcd.min     = rtc_convert_decimal_to_bcd(datetime.minute);
  time_bcd.sec     = rtc_convert_decimal_to_bcd(datetime.seconds);
  return EXIT_SUCCESS;
}



uint8_t rtc_read_time_data(void){
  // Request time stamp data
  int ret = rtc_i2c_read_multiple(RTC_Sec, 7);
  if (ret != 0){
    printk("Error in reading time data!\n");
    return EXIT_FAILURE;
  }
  // Read data from I2C buffer
  // rtc_time->sec100   = rtc_convert_bcd_to_decimal(i2c_ReceiveBuffer[0]);
  rtc_time.seconds  = rtc_convert_bcd_to_decimal(i2c_ReceiveBuffer[0]);
  rtc_time.minute   = rtc_convert_bcd_to_decimal(i2c_ReceiveBuffer[1]);
  rtc_time.hour     = rtc_convert_bcd_to_decimal(i2c_ReceiveBuffer[2]);
  rtc_time.weekday  = i2c_ReceiveBuffer[3];
  rtc_time.day      = rtc_convert_bcd_to_decimal(i2c_ReceiveBuffer[4]);
  rtc_time.month    = rtc_convert_bcd_to_decimal(i2c_ReceiveBuffer[5]);
  rtc_time.year     = rtc_convert_bcd_to_decimal(i2c_ReceiveBuffer[6]);
  // Some sort of check here
  return EXIT_SUCCESS;
}

// // Write time data directly to registers
// void rtc_write_time_data(rtc_time_bcd_t time_data, uint8_t bytes, uint8_t reg_offset)
// {
//   uint8_t data[7] = {time_data.sec, time_data.min, time_data.hour, time_data.weekday, time_data.date, time_data.month, time_data.year};
//   if (bytes < 1 || bytes > 7){
//       bytes = 7;
//   }
//   rtc_i2c_write_array(RTC_Sec + reg_offset, data, bytes);
// }

// // Write time data using stop flag, the RTC starts counting when the stop flag is zero
// void rtc_write_time_data_stop(rtc_time_bcd_t time_data, uint8_t bytes)
// {

//   // Set stop bit = 1
//   rtc_read_than_write(RTC_Control_2, RTC_Control_2_STOP, 1);

//   // Write time data
//   rtc_write_time_data(time_data, bytes, 0);

//   // Set stop bit = 0
//   rtc_read_than_write(RTC_Control_2, RTC_Control_2_STOP, 0);
// }

// // Requires rtc_evi_init, write data using the EVI pin
// void rtc_write_time_data_esyn(rtc_time_bcd_t time_data, uint8_t bytes)
// {
//   //rtc_time_bcd_t time_bcd = rtc_convert_time_struct_decimal_to_bcd(time_data);

//   // Set ESYN = 1
//   rtc_read_than_write(RTC_EVI_Control, RTC_EVI_Control_ESYN, 1);

//   // Generate time_stamp and write data
//   rtc_generate_evi_interrupt();
//   rtc_write_time_data(time_data, bytes, 0);
//   rtc_clear_time_stamp_evi();
// }

void rtc_evi_init(){

  // // EIE = 0 (external event interrupt enable)
  // rtc_read_than_write(RTC_Control_2, RTC_Control_2_EIE, 0);

  // // EVOW = 0 (time stamp evi overwrite bit, first event)
  // // Set EVOW = 1 if last event recorded
  // rtc_read_than_write(RTC_TS_Control, RTC_TS_Control_EVOW, 0);


  // // EVF = 0 (External event flag)
  // rtc_read_than_write(RTC_Status, RTC_Status_EVF, 0);

  // // EVR = 1 (Time stamp EVI reset bit, reset all 8 timestamp EVI registers to 0x00)
  // rtc_read_than_write(RTC_TS_Control, RTC_TS_Control_EVR, 1);

  // // Setup EVI Control: EHL = 1, ET = 00 (rising edge)
  // rtc_i2c_write(RTC_EVI_Control, RTC_EVI_Control_EHL);

  // // Set CEIE = 1 if enable clkout when event occurs
  // // Set INTDE = 1 if enable interrupt delay after clkout on
  // // Set EIE = 1 if interrupt on INT pin or if want to use interrupt for controlled clock output function
  // rtc_read_than_write(RTC_Control_2, RTC_Control_2_EIE, 1);

}

// void rtc_clear_time_stamp_evi()
// {
//   // Set EVI Pin low
//   GPIO_PinOutClear(RTC_PORT, RTC_EVI);

//   // EVR = 1 (Time stamp EVI reset bit, reset all 8 timestamp EVI registers to 0x00)
//   rtc_read_than_write(RTC_TS_Control, RTC_TS_Control_EVR, 1);
// }

// void rtc_generate_evi_interrupt()
// {
//   // Set EVI pin high
//   GPIO_PinOutSet(RTC_PORT, RTC_EVI);
// }

// rtc_time_bcd_t rtc_read_time_stamp_evi()
// {
//   rtc_time_bcd_t rtc_time_stamp;

//   // Request time stamp data
//   rtc_i2c_read(RTC_TS_EVI_100_Sec, 7);

//   // Read data from I2C buffer
//   rtc_time_stamp.sec100 = i2c_ReceiveBuffer[0];
//   rtc_time_stamp.sec = i2c_ReceiveBuffer[1];
//   rtc_time_stamp.min = i2c_ReceiveBuffer[2];
//   rtc_time_stamp.hour = i2c_ReceiveBuffer[3];
//   rtc_time_stamp.date = i2c_ReceiveBuffer[4];
//   rtc_time_stamp.month = i2c_ReceiveBuffer[5];
//   rtc_time_stamp.year = i2c_ReceiveBuffer[6];

//   // Some sort of check here
//   rtc_time_stamp.valid_time = 1;

//   rtc_clear_time_stamp_evi();

//   return rtc_time_stamp;
// }

// rtc_time_dec_t rtc_convert_time_struct_bcd_to_decimal(rtc_time_bcd_t time_bcd){
//   rtc_time_dec_t time_dec;
//   time_dec.sec100 = rtc_convert_bcd_to_decimal(time_bcd.sec100);
//   time_dec.sec = rtc_convert_bcd_to_decimal(time_bcd.sec);
//   time_dec.min = rtc_convert_bcd_to_decimal(time_bcd.min);
//   time_dec.hour = rtc_convert_bcd_to_decimal(time_bcd.hour);
//   time_dec.month = time_bcd.month;
//   time_dec.weekday = time_bcd.weekday;
//   time_dec.date = rtc_convert_bcd_to_decimal(time_bcd.date);
//   time_dec.year = rtc_convert_bcd_to_decimal(time_bcd.year);
//   return time_dec;
// }

// rtc_time_bcd_t rtc_convert_time_struct_decimal_to_bcd(rtc_time_dec_t time_dec){
//   rtc_time_bcd_t time_bcd;
//   time_bcd.sec100 = rtc_convert_decimal_to_bcd(time_dec.sec100);
//   time_bcd.sec = rtc_convert_decimal_to_bcd(time_dec.sec);
//   time_bcd.min = rtc_convert_decimal_to_bcd(time_dec.min);
//   time_bcd.hour = rtc_convert_decimal_to_bcd(time_dec.hour);
//   time_bcd.month = time_dec.month;
//   time_bcd.weekday = time_dec.weekday;
//   time_bcd.date = rtc_convert_decimal_to_bcd(time_dec.date);
//   time_bcd.year = rtc_convert_decimal_to_bcd(time_dec.year);
//   return time_bcd;
// }

uint8_t rtc_convert_bcd_to_decimal(uint8_t time_bcd){
  uint8_t time_dec = time_bcd & 15;    // last 4 bits are first digit
  time_dec += (time_bcd >> 4) * 10;    // next 4 bits are second digit
  return time_dec;
}

uint8_t rtc_convert_decimal_to_bcd(uint8_t time_dec){
  uint8_t time_bcd = time_dec % 10;    // last 4 bits are standard encoded (values 0, 1, 4, 8)
  time_bcd += (time_dec / 10) << 4;
  return time_bcd;
}

// /*
// uint32_t rtc_convert_timestamp_to_unix(rtc_time_bcd_t time_bcd){

//   uint16_t year = 2000 + (uint16_t)rtc_convert_bcd_to_decimal(time_bcd.year);
//   return utc_time_to_unix_time(year, time_bcd.month,
//                              rtc_convert_bcd_to_decimal(time_bcd.date),
//                              rtc_convert_bcd_to_decimal(time_bcd.hour),
//                              rtc_convert_bcd_to_decimal(time_bcd.min),
//                              rtc_convert_bcd_to_decimal(time_bcd.sec));

// }*/

// void rtc_enable_clockout_evi_interrupt(){
//   rtc_read_than_write(RTC_EEPROM_PMU, RTC_EEPROM_PMU_NCLKE, 1);     // enable clkout when interrupt
//   rtc_read_than_write(RTC_Control_2, RTC_Control_2_CLKIE, 1);       // enable clk interrupt
//   rtc_read_than_write(RTC_Clock_Int_Mask, RTC_Clock_Int_Mask_CEIE, 1);  // clkout when evi interrupt
//   rtc_read_than_write(RTC_Control_2, RTC_Control_2_EIE, 1);
// }


// void rtc_enable_4096_Hz_output_on_timer_interrupt(){
//   // RTC Interrupt Pin
//   GPIO_PinModeSet(RTC_PORT, RTC_nINT, gpioModeInputPull, 1);
//   GPIO_IntConfig(RTC_PORT, RTC_nINT, true, false, true);        // rising edge

//   // Setup Periodic Countdown timer interrupt
//   // Default: Set register RTC_Control_1 (TE, TD = 0) (Periodic Countdown timer enable, timer clock frequency selection = 00 = 4096Hz)
//   // Default: Set register RTC_Control_2 (TIE = 0) (periodic countdown timer interrupt enable bit)
//   // Default: Set register RTC_Status (TF = 0) Periodic Time Update flag

//   // Countdown Period based on timer Clock frequency = Frequency = 4096 (= 0.9998 s)
//   // Timer value = 1 => INT pin outputs 4096 Hz clock signal
//   rtc_i2c_write(RTC_Timer_Value_0, 0x01);  // Timer Value 0
//   rtc_i2c_write(RTC_Timer_Value_1, 0x00);  // Timer Value 1

//   // Enable interrupt
//   rtc_read_than_write(RTC_Control_2, RTC_Control_2_TIE, 1);  // Enable timer interrupt
//   rtc_i2c_write(RTC_Control_1, RTC_Control_1_TE);   // Enable periodic timer
// }

// // Enable evi for this
// /*
// void rtc_test_read_write_and_evi_interrupt_timestamps(){
//     // Test rtc_time read
//     rtc_time_bcd_t rtc_time = rtc_read_time_data();

//     // Test generate time_stamps

//     rtc_generate_evi_interrupt();
//     rtc_time_bcd_t time_stamp = rtc_read_time_stamp_evi();

//     rtc_generate_evi_interrupt();
//     time_stamp = rtc_read_time_stamp_evi();

//     rtc_generate_evi_interrupt();
//     time_stamp = rtc_read_time_stamp_evi();

//     // Test set time with stop byte
//     rtc_time_bcd_t my_time;
//     my_time.sec = 0;
//     my_time.min = 0;
//     my_time.hour = 0;
//     my_time.weekday = 5;
//     my_time.month = 9;
//     my_time.year = 0;
//     my_time.date = 15;

//     rtc_write_time_data_stop(my_time, 0);

//     rtc_time = rtc_read_time_data();

//     // Test set time with event interrupt
//     rtc_time = rtc_read_time_data();
//     rtc_write_time_data_esyn(my_time, 0);
//     rtc_time = rtc_read_time_data();

//     // Test dec bcd conversion
//     rtc_time_dec_t time_norm;
//     rtc_generate_evi_interrupt();
//     time_stamp = rtc_read_time_stamp_evi();
//     time_norm = rtc_convert_time_struct_bcd_to_decimal(time_stamp);
// }*/

// void rtc_test_measure_time_to_clkout_enabled(uint8_t evi){
//     /* Test EVI interrupt controlled clock output*/
//     test_time.sec = 1;

//     if (evi){
//         rtc_read_than_write(RTC_EVI_Control, RTC_EVI_Control_ESYN, 1);
//         rtc_generate_evi_interrupt();
//         rtc_write_time_data_esyn(test_time, 1);
//         rtc_clear_time_stamp_evi();
//     }
//     else{
//         rtc_generate_evi_interrupt();
//         rtc_write_time_data(test_time, 1, 0);
//         rtc_clear_time_stamp_evi();
//     }
// }



//returns temperature in celcius
int16_t rtc_read_temp(void)
{
  // uint8_t lsbtemp = rtc_i2c_read(RTC_TEMP_LSB, 1) & (15 << 4);
  int16_t temp   = (int16_t)rtc_i2c_read(RTC_TEMP_MSB, 1);
  temp = (temp << 4) / 16;
  
  // int16_t temp   = ((rtc_i2c_read(RTC_TEMP_MSB, 1) << 4) | lsbtemp) / 16;
  return (int16_t)temp;
}
