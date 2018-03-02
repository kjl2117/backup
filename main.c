/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 * @defgroup fatfs_example_main main.c
 * @{
 * @ingroup fatfs_example
 * @brief FATFS Example Application main file.
 *
 * This file contains the source code for a sample application using FAT filesystem and SD card library.
 *
 */



#include "nrf.h"
#include "bsp.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// Taken from example: example_code/saadc_simpler
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
//#include "nrf.h"
#include "nrf_drv_saadc.h"
#include <math.h>
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include <string.h>
//#define NRF_LOG_MODULE_NAME "APP"		// CHANGED: SOME ERROR FROM THIS
//#include "nrf_log.h"
//#include "nrf_log_ctrl.h"
// END OF example_code/saadc_simpler

// Taken from example: peripheral/twi_sensor
//#include "boards.h"
//#include "app_util_platform.h"
//#include "app_error.h"
#include "nrf_drv_twi.h"
//#include "nrf_delay.h"

#include "Dht22.h"
#include "nrf_gpio.h"

//#include "app_simple_timer.h"
#include "nrf_drv_timer.h"

#include "nrf_serial.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_power.h"

#include "app_uart.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "time.h"

#include "nrf_drv_wdt.h"

#include "nrf_temp.h"




/** GLOBAL VARIABLES **/
//--------------------//

/** Overall **/ //asdf
#define LOG_INTERVAL				5*1000		// ms between sleep/wake
#define NUM_SAMPLES_PER_ON_CYCLE	3	// 20
#define WAIT_BETWEEN_SAMPLES		0	// ms, waitin only if there are multiple samples
#define INITIAL_SETTLING_WAIT		1000	// <500 causes issues?
static uint32_t loop_num = 0;
 static uint32_t avoided_error_cnt = 0;
 static uint32_t avoided_error_cnt_total = 0;
 static uint32_t err_cnt = 0;
 static uint32_t err_cnt_total = 0;
 static uint32_t dht_error_cnt_total = 0;
 static uint32_t hpm_error_cnt_total = 0;
static ret_code_t err_code;
//#define USING_SMALL_PLANTOWER		1	// The Small Plantower uses TWI



/** SD Card Variables.  From example: peripheral/fatfs **/
#define FILE_NAME   "test.TXT"
//#define FILE_NAME   "NORDIC.TXT"
#define MAX_OUT_STR_SIZE	200
#define FILE_HEADER	"Time,PM2_5,PM10,sharpPM,dhtTemp,dhtHum,specCO,figaroCO,figaroCO2,plantower_2_5_value,plantower_10_value, bme_temp_C, bme_humidity, bme_pressure, battery_value, err_cnt, dht_error_cnt_total, hpm_error_cnt_total\r\n"
static FATFS fs;
static FIL file;
FRESULT ff_result;
char* TEST_STRING = "SD card test, v09.\r\n";
static int header_is_written = 0;

/** ADC Card Variables.  From example: example_code/saadc_simpler	**/
#define SAMPLES_IN_BUFFER 1		// It will read each time, but only enter saadc_callback() after buffer is full
//static nrf_saadc_value_t m_buffer[SAMPLES_IN_BUFFER];
static nrf_saadc_value_t adc_value;

/** Figaro CO2, TWI aka I2C.  from example: peripheral/twi_sensor **/
#define TWI_INSTANCE_ID     1
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;
/* Buffer for samples read from temperature sensor. */
static int figCO2_value;
const int figCO2_addr = 0x69; // 8bit I2C address, 0x69 for figaro CO2.  0x68 also available
#define FIG_CO2_XFER_WAIT_TIME	2

/** BME280 Variables, TWI **/
#define BME_MEAS_WAIT	10	// ms, need to wait for it to measure, then read
static int32_t bme_temp_C = 0;	// units: degC*100
static int32_t bme_humidity = 0;	// units: %RH*1000
static uint32_t bme_pressure = 0;	// units: Pa
const int bme_addr = 0x77; // 8bit I2C address, 0x69 for figaro CO2.  0x68 also available
#define BME_BUFF_SIZE	8
// All of the calibration values
static int has_read_calib_data = 0;
static uint16_t    dig_T1;	// use to check if it's been initialized
static int16_t     dig_T2, dig_T3;
static uint16_t    dig_P1;
static int16_t     dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static uint16_t    dig_H1, dig_H3;
static int16_t     dig_H2, dig_H4, dig_H5, dig_H6;
static int32_t     t_fine;

/** RTC, for measuring Time **/
#define RTC_BUFF_SIZE	7
const int rtc_addr = 0x68; // 8bit I2C address, 0x68 for RTC
int timeNow = 0;

/** DHT Variables **/
static int dht_temp_C = 0;
static int dht_humidity = 0;
#define DHT_RETRY_NUM	5


/** Timer variables **/
#define TIMER_NUM	0	// Which Timer to use: TIMER0 reserved for SoftDevice, maybe change sdk_config.h
//static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(TIMER_NUM);
//nrf_timer_cc_channel_t TIMER_CHANNEL_NUM = NRF_TIMER_CC_CHANNEL0;
#define TIMER_CHANNEL_NUM	NRF_TIMER_CC_CHANNEL0
#define	TIMER_TEST_DELAY	100		// delay in us

/** ADC Variables **/
#define ADC_CHANNEL_NUM			0
#define ADC_SAMPLE_DELAY		100		// Unit: ms
#define ADC_GAIN_VALUE			(1.0f/6)     // 1/6, ADC gain.
#define ADC_REFERENCE_VOLTAGE   (0.6f)       // 0.6V, The standard internal ADC reference voltage.
#define ADC_RESOLUTION_BITS		(8 + (SAADC_CONFIG_RESOLUTION * 2)) // 10-bit resolution, ADC resolution [bits].
#define MBED_VREF				3.0f	// In Mbed, AIN was read as percentage of VREF=3V
// Result = [V(p) - V(n)] * GAIN/REFERENCE * 2^(RESOLUTION); V(n)=0, GAIN=1/6, REFERENCE=0.6V
// Therefore: V(p) = Result / [GAIN/REFERENCE * 2^(RESOLUTION)]
static float adc_to_V;	// 0.003515625 == 1.0f / ((ADC_GAIN_VALUE / ADC_REFERENCE_VOLTAGE) * pow(2, ADC_RESOLUTION_BITS));
#define PRE_READ_WAIT			5	// ms, wait before an Analog read to let transients settle after switching to high impedance of AIN

/** GPIO Variables **/
#define GPIO_TEST_DELAY		2	// LED on time (ms)

/** Sharp PM Variables **/
#define SHARP_PM_CHANNEL_NUM		1
static nrf_saadc_value_t sharpPM_value;

/** Spec CO Variables **/
#define SPEC_CO_CHANNEL_NUM		2
#define SPEC_CO_DELAY			2	//ms, wait between samples that will be avg'ed
static int specCO_value;

/** Figaro CO Variables **/
#define FIG_CO_CHANNEL_NUM		3
static nrf_saadc_value_t figCO_value;

/** Battery Check Variables **/
#define BATTERY_CHANNEL_NUM		4
static nrf_saadc_value_t battery_value;

/** Small Plantower (TWI) **/
#define PLANTOWER_TWI_BUFF_SIZE		32
const int plantower_twi_addr = 0x12; // 8bit I2C address, 0x12 for Small Plantower
static int plantower_2_5_value = 0;
static int plantower_10_value = 0;


/** HPM Variables (Serial communication) **/
#define USING_AUTOSEND			1
#define USING_HONEYWELL			0
#define USING_PLANTOWER			1
//#define HPM_TEST_DELAY			5*1000	//ms
#define NUM_AUTOSEND_READ_TRIES	32
#define OP_QUEUES_SIZE          3
#define APP_TIMER_PRESCALER     NRF_SERIAL_APP_TIMER_PRESCALER
#define HPM_SERIAL_TIMEOUT		1100	// ms, tested with 1ms and it worked fine
#define HPM_NUM_RETRIES			3
#define HPM_RETRY_WAIT			0	// ms, wait between each read attempt
//#define HPM_RETRY_NUM			5
//#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
//#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
//#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */
#define HPM_CMD_LEN		4
#if USING_AUTOSEND
	#define HPM_BUFF_SIZE	32
#else
	#define HPM_BUFF_SIZE	8
#endif
#if USING_HONEYWELL
	static char hpm_stop_autosend_cmd[] = 	{0x68, 0x01, 0x20, 0x77 };
	static char hpm_start_meas_cmd[] = 		{0x68, 0x01, 0x01, 0x96 };
	static char hpm_stop_meas_cmd[] = 		{0x68, 0x01, 0x02, 0x95 };
	static char hpm_read_meas_cmd[] = 		{0x68, 0x01, 0x04, 0x93 };
#elif USING_PLANTOWER	// TODO: Replace with different commands and get Passive mode working
	static char hpm_stop_autosend_cmd[] = 	{0x68, 0x01, 0x20, 0x77 };
	static char hpm_start_meas_cmd[] = 		{0x68, 0x01, 0x01, 0x96 };
	static char hpm_stop_meas_cmd[] = 		{0x68, 0x01, 0x02, 0x95 };
	static char hpm_read_meas_cmd[] = 		{0x68, 0x01, 0x04, 0x93 };
#endif
//static char hpm_ack_rx[2] = {0x0, 0x0 };
static int hpm_2_5_value = 0;
static int hpm_10_value = 0;


/** WATCHDOG TIMER (WDT) **/
nrf_drv_wdt_channel_id wdt_channel_id;
//#define WDT_TIMEOUT		20*1000	// ms
//#define WDT_TIMEOUT		(10*1000 + 2*(HPM_TEST_DELAY+DHT_STARTUP_WAIT_TIME))	// ms, make it more than DHT and HPM delays
#define WDT_TIMEOUT		60*1000 + NUM_SAMPLES_PER_ON_CYCLE*(WAIT_BETWEEN_SAMPLES+1000) + (DHT_STARTUP_WAIT_TIME + PLANTOWER_STARTUP_WAIT_TIME + HPM_STARTUP_WAIT_TIME)	// ms, make it more than DHT and HPM delays
static int wdt_triggered = 0;

/** APP TIMERS **/
static int dht_startup_wait_done = 0;
APP_TIMER_DEF(dht_startup_timer);
#define DHT_STARTUP_WAIT_TIME			0.1*1000	//ms
static int plantower_startup_wait_done = 0;
APP_TIMER_DEF(plantower_startup_timer);
#define PLANTOWER_STARTUP_WAIT_TIME		6*1000	//ms
static int hpm_startup_wait_done = 0;
APP_TIMER_DEF(hpm_startup_timer);
#define HPM_STARTUP_WAIT_TIME			6*1000	//ms,	6s (10-15s recommended) for Honeywell; 10s for Plantower

/** PIN maps **/
// NOTE: for AIN, must use full name, not just pin num!
//#define TWI_SCL_PIN			27		///< 	P0.27 SCL pin.
//#define TWI_SDA_PIN			26		///< 	P0.26 SDA pin.
//#define DHT_PIN				16		///< 	P0.16 DHT pin
//#define ADC_PIN				NRF_SAADC_INPUT_AIN0		///< 	P0.02 AIN0 pin.
//#define GPIO_TEST_PIN		23		///< 	P0.23 random pin for testing
//#define SHARP_PM_LED		22
//#define SHARP_PM_PIN		NRF_SAADC_INPUT_AIN3		///< 	P0.05 AIN3 pin.
//#define SPEC_CO_PIN			NRF_SAADC_INPUT_AIN1		///< 	P0.03 AIN1 pin.
//#define FIG_CO_PIN			NRF_SAADC_INPUT_AIN7		///< 	P0.31 AIN7 pin.
//#define BATTERY_PIN			NRF_SAADC_INPUT_VDD			///< 	NO PIN, VDD internally
//#define HPM_RX_PIN_NUMBER	17		//8
//#define HPM_TX_PIN_NUMBER	18		//6
//#define SDC_SCK_PIN			14		///< 	P0.14 SDC serial clock (SCK) pin.
//#define SDC_MOSI_PIN		12		///< 	P0.12 SDC serial data in (DI) pin.
//#define SDC_MISO_PIN		13		///< 	P0.13 SDC serial data out (DO) pin.
//#define SDC_CS_PIN			11		///< 	P0.11 SDC chip select (CS) pin.
//#define ADP_PIN				20		///< 	P0.22 ADP pin for cutting power, sleep/wake
//#define STATUS_LED			19		///< 	P0.19 pin. LED3, LED1-4 => p0.17-20

// Pins for the custom Sensen boards
#define TWI_SCL_PIN			27		///< 	P0.27 SCL pin.
#define TWI_SDA_PIN			26		///< 	P0.26 SDA pin.

#define DHT_PIN				16		///< 	P0.16 DHT pin
#define ADC_PIN				NRF_SAADC_INPUT_AIN0		///< 	P0.02 AIN0 pin.
#define GPIO_TEST_PIN		23		///< 	P0.23 random pin for testing

#define SHARP_PM_LED		15
#define SHARP_PM_PIN		NRF_SAADC_INPUT_AIN3		///< 	P0.05 AIN3 pin.
#define SPEC_CO_PIN			NRF_SAADC_INPUT_AIN1		///< 	P0.03 AIN1 pin.
#define FIG_CO_PIN			NRF_SAADC_INPUT_AIN7		///< 	P0.31 AIN7 pin.
#define BATTERY_PIN			NRF_SAADC_INPUT_VDD			///< 	NO PIN, VDD internally

#define HPM_RX_PIN_NUMBER	8		//8
#define HPM_TX_PIN_NUMBER	6		//6
#define SDC_SCK_PIN			25		///< 	SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN		23		///< 	SDC serial data in (DI) pin.
#define SDC_MISO_PIN		24		///< 	SDC serial data out (DO) pin.
#define SDC_CS_PIN			22		///< 	SDC chip select (CS) pin.
#define ADP_PIN				20		///< 	20/19, ADP1/ADP2 pin for cutting power, sleep/wake
#define STATUS_LED			18		///< 	Red
#define SAMPLE_LED			17		///< 	Yellow



// The different sensors (used for startup handler)
typedef enum {
	DHT,		// DIG
	GPIO,		// DIG
	FIGARO_CO2,	// I2C
	RTC,		// I2C
	BME,		// I2C
	ADC,		// AIN
	SHARP,		// AIN
	SPEC_CO,	// AIN
	FIGARO_CO,	// AIN
	BATTERY,	// AIN (no pin needed)
	SMALL_PLANTOWER,	// I2C
	HONEYWELL,			// SERIAL
} component_type;

component_type twi_sensors[] = {
		FIGARO_CO2,
		RTC,
		BME,
		SMALL_PLANTOWER,
};

//static component_type *components_used;
static int components_used_size;


/**
 * Function for checking if array contains a value (used for selecting sensor code)
 */
bool using_sensor(component_type val, component_type *arr) {
	int i;
    for (i=0; i < components_used_size; i++) {
        if (arr[i] == val)
            return true;
    }
    return false;
}


// Watchdog timer event handler
void wdt_event_handler(void)
{
	//    bsp_board_leds_off();
//	    bsp_board_leds_on();
	wdt_triggered = 1;
//	NVIC_SystemReset();
//	NRF_LOG_INFO("*** TIMEOUT: Watchdog Timer ***");


    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}

// Timeout handler for the single shot timer
static void dht_startup_handler(void * p_context) {
	dht_startup_wait_done = 1;
	NRF_LOG_INFO("dht_startup_wait_done: %d", dht_startup_wait_done);
}
static void plantower_startup_handler(void * p_context) {
	plantower_startup_wait_done = 1;
	NRF_LOG_INFO("plantower_startup_wait_done: %d", plantower_startup_wait_done);
}
static void hpm_startup_handler(void * p_context) {
	hpm_startup_wait_done = 1;
	NRF_LOG_INFO("hpm_startup_wait_done: %d", hpm_startup_wait_done);
}
//static void dht_startup_handler(void * p_context)
//{
//	NRF_LOG_INFO("p_context: %d", p_context);
//
//
//	switch ((int) p_context) {
////	switch (p_context) {
//	case DHT:
//		dht_startup_wait_done = 1;
//		NRF_LOG_INFO("dht_startup_wait_done: %d", dht_startup_wait_done);
//		break;
//	case PLANTOWER:
//		plantower_startup_wait_done = 1;
//		NRF_LOG_INFO("plantower_startup_wait_done: %d", plantower_startup_wait_done);
//		break;
//	case HPM:
//		hpm_startup_wait_done = 1;
//		NRF_LOG_INFO("hpm_startup_wait_done: %d", hpm_startup_wait_done);
//		break;
//	default:
//		NRF_LOG_INFO("**ERROR: p_context in sensor_startup_handler not recognized: %d **", p_context);
//		err_cnt_total++;
//
//
//	}
//}



///**
// * @brief Assert callback.
// *
// * @param[in] id    Fault identifier. See @ref NRF_FAULT_IDS.
// * @param[in] pc    The program counter of the instruction that triggered the fault, or 0 if
// *                  unavailable.
// * @param[in] info  Optional additional information regarding the fault. Refer to each fault
// *                  identifier for details.
// */
//void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
//{
//    bsp_board_leds_off();
//    while (1);
//}








/**
 * @brief  SDC block device definition
 * */
//NRF_BLOCK_DEV_SDC_DEFINE(
//        m_block_dev_sdc,
//        NRF_BLOCK_DEV_SDC_CONFIG(
//                SDC_SECTOR_SIZE,
//                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
//         ),
//         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
//);






///** MACROS needed for app_uart **/
//// Taken from uart example
void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}



















/** MACROS needed for Serial **/
// Taken from Serial example
// TODO: Maybe should remove this altogether.
static void sleep_handler(void)
{
//    __WFE();
//    __SEV();
//    __WFE();
}

NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uart0_drv_config,
                      HPM_RX_PIN_NUMBER, HPM_TX_PIN_NUMBER,
                      RTS_PIN_NUMBER, CTS_PIN_NUMBER,
					  NRF_UART_HWFC_DISABLED,	//NRF_UART_HWFC_ENABLED,
					  NRF_UART_PARITY_EXCLUDED,
					  NRF_UART_BAUDRATE_9600,	//NRF_UART_BAUDRATE_115200,
                      UART_DEFAULT_CONFIG_IRQ_PRIORITY);

#define SERIAL_FIFO_TX_SIZE 32
//#define SERIAL_FIFO_RX_SIZE 3
#define SERIAL_FIFO_RX_SIZE 32
NRF_SERIAL_QUEUES_DEF(serial_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);

#define SERIAL_BUFF_TX_SIZE 1
#define SERIAL_BUFF_RX_SIZE 1
//#define SERIAL_BUFF_TX_SIZE 32
//#define SERIAL_BUFF_RX_SIZE 32
NRF_SERIAL_BUFFERS_DEF(serial_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);

NRF_SERIAL_CONFIG_DEF(serial_config, NRF_SERIAL_MODE_IRQ,
                      &serial_queues, &serial_buffs, NULL, sleep_handler);

NRF_SERIAL_UART_DEF(serial_uart, 0);







// HPM, send a command and check that it was properly received and acknowledged
int hpm_cmd_and_ack(char cmd[]) {

//	ret_code_t err_code;
    size_t serial_bytes_written = 0;
//    size_t serial_bytes_read;
	NRF_LOG_INFO("--1A");

//	err_code = nrf_serial_rx_drain(&serial_uart);

    // Sending the cmd
    err_code = nrf_serial_write(&serial_uart, cmd, HPM_CMD_LEN, &serial_bytes_written, HPM_SERIAL_TIMEOUT);
//    err_code = nrf_serial_write(&serial_uart, cmd, HPM_CMD_LEN, NULL, 0);
    APP_ERROR_CHECK(err_code);
//	NRF_LOG_INFO("--1A2");
//	err_code = nrf_serial_flush(&serial_uart, 0);
//	APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("serial_bytes_written: %d", serial_bytes_written);
	NRF_LOG_INFO("--1B");
//	NRF_LOG_INFO("--1B2");

//	err_code = nrf_serial_rx_drain(&serial_uart);
//    nrf_delay_ms(10);


//    // Check if the HPM acknowledged
    char hpm_ack_rx[2] = {0x0, 0x0 };
	size_t serial_bytes_read;
    err_code = nrf_serial_read(&serial_uart, hpm_ack_rx, sizeof(hpm_ack_rx), &serial_bytes_read, HPM_SERIAL_TIMEOUT);
//    NRF_LOG_INFO("serial_bytes_read: %d", serial_bytes_read);
//    NRF_LOG_INFO("hpm_ack_rx: 0x%x", hpm_ack_rx);
    NRF_LOG_INFO("hpm_ack_rx: 0x%x, 0x%x", hpm_ack_rx[0], hpm_ack_rx[1]);
    NRF_LOG_INFO("err_code: %d", err_code);
    APP_ERROR_CHECK(err_code);


//    // For TESTING
//    char test_rx;
//    for (int i=0; i < 8; i++) {
////    while (!err_code) {
//    	NRF_LOG_INFO("i: %d", i);
//    	NRF_LOG_INFO("err_code: %d", err_code);
//    	err_code = nrf_serial_read(&serial_uart, &test_rx, sizeof(test_rx), &serial_bytes_read, HPM_SERIAL_TIMEOUT);
//    	NRF_LOG_INFO("--1C");
//        NRF_LOG_INFO("serial_bytes_read: %d", serial_bytes_read);
//		NRF_LOG_INFO("test_rx: 0x%x", test_rx);
////		break;	// FOR TESTING
//
//    }

//	nrf_delay_ms(1000);
//	err_code = nrf_serial_rx_drain(&serial_uart);
//    APP_ERROR_CHECK(err_code);


//    // Check if the HPM acknowledged
//    char hpm_ack_rx = 0x0;
//    int found_A5 = 0;
//    while (!found_A5) {
//    	err_code = nrf_serial_read(&serial_uart, &hpm_ack_rx, sizeof(hpm_ack_rx), &serial_bytes_read, HPM_SERIAL_TIMEOUT);
//        APP_ERROR_CHECK(err_code);
//        NRF_LOG_INFO("hpm_ack_rx: 0x%x", hpm_ack_rx);
//        if (hpm_ack_rx == 0xA5 || hpm_ack_rx == 0x0) {
//        	found_A5 = 1;	// we found it!
//        	// Read the second one
//        	err_code = nrf_serial_read(&serial_uart, &hpm_ack_rx, sizeof(hpm_ack_rx), &serial_bytes_read, HPM_SERIAL_TIMEOUT);
//            APP_ERROR_CHECK(err_code);
//        } else if (hpm_ack_rx == 0x0) {
//        	found_A5 = 1;	// we found it!
//        }
//        // Otherwise keep searching
////        NRF_LOG_INFO("hpm_ack_rx: 0x%x", hpm_ack_rx);
//    }


//    // Check if the HPM acknowledged
//    char hpm_ack_rx[40];
//	err_code = nrf_serial_read(&serial_uart, &hpm_ack_rx, sizeof(hpm_ack_rx), &serial_bytes_read, HPM_SERIAL_TIMEOUT);
//	APP_ERROR_CHECK(err_code);
//	for (int i=0; i < serial_bytes_read; i++) {
//		NRF_LOG_INFO("hpm_ack_rx[i]: 0x%x", hpm_ack_rx[i]);
//	}



    NRF_LOG_INFO("hpm_ack_rx: 0x%x, 0x%x", hpm_ack_rx[0], hpm_ack_rx[1]);
//    NRF_LOG_INFO("hpm_ack_rx: 0x%x, 0x%x", hpm_ack_rx, hpm_ack_rx);
//	NRF_LOG_INFO("--3");

    // TODO: make this check if 0xA5A5 was received, also check bytes read/written match
    return err_code;

}

int hpm_read_meas() {

//    ret_code_t err_code;
    size_t serial_bytes_written;
    size_t serial_bytes_read;
	hpm_2_5_value = 0;
	hpm_10_value = 0;

    // Don't use autosend; should be more robust, but HPM is weird when power cycled
    if (!USING_AUTOSEND) {
		//TODO: use a static char[] and memset to clear each time (be careful memset does same size)
		char hpm_buff[HPM_BUFF_SIZE];		// this initializes everything as 0 (first one is 0, then fills remainder with 0)

		// Clear the RX from previous
//		nrf_delay_ms(1000);
		err_code = nrf_serial_rx_drain(&serial_uart);
		NRF_LOG_INFO("err_code: %d", hpm_read_meas_cmd);
	    APP_ERROR_CHECK(err_code);
//		nrf_delay_ms(1000);



		// Sending the read_meas command
		NRF_LOG_INFO("hpm_read_meas_cmd: 0x%x", hpm_read_meas_cmd);
		err_code = nrf_serial_write(&serial_uart, hpm_read_meas_cmd, HPM_CMD_LEN, &serial_bytes_written, HPM_SERIAL_TIMEOUT);
//		err_code = nrf_serial_write(&serial_uart, hpm_read_meas_cmd, HPM_CMD_LEN, &serial_bytes_written, 0);
	//    NRF_LOG_INFO("serial_bytes_written: %d", serial_bytes_written);
		APP_ERROR_CHECK(err_code);
//		err_code = nrf_serial_flush(&serial_uart, 0);
//		APP_ERROR_CHECK(err_code);

		NRF_LOG_INFO("--2A");


//		// For TESTING
//		char test_rx;
//		for (int i=0; i < 8; i++) {
//	//    while (!err_code) {
//			NRF_LOG_INFO("i: %d", i);
//			NRF_LOG_INFO("err_code: %d", err_code);
//			err_code = nrf_serial_read(&serial_uart, &test_rx, sizeof(test_rx), &serial_bytes_read, HPM_SERIAL_TIMEOUT);
//			NRF_LOG_INFO("--2T");
//			NRF_LOG_INFO("serial_bytes_read: %d", serial_bytes_read);
//			NRF_LOG_INFO("test_rx: 0x%x", test_rx);
//	//		break;	// FOR TESTING
//
//		}



//		// Check if the HPM acknowledged
//		char hpm_ack_rx[2] = {0x0, 0x0 };
//		err_code = nrf_serial_read(&serial_uart, hpm_ack_rx, sizeof(hpm_ack_rx), &serial_bytes_read, HPM_SERIAL_TIMEOUT);
//	//    NRF_LOG_INFO("serial_bytes_read: %d", serial_bytes_read);
//	//    NRF_LOG_INFO("hpm_ack_rx: 0x%x", hpm_ack_rx);
//		NRF_LOG_INFO("hpm_ack_rx: 0x%x, 0x%x", hpm_ack_rx[0], hpm_ack_rx[1]);

		NRF_LOG_INFO("--2B");

//		nrf_delay_ms(1000);

		// Read back all data into a buffer
//		err_code = nrf_serial_read(&serial_uart, hpm_buff, HPM_BUFF_SIZE, &serial_bytes_read, HPM_SERIAL_TIMEOUT);
	//    NRF_LOG_INFO("serial_bytes_read: %d", serial_bytes_read);
	//    NRF_LOG_INFO("hpm_ack_rx: %s", hpm_ack_rx);
	    for(int i = 0; i < HPM_BUFF_SIZE; i++) {

	        NRF_LOG_INFO("i: %d", i);
			err_code = nrf_serial_read(&serial_uart, &hpm_buff[i], sizeof(char), &serial_bytes_read, HPM_SERIAL_TIMEOUT);
	        NRF_LOG_INFO("err_code: %d", err_code);
			APP_ERROR_CHECK(err_code);
	        NRF_LOG_INFO("0x%x", hpm_buff[i]);
	    }
		APP_ERROR_CHECK(err_code);

		// calc values
		hpm_2_5_value = 256*hpm_buff[3] + hpm_buff[4];
		hpm_10_value = 256*hpm_buff[5] + hpm_buff[6];
		NRF_LOG_INFO("hpm_2_5_value: %d", hpm_2_5_value);
	//    NRF_LOG_INFO("hpm_2_5_value: %d", twosComp((uint8_t) hpm_buff[3],(uint8_t) hpm_buff[4]));
		NRF_LOG_INFO("hpm_10_value: %d", hpm_10_value);

		// calc checksum
		int checksum_calc = 0;
		for(int i = 0; i < HPM_BUFF_SIZE - 1; i++) {	// everything except last one (checksum value)
			checksum_calc += hpm_buff[i];
		}
		checksum_calc = (65536 - checksum_calc) % 256;
	    NRF_LOG_INFO("checksum_calc: 0x%x", checksum_calc);

		// TODO: make this return a proper success code
		if (checksum_calc == hpm_buff[HPM_BUFF_SIZE - 1]) {
	//        NRF_LOG_INFO("HPM checksum SUCCESS!");
			return 0;
		} else {
			NRF_LOG_INFO("HPM checksum ERROR!");
			return 1;
		}
    }

    // Reading using HPM's Autosend
    else {
		NRF_LOG_INFO("USING AUTOSEND");

		//TODO: use a static char[] and memset to clear each time (be careful memset does same size)
		char hpm_buff[HPM_BUFF_SIZE];		// this initializes everything as 0 (first one is 0, then fills remainder with 0)
		int found_header = 0;

//		// Drain the receive buffer
//		err_code = nrf_serial_rx_drain(&serial_uart);
//		NRF_LOG_INFO("err_code: %d", err_code);
//	    APP_ERROR_CHECK(err_code);

		// Read back all data into a buffer
		for (int i=0; i < NUM_AUTOSEND_READ_TRIES; i++) {
//			NRF_LOG_INFO("i: %d", i);
			err_code = nrf_serial_read(&serial_uart, &hpm_buff[0], sizeof(char), &serial_bytes_read, HPM_SERIAL_TIMEOUT);
//			NRF_LOG_INFO("err_code: %d", err_code);
		//    NRF_LOG_INFO("serial_bytes_read: %d", serial_bytes_read);
		//    NRF_LOG_INFO("hpm_ack_rx: %s", hpm_ack_rx);
		//    for(int i = 0; i < HPM_BUFF_SIZE; i++) {
		//        NRF_LOG_INFO("0x%x", hpm_buff[i]);
		//    }
			// If it timed out, return and handle it there
			if (err_code == NRF_ERROR_TIMEOUT) return err_code;
			APP_ERROR_CHECK(err_code);

//			NRF_LOG_INFO("i: %d", i);

			// Check if we found the header (then start reading, or continue and keep looking)
			if (hpm_buff[0] == 0x42) {
//				NRF_LOG_INFO("hpm_buff[0]: 0x%x", hpm_buff[0]);

				// Read the next one and verify it's the next header
				err_code = nrf_serial_read(&serial_uart, &hpm_buff[1], sizeof(char), &serial_bytes_read, HPM_SERIAL_TIMEOUT);
//				NRF_LOG_INFO("err_code: %d", err_code);
				// If it timed out, return and handle it there TODO: make this more robust
				if (err_code == NRF_ERROR_TIMEOUT) return err_code;
				APP_ERROR_CHECK(err_code);
				if (hpm_buff[1] == 0x4D) {
//					NRF_LOG_INFO("hpm_buff[1]: 0x%x", hpm_buff[1]);
					found_header = 1;

//					// For TESTING
//					char test_read;
//					for (int j=0; j < 8; j++) {
//						NRF_LOG_INFO("j: %d", j);
//
//						err_code = nrf_serial_read(&serial_uart, &test_read, sizeof(char), &serial_bytes_read, HPM_SERIAL_TIMEOUT);
//
//						NRF_LOG_INFO("test_read: 0x%x", test_read);
//					}


					break;	// Move on to reading the data
				}
			} else {
				// Keep looking for header
				NRF_LOG_INFO("HAVEN'T FOUND HEADER YET..");

			}

		}

//		// For TESTING
//		char test_read;
//		NRF_LOG_INFO("HPM_BUFF_SIZE: %d", HPM_BUFF_SIZE);
//		for (int i=2; i < HPM_BUFF_SIZE; i++) {
////			err_code = nrf_serial_read(&serial_uart, &hpm_buff[i], sizeof(char), &serial_bytes_read, HPM_SERIAL_TIMEOUT);
//			err_code = nrf_serial_read(&serial_uart, &test_read, sizeof(char), &serial_bytes_read, HPM_SERIAL_TIMEOUT);
//			APP_ERROR_CHECK(err_code);
//
//			NRF_LOG_INFO("i: %d", i);
////			NRF_LOG_INFO("hpm_buff[i]: 0x%x", hpm_buff[i]);
//			NRF_LOG_INFO("test_read: 0x%x", test_read);
//
//
//		}



		// If we found the header, read the rest
		if (found_header) {
			NRF_LOG_INFO("FOUND HEADER..");


			err_code = nrf_serial_read(&serial_uart, &hpm_buff[2], HPM_BUFF_SIZE - 2, &serial_bytes_read, HPM_SERIAL_TIMEOUT);
			// If it timed out, return and handle it there TODO: make this more robust
			NRF_LOG_INFO("err_code: %d", err_code);
			if (err_code == NRF_ERROR_TIMEOUT) return err_code;
			APP_ERROR_CHECK(err_code);

//			NRF_LOG_INFO("T2");

			// calc values
			hpm_2_5_value = 256*hpm_buff[6] + hpm_buff[7];
			hpm_10_value = 256*hpm_buff[8] + hpm_buff[9];
			NRF_LOG_INFO("hpm_2_5_value: %d", hpm_2_5_value);
		//    NRF_LOG_INFO("hpm_2_5_value: %d", twosComp((uint8_t) hpm_buff[3],(uint8_t) hpm_buff[4]));
			NRF_LOG_INFO("hpm_10_value: %d", hpm_10_value);

			// calc checksum
			int checksum_calc = 0;
			for(int i = 0; i < HPM_BUFF_SIZE - 2; i++) {	// everything except last one (checksum value)
				checksum_calc += hpm_buff[i];
			}
			int checksum_value = 256*hpm_buff[HPM_BUFF_SIZE-2] + hpm_buff[HPM_BUFF_SIZE-1];

//			checksum_calc = (65536 - checksum_calc) % 256;
//		    NRF_LOG_INFO("checksum_calc: 0x%x", checksum_calc);
//		    NRF_LOG_INFO("checksum_value: 0x%x", checksum_value);
//		    NRF_LOG_INFO("hpm_buff[HPM_BUFF_SIZE-2]: 0x%x", hpm_buff[HPM_BUFF_SIZE-2]);
//		    NRF_LOG_INFO("hpm_buff[HPM_BUFF_SIZE-1]: 0x%x", hpm_buff[HPM_BUFF_SIZE-1]);

			// TODO: make this return a proper success code
			if (checksum_calc == checksum_value) {
		//        NRF_LOG_INFO("HPM checksum SUCCESS!");
				return 0;
			} else {
				NRF_LOG_INFO("** ERROR: HPM checksum ERROR! **");
				return 2;
			}
		} else {
			NRF_LOG_INFO("** ERROR: HPM never found Header! **");
			return 1;
		}


    }


    // TODO: make this check if 0x400504 was received, also check bytes read/written match
    return 2;



}





// Dummy function for argument, Taken from peripheral/saadc
void timer_handler(nrf_timer_event_t event_type, void * p_context)
{

}


// Taken from example_code/saadc_simpler
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
//        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

//        NRF_LOG_INFO("%d\r\n", p_event->data.done.p_buffer[0]);
        for (int i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            NRF_LOG_INFO("saadc_callback (should NOT be here): %d", p_event->data.done.p_buffer[i]);
        }

    }
}

// Taken from example_code/saadc_simpler
// TODO:  init and uninit each channel as you use them.
void saadc_init(void)
{
	// Calculate adc_to_V (couldn't do this up top since C is dumb)
	adc_to_V = 1.0f / ((ADC_GAIN_VALUE / ADC_REFERENCE_VOLTAGE) * (pow(2, ADC_RESOLUTION_BITS)-1) );

//    ret_code_t err_code;
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    // Default channel config: internal reference 0.6V, 1/6 gain, 10us acq_time,
    nrf_saadc_channel_config_t adc_channel_config
        = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(ADC_PIN);
    err_code = nrf_drv_saadc_channel_init(ADC_CHANNEL_NUM, &adc_channel_config);
    APP_ERROR_CHECK(err_code);

    // Sharp PM channel config: internal reference 0.6V, 1/6 gain, 10us acq_time,
    nrf_saadc_channel_config_t sharpPM_channel_config
        = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(SHARP_PM_PIN);
    err_code = nrf_drv_saadc_channel_init(SHARP_PM_CHANNEL_NUM, &sharpPM_channel_config);
    APP_ERROR_CHECK(err_code);

    // Spec CO channel config: internal reference 0.6V, 1/6 gain, 10us acq_time,
    nrf_saadc_channel_config_t specCO_channel_config
        = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(SPEC_CO_PIN);
    err_code = nrf_drv_saadc_channel_init(SPEC_CO_CHANNEL_NUM, &specCO_channel_config);
    APP_ERROR_CHECK(err_code);

    // Figaro CO channel config: internal reference 0.6V, 1/6 gain, 10us acq_time,
    nrf_saadc_channel_config_t figCO_channel_config
        = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(FIG_CO_PIN);
    err_code = nrf_drv_saadc_channel_init(FIG_CO_CHANNEL_NUM, &figCO_channel_config);
    APP_ERROR_CHECK(err_code);

    // Battery Check channel config: internal reference 0.6V, 1/6 gain, 10us acq_time,
    nrf_saadc_channel_config_t battery_channel_config
        = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(BATTERY_PIN);
    err_code = nrf_drv_saadc_channel_init(BATTERY_CHANNEL_NUM, &battery_channel_config);
    APP_ERROR_CHECK(err_code);


}









///**
// * @brief Function for handling data from temperature sensor.
// *
// * @param[in] temp          Temperature in Celsius degrees read from sensor.
// */
//__STATIC_INLINE void data_handler_TWI(uint8_t temp)
//{
//    NRF_LOG_INFO("Temperature: %d Celsius degrees.", temp);
//}
//
///**
// * @brief TWI events handler.
// */
//void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
//{
//    switch (p_event->type)
//    {
//        case NRF_DRV_TWI_EVT_DONE:
//            NRF_LOG_INFO("In twi_handler...");
//            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
//            {
//                data_handler_TWI(figCO2_value);
//            }
//            m_xfer_done = true;
//            break;
//        default:
//            break;
//    }
//}


/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
//    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_figCO2_config = {
       .scl                = TWI_SCL_PIN,
       .sda                = TWI_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    // Maybe don't want handler??
    // From H file:
    // @param[in] event_handler   Event handler provided by the user. If NULL, blocking mode is enabled.
//    err_code = nrf_drv_twi_init(&m_twi, &twi_figCO2_config, twi_handler, NULL);
    err_code = nrf_drv_twi_init(&m_twi, &twi_figCO2_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

// 2's Complement
int twosComp( uint8_t bit_msb, uint8_t bit_lsb)
{
    int16_t myInt=0;
    myInt = (bit_msb << 8) | (bit_lsb & 0xff);
//    NRF_LOG_INFO("myInt: 0x%x", myInt);
//    NRF_LOG_INFO("myInt: %d", myInt);

    return myInt;
}


// Read Figaro CO2 with TWI (I2C)
static ret_code_t read_figCO2()
{
    uint8_t readme_msb;
    uint8_t readme_lsb;
//    ret_code_t err_code;

    uint8_t reg = 0x03;
	err_code = nrf_drv_twi_tx(&m_twi, figCO2_addr, &reg, 1, true);
	NRF_LOG_INFO("err_code: %d", err_code);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_twi_rx(&m_twi, figCO2_addr, &readme_lsb, 1);
	NRF_LOG_INFO("err_code: %d", err_code);
    APP_ERROR_CHECK(err_code);
//    nrf_delay_ms(FIG_CO2_XFER_WAIT_TIME);

//	NRF_LOG_INFO("TEST 05");


    reg = 0x04;
    err_code = nrf_drv_twi_tx(&m_twi, figCO2_addr, &reg, 1, true);
    APP_ERROR_CHECK(err_code);
//    nrf_delay_ms(FIG_CO2_XFER_WAIT_TIME);

    err_code = nrf_drv_twi_rx(&m_twi, figCO2_addr, &readme_msb, 1);
    APP_ERROR_CHECK(err_code);
//    nrf_delay_ms(FIG_CO2_XFER_WAIT_TIME);

//	NRF_LOG_INFO("readme_msb: 0x%x", readme_msb);
//	NRF_LOG_INFO("readme_lsb: 0x%x", readme_lsb);

    figCO2_value = twosComp(readme_msb,readme_lsb);

    return err_code;

}


// Compensate the raw BME280 values using calibration values
static void bme_read_calib_data() {

	uint8_t cmd[18];
	uint8_t reg0;

	// Get all the temp calib data
    reg0 = 0x88;
    err_code = nrf_drv_twi_tx(&m_twi, bme_addr, &reg0, 1, true);
    APP_ERROR_CHECK(err_code);
    // Read the value from that register
	err_code = nrf_drv_twi_rx(&m_twi, bme_addr, cmd, 6);
	APP_ERROR_CHECK(err_code);
	// Convert the values
    dig_T1 = (cmd[1] << 8) | cmd[0];
    dig_T2 = (cmd[3] << 8) | cmd[2];
    dig_T3 = (cmd[5] << 8) | cmd[4];

	// Get all the pressure calib data
    reg0 = 0x8E;
    err_code = nrf_drv_twi_tx(&m_twi, bme_addr, &reg0, 1, true);
    APP_ERROR_CHECK(err_code);
    // Read the value from that register
	err_code = nrf_drv_twi_rx(&m_twi, bme_addr, cmd, 18);
	APP_ERROR_CHECK(err_code);
	// Convert the values
    dig_P1 = (cmd[ 1] << 8) | cmd[ 0];
    dig_P2 = (cmd[ 3] << 8) | cmd[ 2];
    dig_P3 = (cmd[ 5] << 8) | cmd[ 4];
    dig_P4 = (cmd[ 7] << 8) | cmd[ 6];
    dig_P5 = (cmd[ 9] << 8) | cmd[ 8];
    dig_P6 = (cmd[11] << 8) | cmd[10];
    dig_P7 = (cmd[13] << 8) | cmd[12];
    dig_P8 = (cmd[15] << 8) | cmd[14];
    dig_P9 = (cmd[17] << 8) | cmd[16];

	// Get all the humidity calib data
    reg0 = 0xA1;
    err_code = nrf_drv_twi_tx(&m_twi, bme_addr, &reg0, 1, true);
    APP_ERROR_CHECK(err_code);
    // Read the value from that register
	err_code = nrf_drv_twi_rx(&m_twi, bme_addr, cmd, 1);
	APP_ERROR_CHECK(err_code);
	reg0 = 0xE1;
    err_code = nrf_drv_twi_tx(&m_twi, bme_addr, &reg0, 1, true);
    APP_ERROR_CHECK(err_code);
    // Read the value from that register
	err_code = nrf_drv_twi_rx(&m_twi, bme_addr, &cmd[1], 7);
	APP_ERROR_CHECK(err_code);
	// Convert the values
    dig_H1 = cmd[0];
    dig_H2 = (cmd[2] << 8) | cmd[1];
    dig_H3 = cmd[3];
    dig_H4 = (cmd[4] << 4) | (cmd[5] & 0x0f);
    dig_H5 = (cmd[6] << 4) | ((cmd[5]>>4) & 0x0f);
    dig_H6 = cmd[7];

}


// Read BME280 TRH with TWI (I2C)
static ret_code_t bme_init() {

	// Read and store calibration data once for all future reads
	if (!has_read_calib_data) {
		bme_read_calib_data();
		has_read_calib_data = 1;
	}

//	uint8_t bmebuff[BME_BUFF_SIZE];
	uint8_t cmd[2];
//    ret_code_t err_code;
//    uint8_t reg_test = 0x00;

	// Starting the measurements by editing control registers
    cmd[0] = 0xf2; // ctrl_hum
    cmd[1] = 0x01; // Humidity oversampling x1
    err_code = nrf_drv_twi_tx(&m_twi, bme_addr, cmd, 2, true);
    APP_ERROR_CHECK(err_code);

    cmd[0] = 0xf4; // ctrl_meas
    cmd[1] = 0x25; // Temparature oversampling x1, Pressure oversampling x1, Force mode (reads, then goes to standby)
    err_code = nrf_drv_twi_tx(&m_twi, bme_addr, cmd, 2, true);
    APP_ERROR_CHECK(err_code);

//	uint8_t readreg;
////	uint8_t readreg_changed;
//	uint8_t regt;
//
//    // Check ctrl_hum register
//    regt = 0xF2;
//    err_code = nrf_drv_twi_tx(&m_twi, bme_addr, &regt, 1, true);
//    APP_ERROR_CHECK(err_code);
//	err_code = nrf_drv_twi_rx(&m_twi, bme_addr, &readreg, 1);
//    APP_ERROR_CHECK(err_code);
//    NRF_LOG_INFO("readreg 0x%x: 0x%x", regt, readreg);
//    // Set osrs_h to oversample=1x (needed to turn on)
//    readreg |= 1UL;
//	uint8_t reg_h[2] = {regt,	readreg};
//	err_code = nrf_drv_twi_tx(&m_twi, bme_addr, reg_h, sizeof(reg_h), false);
//	APP_ERROR_CHECK(err_code);
//	// Reread register to verify change
//    err_code = nrf_drv_twi_tx(&m_twi, bme_addr, &regt, 1, true);
//    APP_ERROR_CHECK(err_code);
//	err_code = nrf_drv_twi_rx(&m_twi, bme_addr, &readreg, 1);
//    APP_ERROR_CHECK(err_code);
//    NRF_LOG_INFO("readreg 0x%x: 0x%x", regt, readreg);
//
//    // Check ctrl_meas register
//    regt = 0xF4;
//    err_code = nrf_drv_twi_tx(&m_twi, bme_addr, &regt, 1, true);
//    APP_ERROR_CHECK(err_code);
//	err_code = nrf_drv_twi_rx(&m_twi, bme_addr, &readreg, 1);
//    APP_ERROR_CHECK(err_code);
//    NRF_LOG_INFO("readreg 0x%x: 0x%x", regt, readreg);
//    // Set temp and pressure to oversample, set mode to Force
//	uint8_t reg_meas[2] = {regt,	0x25};
//	err_code = nrf_drv_twi_tx(&m_twi, bme_addr, reg_meas, sizeof(reg_meas), false);
//	APP_ERROR_CHECK(err_code);

    return err_code;

}



// Read BME280 TRH with TWI (I2C)
static ret_code_t read_BME()
{



	// Wait for the measurements to end
//	nrf_delay_ms(10);

	uint8_t bmebuff[BME_BUFF_SIZE];

	// Tell it which register we want to start reading from
    uint8_t reg0 = 0xF7;
    err_code = nrf_drv_twi_tx(&m_twi, bme_addr, &reg0, 1, true);
    APP_ERROR_CHECK(err_code);
    // Read the value from that register
	err_code = nrf_drv_twi_rx(&m_twi, bme_addr, bmebuff, BME_BUFF_SIZE);
	APP_ERROR_CHECK(err_code);


//	// Read the bme registers and store them
//	for (uint8_t reg = 0x00; reg < BME_BUFF_SIZE; reg++) {
////		// Tell it which register we want
////		err_code = nrf_drv_twi_tx(&m_twi, bme_addr, &reg, 1, true);
////		APP_ERROR_CHECK(err_code);
////		// Read the value from that register
////		err_code = nrf_drv_twi_rx(&m_twi, bme_addr, &readreg, 1);
////		APP_ERROR_CHECK(err_code);
////		// Convert the value
////		NRF_LOG_INFO("readreg: 0x%x", readreg);
//		NRF_LOG_INFO("bmebuff[0x%x]: 0x%x, %d", reg, bmebuff[reg], bmebuff[reg]);
//	}


	// Convert and store the values (uncompensated)
	uint32_t press_raw = 	(bmebuff[0] << 12) | (bmebuff[1] << 4) | (bmebuff[2] >> 4);
	uint32_t temp_raw = 	(bmebuff[3] << 12) | (bmebuff[4] << 4) | (bmebuff[5] >> 4);
	uint32_t hum_raw = 		(bmebuff[6] << 8) | bmebuff[7];



	// Compensate the values (using internal calibration values)

	// Compensate Temp
    int32_t temp;
	temp =
	        (((((temp_raw >> 3) - (dig_T1 << 1))) * dig_T2) >> 11) +
	        ((((((temp_raw >> 4) - dig_T1) * ((temp_raw >> 4) - dig_T1)) >> 12) * dig_T3) >> 14);
	t_fine = temp;
	bme_temp_C = (temp * 5 + 128) >> 8;


	// Compensate Pressure
	int32_t var1, var2;
	uint32_t press;

	var1 = (t_fine >> 1) - 64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * dig_P6;
	var2 = var2 + ((var1 * dig_P5) << 1);
	var2 = (var2 >> 2) + (dig_P4 << 16);
	var1 = (((dig_P3 * (((var1 >> 2)*(var1 >> 2)) >> 13)) >> 3) + ((dig_P2 * var1) >> 1)) >> 18;
	var1 = ((32768 + var1) * dig_P1) >> 15;
	if (var1 == 0) {
		return NRF_ERROR_INVALID_DATA;
	}
	press = (((1048576 - press_raw) - (var2 >> 12))) * 3125;
	if(press < 0x80000000) {
		press = (press << 1) / var1;
	} else {
		press = (press / var1) * 2;
	}
	var1 = ((int32_t)dig_P9 * ((int32_t)(((press >> 3) * (press >> 3)) >> 13))) >> 12;
	var2 = (((int32_t)(press >> 2)) * (int32_t)dig_P8) >> 13;
	bme_pressure = (press + ((var1 + var2 + dig_P7) >> 4));


	// Compensate Humidity
	int32_t v_x1;

	v_x1 = t_fine - 76800;
	v_x1 =  (((((hum_raw << 14) -(((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1)) +
			   ((int32_t)16384)) >> 15) * (((((((v_x1 * (int32_t)dig_H6) >> 10) *
											(((v_x1 * ((int32_t)dig_H3)) >> 11) + 32768)) >> 10) + 2097152) *
											(int32_t)dig_H2 + 8192) >> 14));
	v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * (int32_t)dig_H1) >> 4));
	v_x1 = (v_x1 < 0 ? 0 : v_x1);
	v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);

	bme_humidity = (v_x1 >> 12)*1000/1024;



    return err_code;

}




//// Read BME Temp and convert it with calib data
//static ret_code_t bme_get_temp() {
//
//    uint32_t temp_raw;
//    float tempf;
//    uint8_t cmd[4];
//
//	reg0 = 0xfa;
//    err_code = nrf_drv_twi_tx(&m_twi, bme_addr, &reg0, 1, true);
//    APP_ERROR_CHECK(err_code);
//    // Read the value from that register
//	err_code = nrf_drv_twi_rx(&m_twi, bme_addr, &cmd[1], 3);
//	APP_ERROR_CHECK(err_code);
//
////    cmd[0] = 0xfa; // temp_msb
////    i2c.write(address, cmd, 1);
////    i2c.read(address, &cmd[1], 3);
//
//    temp_raw = (cmd[1] << 12) | (cmd[2] << 4) | (cmd[3] >> 4);
//
//    int32_t temp;
//
//    temp =
//        (((((temp_raw >> 3) - (dig_T1 << 1))) * dig_T2) >> 11) +
//        ((((((temp_raw >> 4) - dig_T1) * ((temp_raw >> 4) - dig_T1)) >> 12) * dig_T3) >> 14);
//
//    t_fine = temp;
//    temp = (temp * 5 + 128) >> 8;
//    tempf = (float)temp;
//
//    return (tempf/100.0f);
//
//}


// Read RTC crystal with TWI (I2C) TODO: pass TWI address as argument
static ret_code_t read_rtc()
{
	uint8_t readreg;
	uint8_t rtcbuff[RTC_BUFF_SIZE];
//    ret_code_t err_code;
//    uint8_t reg_test = 0x00;


//    // Just try reading a bunch of times and see what comes
//    for (int i = 0; i < 10; i++) {
//		err_code = nrf_drv_twi_rx(&m_twi, rtc_addr, &readreg, 1);
//		APP_ERROR_CHECK(err_code);
//		NRF_LOG_INFO("i: %d", i);
//		NRF_LOG_INFO("readreg: 0x%x", readreg);
//		NRF_LOG_INFO("converted: %d", readreg - 6 * (readreg >> 4));
//    }

//    // Try overwriting the CH clock halt (pause) bit
//    uint8_t regCH[2] = {0x00,	0x47};
//    err_code = nrf_drv_twi_tx(&m_twi, rtc_addr, regCH, sizeof(regCH), false);
//    APP_ERROR_CHECK(err_code);


//    // Gives some time for NRF_LOG_INFO, otherwise RTT can't keep up fast enough
//    nrf_delay_ms(5);
//	NRF_LOG_INFO("");

    // Checking the first register, make sure it's running (CH bit == 0)
    uint8_t regt = 0x00;
//	NRF_LOG_INFO("read_rtc(): BEFORE TX");
//	nrf_delay_ms(500);
    err_code = nrf_drv_twi_tx(&m_twi, rtc_addr, &regt, 1, true);
//	NRF_LOG_INFO("err_code: %d", err_code);
//	nrf_delay_ms(1000);
    APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(&m_twi, rtc_addr, &readreg, 1);
//	NRF_LOG_INFO("err_code: %d", err_code);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("readreg 0x%x: 0x%x", regt, readreg);
//    NRF_LOG_INFO("converted: %d", readreg - 6 * (readreg >> 4));
    uint8_t is_running = !(readreg>>7);		// TODO: add a check and correction action here
    NRF_LOG_INFO("is_running: %d", is_running );


    // Check another register that the change was made
    regt = 0x0e;
    err_code = nrf_drv_twi_tx(&m_twi, rtc_addr, &regt, 1, true);
    APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(&m_twi, rtc_addr, &readreg, 1);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("readreg 0x%x: 0x%x", regt, readreg);


    // Turn off the square wave, it's not needed and wastes power
    regt = 0x0f;
    err_code = nrf_drv_twi_tx(&m_twi, rtc_addr, &regt, 1, true);
    APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(&m_twi, rtc_addr, &readreg, 1);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("readreg 0x%x: 0x%x", regt, readreg);
    // Turn OFF the specific bits
    readreg &= ~(1UL << 3);	// EN32KHZ
    readreg &= ~(1UL << 6);	// BB32KHZ
    NRF_LOG_INFO("new readreg 0x%x: 0x%x", regt, readreg);
    // Write to the register
	uint8_t sqwv_off_cmd[2] = {regt,	readreg};
	err_code = nrf_drv_twi_tx(&m_twi, rtc_addr, sqwv_off_cmd, sizeof(sqwv_off_cmd), false);
	APP_ERROR_CHECK(err_code);

    // Check another register that the change was made
    regt = 0x0e;
    err_code = nrf_drv_twi_tx(&m_twi, rtc_addr, &regt, 1, true);
    APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(&m_twi, rtc_addr, &readreg, 1);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("readreg 0x%x: 0x%x", regt, readreg);

    // Check that the change was made
    regt = 0x0f;
    err_code = nrf_drv_twi_tx(&m_twi, rtc_addr, &regt, 1, true);
    APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(&m_twi, rtc_addr, &readreg, 1);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("readreg 0x%x: 0x%x", regt, readreg);




//    // Read the RTC registers and store them
//    for (uint8_t reg = 0x00; reg < RTC_BUFF_SIZE; reg++) {
//    	// Tell it which register we want
//        err_code = nrf_drv_twi_tx(&m_twi, rtc_addr, &reg, 1, true);
//        APP_ERROR_CHECK(err_code);
//        // Read the value from that register
//		err_code = nrf_drv_twi_rx(&m_twi, rtc_addr, &readreg, 1);
//		APP_ERROR_CHECK(err_code);
//		// Convert the value
//		rtcbuff[reg] = readreg - 6 * (readreg >> 4);
//	    NRF_LOG_INFO("readreg: 0x%x", readreg);
//	    NRF_LOG_INFO("rtcbuff[0x%x]: 0x%x, %d", reg, rtcbuff[reg], rtcbuff[reg]);
//    }


	// Tell it which register we want to start reading from
    uint8_t reg0 = 0x00;
    err_code = nrf_drv_twi_tx(&m_twi, rtc_addr, &reg0, 1, true);
    APP_ERROR_CHECK(err_code);
    // Read the value from that register
	err_code = nrf_drv_twi_rx(&m_twi, rtc_addr, rtcbuff, RTC_BUFF_SIZE);
	APP_ERROR_CHECK(err_code);

	// Convert bcd to dec, and print out the values
	for (int i = 0; i < RTC_BUFF_SIZE; i++) {
	    rtcbuff[i] = rtcbuff[i] - 6 * (rtcbuff[i] >> 4);
//	    NRF_LOG_INFO("rtcbuff[%d]: %d", i, rtcbuff[i]);
    }

    // Convert and store
    struct tm t;
    t.tm_year = (int)rtcbuff[6]+100;
    t.tm_mon = (int)rtcbuff[5]-1;           // Month, 0 - jan
    t.tm_mday = (int)rtcbuff[4];          // Day of the month
    t.tm_hour = (int)rtcbuff[2];
    t.tm_min = (int)rtcbuff[1];
    t.tm_sec = (int)rtcbuff[0];
    t.tm_isdst = 0;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
    timeNow = mktime(&t);


//    // Gives some time for NRF_LOG_INFO, otherwise RTT can't keep up fast enough
//    nrf_delay_ms(5);

//    // testing some stuff
//    NRF_LOG_INFO("readreg: 0x%x", readreg);
//    readreg = readreg - 6 * (readreg >> 4);
//    NRF_LOG_INFO("converted readreg: %d", readreg);
//    NRF_LOG_INFO("timeNow: %d", timeNow);



    return err_code;

}


// Read Small Plantower with TWI (I2C)
static ret_code_t read_plantower_twi()
{
	uint8_t plantower_buff[PLANTOWER_TWI_BUFF_SIZE];
//    ret_code_t err_code;
//    uint8_t reg_test = 0x00;


	// Tell it which register we want to start reading from
    uint8_t reg0 = 0x00;
//    NRF_LOG_INFO("BEFORE TX");
    err_code = nrf_drv_twi_tx(&m_twi, plantower_twi_addr, &reg0, 1, true);
    NRF_LOG_INFO("AFTER TX, err_code: %d", err_code);
    APP_ERROR_CHECK(err_code);
    // Read the value from that register
//    NRF_LOG_INFO("BEFORE RX");
	err_code = nrf_drv_twi_rx(&m_twi, plantower_twi_addr, plantower_buff, PLANTOWER_TWI_BUFF_SIZE);
    NRF_LOG_INFO("AFTER RX, err_code: %d", err_code);
	APP_ERROR_CHECK(err_code);

//	// For TESTING
//	for (int i = 0; i < PLANTOWER_TWI_BUFF_SIZE; i++) {
//	    NRF_LOG_INFO("plantower_buff[%d]: %d", i, plantower_buff[i]);
//    }


	// calc values
	plantower_2_5_value = 256*plantower_buff[6] + plantower_buff[7];
	plantower_10_value = 256*plantower_buff[8] + plantower_buff[9];
//	NRF_LOG_INFO("plantower_2_5_value: %d", plantower_2_5_value);
//    NRF_LOG_INFO("plantower_2_5_value: %d", twosComp((uint8_t) plantower_buff[3],(uint8_t) plantower_buff[4]));
//	NRF_LOG_INFO("plantower_10_value: %d", plantower_10_value);

	// calc checksum
	int checksum_calc = 0;
	for(int i = 0; i < PLANTOWER_TWI_BUFF_SIZE - 2; i++) {	// everything except last one (checksum value)
		checksum_calc += plantower_buff[i];
	}
	int checksum_value = 256*plantower_buff[PLANTOWER_TWI_BUFF_SIZE-2] + plantower_buff[PLANTOWER_TWI_BUFF_SIZE-1];

//			checksum_calc = (65536 - checksum_calc) % 256;
//		    NRF_LOG_INFO("checksum_calc: 0x%x", checksum_calc);
//		    NRF_LOG_INFO("checksum_value: 0x%x", checksum_value);
//		    NRF_LOG_INFO("plantower_buff[PLANTOWER_TWI_BUFF_SIZE-2]: 0x%x", plantower_buff[PLANTOWER_TWI_BUFF_SIZE-2]);
//		    NRF_LOG_INFO("plantower_buff[PLANTOWER_TWI_BUFF_SIZE-1]: 0x%x", plantower_buff[PLANTOWER_TWI_BUFF_SIZE-1]);

	// TODO: make this return a proper success code
	if (checksum_calc == checksum_value) {
//        NRF_LOG_INFO("PLANTOWER_TWI checksum SUCCESS!");
		return 0;
	} else {
		NRF_LOG_INFO("** ERROR: HPM checksum ERROR! **");
		return 2;
	}


    return err_code;

}




// Initialize SD
void sd_init() {

	// Need to Turn on SPI0 (maybe it was turned off to save power)
//	NRF_SPI0->ENABLE = 1;
//	nrf_delay_ms(1000);


	/**
	 * @brief  SDC block device definition, MAYBE MOVE TO TOP: where it was originally
	 * */
	NRF_BLOCK_DEV_SDC_DEFINE(
	        m_block_dev_sdc,
	        NRF_BLOCK_DEV_SDC_CONFIG(
	                SDC_SECTOR_SIZE,
	                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
	         ),
	         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
	);

//	NRF_SPI0->ENABLE = 1;
//	nrf_delay_ms(1000);


	//    uint32_t bytes_written;
	//    FRESULT ff_result;
	    DSTATUS disk_state = STA_NOINIT;

	    // Initialize FATFS disk I/O interface by providing the block device.
	    static diskio_blkdev_t drives[] =
	    {
	            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
	    };

//		NRF_SPI0->ENABLE = 1;
//		nrf_delay_ms(1000);

	    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

//		NRF_SPI0->ENABLE = 1;
//		nrf_delay_ms(1000);

	    NRF_LOG_INFO("Initializing disk 0 (SDC)...");
	    for (uint32_t retries = 3; retries && disk_state; --retries)
	    {
	        disk_state = disk_initialize(0);
	    }
	    if (disk_state)
	    {
	        NRF_LOG_INFO("Disk initialization failed.");
//	        return -1;
	    }

//		NRF_SPI0->ENABLE = 1;

}


// Mounting SD
void sd_mount() {

    NRF_LOG_INFO("Mounting volume...");
    ff_result = f_mount(&fs, "", 1);
//    ff_result = f_mount(&fs, "", 0);
    if (ff_result)
    {
        NRF_LOG_INFO("Mount failed.");
//        return -1;
    }

}

// Open SD
void sd_open() {

    // Open SD
    NRF_LOG_INFO("Writing to file " FILE_NAME "...");
    ff_result = f_open(&file, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    NRF_LOG_INFO("--AFO");
//    nrf_delay_ms(500);
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Unable to open or create file: " FILE_NAME ".");
//        return -1;
    }

}


void sd_write_str(const void* buff) {

	uint32_t bytes_written;


//	NRF_LOG_INFO("buff: %s\r\n", buff);
//	NRF_LOG_INFO("sizeof(*buff) - 1: %d\r\n", sizeof(*buff) - 1);
//	NRF_LOG_INFO("strlen(buff): %d\r\n", strlen(buff));

//	ff_result = f_write(&file, buff, sizeof(*buff) - 1, (UINT *) &bytes_written);
	ff_result = f_write(&file, buff, strlen(buff), (UINT *) &bytes_written);
	if (ff_result != FR_OK)
	{
		NRF_LOG_INFO("Write failed\r\n.");
	}
	else
	{
		NRF_LOG_INFO("%d bytes written.", bytes_written);
	}

}






/**
 * Save all of the data
 */
void save_data(void) {

    // SD card TODO: add error code checking and err_cnt++
	NRF_LOG_INFO("");
    NRF_LOG_INFO("Testing SD Card...");
	NRF_LOG_INFO("------------------");
    sd_init();		// TODO: check that this doesn't need to be init with the other init's
    sd_mount();
    NRF_LOG_INFO("--AM");
//    nrf_delay_ms(500);
    sd_open();
    NRF_LOG_INFO("--AO");
//    nrf_delay_ms(500);
    // Write to SD
    if (!header_is_written) {
    	sd_write_str(FILE_HEADER);
    	header_is_written = 1;
    }

    char out_str[MAX_OUT_STR_SIZE];
	NRF_LOG_INFO("sharpPM_value*adc_to_V/MBED_VREF: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(sharpPM_value*adc_to_V/MBED_VREF));
//	sprintf(out_str, "%f", sharpPM_value*adc_to_V/MBED_VREF);
//	NRF_LOG_INFO(out_str);
	NRF_LOG_FLUSH();
//	NRF_LOG_INFO("s: %s", nrf_log_push(out_str));
//	NRF_LOG_INFO("s: %s", (uint32_t)out_str);
	NRF_LOG_FLUSH();
//    int out_str_size = sprintf(out_str, "%d,%d,%d,%.4f,%d,%d,%.4f,%.4f,%d\r\n",timeNow,hpm_2_5_value,hpm_10_value,sharpPM_value*adc_to_V/MBED_VREF,dht_temp_C,dht_humidity,specCO_value*adc_to_V/MBED_VREF,figCO_value*adc_to_V/MBED_VREF,figCO2_value);
//    int out_str_size = sprintf(out_str, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lu,%lu,%lu\r\n",timeNow,hpm_2_5_value,hpm_10_value,(int) (sharpPM_value*adc_to_V/MBED_VREF*1000),dht_temp_C,dht_humidity,(int) (specCO_value*adc_to_V/MBED_VREF*1000),(int) (figCO_value*adc_to_V/MBED_VREF*1000),figCO2_value, plantower_2_5_value, plantower_10_value, (int) (battery_value*adc_to_V*1000), err_cnt, dht_error_cnt_total, hpm_error_cnt_total);
    int out_str_size = sprintf(out_str, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%lu,%d,%lu,%lu,%lu\r\n",timeNow,hpm_2_5_value,hpm_10_value,(int) (sharpPM_value*adc_to_V/MBED_VREF*1000),dht_temp_C,dht_humidity,(int) (specCO_value*adc_to_V/MBED_VREF*1000),(int) (figCO_value*adc_to_V/MBED_VREF*1000),figCO2_value, plantower_2_5_value, plantower_10_value, bme_temp_C, bme_humidity, bme_pressure, (int) (battery_value*adc_to_V*1000), err_cnt, dht_error_cnt_total, hpm_error_cnt_total);
    NRF_LOG_INFO("out_str: %s", out_str);
    // Make sure buffer was big enough and didn't spill over
    if (out_str_size > MAX_OUT_STR_SIZE) {
    	NRF_LOG_INFO("** ERROR: out_str too big!, out_str_size=%d", out_str_size);
    	err_cnt++;
    }

    sd_write_str(out_str);



    // Need to Uninit stuff.  O/w will not write on subsequent loops if ADP shuts off SDC; also drains power
    (void) f_close(&file);
    ff_result = f_mount(0, "", 1);
    if (ff_result) {
    	NRF_LOG_INFO("** WARNING: UNmount Failed, ff_result: %d", ff_result);
    }
    DSTATUS disk_state = disk_uninitialize(0);
    if (disk_state != 1) {
    	NRF_LOG_INFO("** WARNING: Disk NOT properly uninitialized, disk_state: %d", disk_state);
    }





}




/**
 * Read all of the sensors
 */
void get_data(component_type components_used[]) {

		/** Initialize some stuff **/
	//	uint32_t err_cnt = 0;
	//	int avoided_error_cnt = 0;
//		err_cnt = 0;
//		avoided_error_cnt = 0;
//		ret_code_t err_code;
		// Sharp PM, Turn LED OFF (high)
	if (using_sensor(SHARP, components_used)) {
	    nrf_gpio_cfg_output(SHARP_PM_LED);
		nrf_gpio_pin_set(SHARP_PM_LED);
	}




	// DHT sensor
	if (using_sensor(DHT, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing DHT...");
		NRF_LOG_INFO("--------------");
		dht_init(DHT_PIN);

		// Wait for sensor to settle from when ADP turned on
		while (!dht_startup_wait_done) {}
	//	NRF_LOG_INFO("dht_startup_wait_done: %d", dht_startup_wait_done);


	//    int dht_error;
		err_code = DHTLIB_ERROR_TIMEOUT;
		for (int i=0; (err_code == DHTLIB_ERROR_TIMEOUT) && (i < DHT_RETRY_NUM); i++) {
	//	    dht_init(DHT_PIN);
			NRF_LOG_INFO("Start DHT read..");
			err_code = dht_read();
		//	NRF_LOG_INFO("dht_error: %d", dht_error);
			if (err_code == DHTLIB_ERROR_TIMEOUT) {
				NRF_LOG_INFO("* RETRY: DHT TIMEOUT ERROR, err_code=%d *", err_code);
				avoided_error_cnt++;
			}
			nrf_delay_ms(500);
		}
		if (err_code) {
			NRF_LOG_INFO("** ERROR: DHT read, err_code=%d **", err_code);
			dht_temp_C = 0;
			dht_humidity = 0;
			err_cnt++;
			dht_error_cnt_total++;
		} else {
			NRF_LOG_INFO("SUCCESS: DHT READ");
			dht_temp_C = dht_getCelsius();
			dht_humidity = dht_getHumidity();
		}
		NRF_LOG_INFO("dht_temp_C: %d", dht_temp_C);
		NRF_LOG_INFO("dht_temp_F: %d", dht_getFahrenheit());
		NRF_LOG_INFO("dht_humidity: %d", dht_humidity);
		dht_uninit();
	}



	// Testing GPIO
	if (using_sensor(GPIO, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing GPIO...");
		NRF_LOG_INFO("---------------");
		nrf_gpio_cfg_output(GPIO_TEST_PIN);

	//    while (1) {
			NRF_LOG_INFO("LOW.");
			nrf_gpio_pin_clear(GPIO_TEST_PIN);
			nrf_delay_ms(GPIO_TEST_DELAY);

			NRF_LOG_INFO("HIGH.");
			nrf_gpio_pin_set(GPIO_TEST_PIN);
			nrf_delay_ms(2*GPIO_TEST_DELAY);
	//    }
	}





	//Figaro CO2, TWI (I2C)
	if (using_sensor(FIGARO_CO2, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing Figaro CO2 with I2C/TWI...");
		NRF_LOG_INFO("----------------------------------");
		NRF_LOG_FLUSH();
	//    twi_init();
		err_code = read_figCO2();

	//	NRF_LOG_INFO("figCO2_value: 0x%x", figCO2_value);
		NRF_LOG_INFO("figCO2_value: %d", figCO2_value);
		if (err_code) {
			NRF_LOG_INFO("** ERROR: Figaro CO2 read, err_code=%d **", err_code);
			err_cnt++;
		}
	}


	//BME280 TRH, TWI (I2C)
	if (using_sensor(BME, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing BME280 TRH with I2C/TWI...");
		NRF_LOG_INFO("----------------------------------");
//		NRF_LOG_FLUSH();
	//    twi_init();

		err_code = bme_init();
		// Wait for the measurements to end
		nrf_delay_ms(BME_MEAS_WAIT);
		err_code = read_BME();

		NRF_LOG_INFO("bme_temp_C: %d", bme_temp_C);
		NRF_LOG_INFO("bme_humidity: %d", bme_humidity);
		NRF_LOG_INFO("bme_pressure: %d", bme_pressure);
		if (err_code) {
			NRF_LOG_INFO("** ERROR: BME280 TRH read, err_code=%d **", err_code);
			err_cnt++;
		}
	}


	// RTC, TWI (I2C)
	if (using_sensor(RTC, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing RTC with I2C/TWI...");
		NRF_LOG_INFO("---------------------------");
	//    twi_init();	// already initialized with Figaro CO2
		err_code = read_rtc();
		NRF_LOG_INFO("timeNow: %d", timeNow);
	}



	// Trying to read sample from ADC
	if (using_sensor(ADC, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing ADC...");
		NRF_LOG_INFO("--------------");

		nrf_drv_saadc_sample_convert(ADC_CHANNEL_NUM, &adc_value);
		NRF_LOG_INFO("Sample 1: %d", adc_value);

		// convert to V
		NRF_LOG_INFO("adc_to_V*1000: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(adc_to_V*1000));
		NRF_LOG_INFO("adc_value (mV): " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(adc_value*adc_to_V*1000));
		NRF_LOG_INFO("adc_value (\%): " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(adc_value*adc_to_V/MBED_VREF));
	//	nrf_drv_saadc_sample();		// Non-blocking function
	//    }
	}



	// Sharp PM, Analog read.  TODO: implement sample LED
	if (using_sensor(SHARP, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing Sharp PM...");
		NRF_LOG_INFO("-------------------");

		// Pre-read wait, prevents garbage reading
		nrf_drv_saadc_sample_convert(SHARP_PM_CHANNEL_NUM, &sharpPM_value);
		nrf_delay_ms(PRE_READ_WAIT);

		// Turn LED on (low)
		NRF_LOG_INFO("Sampling... (between LED stuff)");
		NRF_LOG_INFO("Turning LED ON (low)");
	//    nrf_gpio_cfg_output(SHARP_PM_LED);
		nrf_gpio_pin_clear(SHARP_PM_LED);
		nrf_delay_us(280);

		// Taking sample (do it twice, just for testing)
		nrf_drv_saadc_sample_convert(SHARP_PM_CHANNEL_NUM, &sharpPM_value);

		// Turn LED off (high)
		nrf_delay_us(40);
	//	NRF_LOG_INFO("Turning LED off (high)");
		nrf_gpio_pin_set(SHARP_PM_LED);
		nrf_delay_us(9680);
		NRF_LOG_INFO("LED turned OFF (high)");

		NRF_LOG_INFO("Sample 1: %d", sharpPM_value);
		NRF_LOG_INFO("sharpPM_value (mV): %d", sharpPM_value*adc_to_V*1000);
	}





	// Spec CO, Analog read.  TODO: implement averaging over 128 samples
	if (using_sensor(SPEC_CO, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing Spec CO... (not connected)");
		NRF_LOG_INFO("------------------");

		// Pre-read wait, prevents garbage reading
		nrf_saadc_value_t specCO_temp;
		nrf_drv_saadc_sample_convert(SPEC_CO_CHANNEL_NUM, &specCO_temp);
		nrf_delay_ms(PRE_READ_WAIT);

	//    while(1) {
		NRF_LOG_INFO("Sampling...");
		int specCO_total = 0;
		// read it a bunch of times and then average
		for (int i = 0; i < 128; i++) {
			nrf_drv_saadc_sample_convert(SPEC_CO_CHANNEL_NUM, &specCO_temp);
			specCO_total += specCO_temp;

	//		NRF_LOG_INFO("Sample 1: %d", specCO_value);
			nrf_delay_ms(SPEC_CO_DELAY);
		}

		specCO_value = specCO_total/128.0f;
		NRF_LOG_INFO("specCO_value: %d", specCO_value);
		NRF_LOG_INFO("specCO_value (mV): %d", specCO_value*adc_to_V*1000);
	}



	// Figaro CO, Analog read.
	if (using_sensor(FIGARO_CO, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing Figaro CO...");
		NRF_LOG_INFO("--------------------");

		// Pre-read wait, prevents garbage reading
		nrf_drv_saadc_sample_convert(FIG_CO_CHANNEL_NUM, &figCO_value);
		nrf_delay_ms(PRE_READ_WAIT);

		NRF_LOG_INFO("Sampling...");
		nrf_drv_saadc_sample_convert(FIG_CO_CHANNEL_NUM, &figCO_value);
		NRF_LOG_INFO("figCO_value (mV): %d", figCO_value*adc_to_V*1000);
	}




	// Check Battery Level
	if (using_sensor(BATTERY, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Checking Battery Level...");
		NRF_LOG_INFO("-------------------------");

		// Pre-read wait, prevents garbage reading
		nrf_drv_saadc_sample_convert(BATTERY_CHANNEL_NUM, &battery_value);
		nrf_delay_ms(PRE_READ_WAIT);

		NRF_LOG_INFO("Sampling...");
		nrf_drv_saadc_sample_convert(BATTERY_CHANNEL_NUM, &battery_value);
		NRF_LOG_INFO("battery_value (mV): %d", battery_value*adc_to_V*1000);
	}




	// TODO: Maybe nrf_drv_saadc_uninit() here to save power: "The SAADC is only enabled before sampling and disabled when sampling completes to avoid high current consumption of the EasyDMA"





	// Small Plantower, TWI (I2C)
	if (using_sensor(SMALL_PLANTOWER, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing Small Plantower with I2C/TWI...");
		NRF_LOG_INFO("---------------------------------------");
		// Wait for sensor to settle from when ADP turned on
		NRF_LOG_INFO("Waiting for startup wait..");
		while (!plantower_startup_wait_done) {}

		err_code = read_plantower_twi();

		NRF_LOG_INFO("plantower_2_5_value = %d", plantower_2_5_value);
		NRF_LOG_INFO("plantower_10_value = %d", plantower_10_value);
	}





	/** Test HPM sensor (Serial comm) **/
	if (using_sensor(HONEYWELL, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing Honeywell/Plantower (Serial Comm)...");
		NRF_LOG_INFO("--------------------------------------------");

		// Wait for sensor to settle from when ADP turned on
		while (!hpm_startup_wait_done) {}

		/** Initialize some stuff **/
		// Serial Read setup, for HPM read. Commented sections are from example, but not sure if they're needed
			if (!nrf_drv_clock_init_check() ) {
				NRF_LOG_INFO("Need to initialize the clock..");
				err_code = nrf_drv_clock_init();	// TODO: Maybe remove, unnecessary?
	//	    	NRF_LOG_INFO("err_code = %d", err_code);
				APP_ERROR_CHECK(err_code);
			}
	//    	err_code = nrf_drv_clock_init();	// TODO: Maybe remove, unnecessary?
	//    	NRF_LOG_INFO("err_code = %d", err_code);
	//        APP_ERROR_CHECK(err_code);
			if (!nrf_drv_power_init_check() ) {
				NRF_LOG_INFO("Need to initialize the power driver..");
				err_code = nrf_drv_power_init(NULL);	// TODO: Maybe remove, unnecessary?
	//	    	NRF_LOG_INFO("err_code = %d", err_code);
				APP_ERROR_CHECK(err_code);
			}
	//        err_code = nrf_drv_power_init(NULL);	// TODO: Maybe remove, unnecessary?
	//    	NRF_LOG_INFO("err_code = %d", err_code);
	//        APP_ERROR_CHECK(err_code);
		nrf_drv_clock_lfclk_request(NULL);
		err_code = app_timer_init();	// needed for serial timeout checking
		NRF_LOG_INFO("err_code = %d", err_code);
		APP_ERROR_CHECK(err_code);
		err_code = nrf_serial_init(&serial_uart, &m_uart0_drv_config, &serial_config);
		NRF_LOG_INFO("err_code = %d", err_code);
		APP_ERROR_CHECK(err_code);

		// Need to send commands if not using autosend
		if (!USING_AUTOSEND) {
		//	err_code = nrf_serial_rx_drain(&serial_uart);

			NRF_LOG_INFO("Sending Command.. Disable Autosend");
			err_code = hpm_cmd_and_ack(hpm_stop_autosend_cmd);	//stop HPM autosend
		//	NRF_LOG_INFO("--2");

		//	err_code = nrf_serial_rx_drain(&serial_uart);


			NRF_LOG_INFO("Sending Command.. Disable Autosend");
			err_code = hpm_cmd_and_ack(hpm_stop_autosend_cmd);	//Do again, since HPM may still be autosending, which may confuse cmd_ack
		//	NRF_LOG_INFO("--2");
			NRF_LOG_INFO("Sending Command.. Start Measurement");
			err_code = hpm_cmd_and_ack(hpm_start_meas_cmd);	//start HPM
		//	NRF_LOG_INFO("--1");
		//    err_code = hpm_cmd_and_ack(hpm_stop_autosend_cmd);	//stop HPM autosend
			//	NRF_LOG_INFO("--2");
		}


	//    NRF_LOG_INFO("HPM_TEST_DELAY: %d", HPM_TEST_DELAY);
	//    nrf_delay_ms(HPM_TEST_DELAY);	// HPM has long response time

		// Try to read to read it a few times
		err_code = NRF_ERROR_TIMEOUT;
		for (int i=0; (err_code == NRF_ERROR_TIMEOUT) && (i < HPM_NUM_RETRIES); i++) {
			// Try reading, and retry if timeout error.  HPM sends every 1000 ms, so make sure we try enough times
			NRF_LOG_INFO("HPM Read, Try #%d", i);
			err_code = NRF_ERROR_TIMEOUT;
			for (int j=0; (err_code == NRF_ERROR_TIMEOUT) && (j < (1000/HPM_SERIAL_TIMEOUT + 1)); j++) {
				err_code = hpm_read_meas();		//read the measurement
				if (err_code == NRF_ERROR_TIMEOUT) {
					NRF_LOG_INFO("* RETRY: HPM TIMEOUT ERROR, err_code=%d *", err_code);
					avoided_error_cnt++;
				} else {
					NRF_LOG_INFO("SUCCESS: HPM READ");
				}
			}

			// Try the whole reading process again if it didn't work
			if ((HPM_NUM_RETRIES > 1) && (err_code != NRF_SUCCESS)) {
				nrf_delay_ms(HPM_RETRY_WAIT);
				NRF_LOG_INFO("* RETRY: HPM wasn't read, err_code=%d *", err_code);
			}
		}

		if (err_code) {
			NRF_LOG_INFO("** ERROR: HPM read UNKNOWN ERROR, err_code=%d **", err_code);
			err_cnt++;
			hpm_error_cnt_total++;
		}

		// Turn Fan off if not using autosend
		if (!USING_AUTOSEND) {
			err_code = hpm_cmd_and_ack(hpm_stop_meas_cmd);	//stop HPM, turns off fan
		}

		// Uninit the serial since no longer needed
		err_code = nrf_serial_uninit(&serial_uart);
		APP_ERROR_CHECK(err_code);
	}




	/** UNinitialize some stuff **/
	// Serial Read setup, for HPM read. Commented sections are from example, but not sure if they're needed
//	err_code = nrf_serial_uninit(&serial_uart);
//	APP_ERROR_CHECK(err_code);
////		err_code = app_timer_init();	// needed for serial timeout checking
////		APP_ERROR_CHECK(err_code);
//	nrf_drv_clock_lfclk_release();
//	nrf_drv_power_uninit();	// TODO: Maybe remove, unnecessary?
//	nrf_drv_clock_uninit();	// TODO: Maybe remove, unnecessary?










	/** UNinitialize some stuff **/
	// Sharp PM, Turn LED ON (low)
	if (using_sensor(SHARP, components_used)) {
		nrf_gpio_cfg_output(SHARP_PM_LED);
		nrf_gpio_pin_clear(SHARP_PM_LED);
	}

//    // Serial Read setup, for HPM read. Commented sections are from example, but not sure if they're needed
//	err_code = nrf_serial_uninit(&serial_uart);
//	APP_ERROR_CHECK(err_code);
//////		err_code = app_timer_init();	// needed for serial timeout checking
//////		APP_ERROR_CHECK(err_code);
//	nrf_drv_clock_lfclk_release();
//	nrf_drv_power_uninit();	// TODO: Maybe remove, unnecessary?
//	nrf_drv_clock_uninit();	// TODO: Maybe remove, unnecessary?


}


/**
 * @brief Test function.
 */
int test_main(component_type components_used[])
{
//	if (loop_num>0) {
//		NRF_LOG_INFO("--1");
//		return 0;
//	}

//    /* Initialize general stuff	*/
//    bsp_board_leds_init();
//    ret_code_t err_code;
//    int err_cnt = 0;
//    // NRF_LOG setup
//    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
//    NRF_LOG_DEFAULT_BACKENDS_INIT();

	// Start basic stuff
//	uint32_t err_cnt = 0;
//	int avoided_error_cnt = 0;
//	ret_code_t err_code;
	NRF_LOG_INFO("");
	NRF_LOG_INFO("");
	NRF_LOG_INFO("-----------------------");
	NRF_LOG_INFO("| Testing test_main() |");
	NRF_LOG_INFO("-----------------------");
	NRF_LOG_INFO("loop_num = %d", loop_num);
	NRF_LOG_INFO("wdt_triggered = %d", wdt_triggered);


//	// ADC setup
//    saadc_init();
//    // Serial Read setup, for HPM read. Commented sections are from example, but not sure if they're needed
//    //    ret = nrf_drv_clock_init();
//    //    APP_ERROR_CHECK(ret);
//    //    ret = nrf_drv_power_init(NULL);
//    //    APP_ERROR_CHECK(ret);
//    //    nrf_drv_clock_lfclk_request(NULL);
//	err_code = app_timer_init();	// needed for serial timeout checking
//	APP_ERROR_CHECK(err_code);
////	NRF_LOG_INFO("TEST 1");
//	err_code = nrf_serial_init(&serial_uart, &m_uart0_drv_config, &serial_config);
//	APP_ERROR_CHECK(err_code);
////	NRF_LOG_INFO("TEST 2");





    // ADP, turn ON all power
	NRF_LOG_INFO("");
	NRF_LOG_INFO("WAKE: Turning ON ADP Power...");
	NRF_LOG_INFO("-----------------------------");

    nrf_gpio_cfg_output(ADP_PIN);
    nrf_gpio_cfg_output(STATUS_LED);
	NRF_LOG_INFO("HIGH.");
	nrf_gpio_pin_set(ADP_PIN);		// Enable HIGH
//	nrf_gpio_pin_clear(STATUS_LED);	// Enable LOW, Turn ON LED
	nrf_gpio_pin_set(STATUS_LED);	// Enable HIGH, Turn ON LED


	// Start timer for DHT startup wait (settling time is ~1.5s)
//    err_code = app_timer_create(&dht_startup_timer,
//    							APP_TIMER_MODE_SINGLE_SHOT,
//                                dht_startup_handler);	// sets a flag when timer expires
//    NRF_LOG_INFO("err_code: %d", err_code);
//    APP_ERROR_CHECK(err_code);
	dht_startup_wait_done = 0;
    err_code = app_timer_start(dht_startup_timer, APP_TIMER_TICKS(DHT_STARTUP_WAIT_TIME), NULL);
//    err_code = app_timer_start(dht_startup_timer, APP_TIMER_TICKS(DHT_STARTUP_WAIT_TIME), (void *) DHT);
    APP_ERROR_CHECK(err_code);
    plantower_startup_wait_done = 0;
    err_code = app_timer_start(plantower_startup_timer, APP_TIMER_TICKS(PLANTOWER_STARTUP_WAIT_TIME), NULL);
//    err_code = app_timer_start(dht_startup_timer, APP_TIMER_TICKS(PLANTOWER_STARTUP_WAIT_TIME), (void *) PLANTOWER);
    APP_ERROR_CHECK(err_code);
    hpm_startup_wait_done = 0;
    err_code = app_timer_start(hpm_startup_timer, APP_TIMER_TICKS(HPM_STARTUP_WAIT_TIME), NULL);
//    err_code = app_timer_start(dht_startup_timer, APP_TIMER_TICKS(HPM_STARTUP_WAIT_TIME), (void *) HPM);
    APP_ERROR_CHECK(err_code);


	// FOR TESTING: initial delay so things can settle
	nrf_delay_ms(INITIAL_SETTLING_WAIT);




	// All the measurements happen here
	err_cnt = 0;
	avoided_error_cnt = 0;
	for (int i=0; i < NUM_SAMPLES_PER_ON_CYCLE; i++) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("-- SAMPLE #%d/%d --", i+1, NUM_SAMPLES_PER_ON_CYCLE);

		// Read all of the sensors
		get_data(components_used);
		// Save the data to SD card
		save_data();

		// Wait between multiple samples as a buffer time, just in case
		if (NUM_SAMPLES_PER_ON_CYCLE > 1) {
			NRF_LOG_INFO("");
			NRF_LOG_INFO("Wait before next sample: %d ms", WAIT_BETWEEN_SAMPLES);
			nrf_delay_ms(WAIT_BETWEEN_SAMPLES);
		}

	}




//	nrf_delay_ms(3000);

    // ADP sleep, turn OFF all power
	NRF_LOG_INFO("");
	NRF_LOG_INFO("SLEEP: Turning OFF ADP Power...");
	NRF_LOG_INFO("-------------------------------");

    nrf_gpio_cfg_output(ADP_PIN);
    nrf_gpio_cfg_output(STATUS_LED);
	NRF_LOG_INFO("LOW.");
	nrf_gpio_pin_clear(ADP_PIN);	// Enable HIGH, Turn OFF ADP
//	nrf_gpio_pin_set(STATUS_LED);	// Enable LOW, Turn OFF LED
	nrf_gpio_pin_clear(STATUS_LED);	// Enable HIGH, Turn OFF LED

//	nrf_delay_ms(3000);





	// Feed the Watchdog Timer
	NRF_LOG_INFO("");
	NRF_LOG_INFO("Feeding the Watchdog..");
	NRF_LOG_INFO("----------------------");
	NRF_LOG_INFO("WDT_TIMEOUT: %d ms", WDT_TIMEOUT);
    nrf_drv_wdt_channel_feed(wdt_channel_id);



//
//    while (true)
//    {
//        __WFE();
//    }


    // Ending stuff
    avoided_error_cnt_total += avoided_error_cnt;
	NRF_LOG_INFO("avoided_error_cnt = %d", avoided_error_cnt);
	NRF_LOG_INFO("avoided_error_cnt_total = %d", avoided_error_cnt_total);
	if (avoided_error_cnt > 0) {
		NRF_LOG_INFO("** ERRORS AVOIDED! **");

	}
	NRF_LOG_FLUSH();


    loop_num++;
    return err_cnt;


}


/**
 * For getting the Temp internally through the nRF
 */
int32_t get_temp_nrf(void) {

    // This function contains workaround for PAN_028 rev2.0A anomalies 28, 29,30 and 31.
    int32_t volatile temp;

    nrf_temp_init();

//    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
//    NRF_LOG_DEFAULT_BACKENDS_INIT();

//    while (true)
//    {

	NRF_TEMP->TASKS_START = 1; /** Start the temperature measurement. */

	/* Busy wait while temperature measurement is not finished, you can skip waiting if you enable interrupt for DATARDY event and read the result in the interrupt. */
	/*lint -e{845} // A zero has been given as right argument to operator '|'" */
	while (NRF_TEMP->EVENTS_DATARDY == 0)
	{
		// Do nothing.
	}
	NRF_TEMP->EVENTS_DATARDY = 0;

	/**@note Workaround for PAN_028 rev2.0A anomaly 29 - TEMP: Stop task clears the TEMP register. */
	temp = (nrf_temp_read() / 4);

	/**@note Workaround for PAN_028 rev2.0A anomaly 30 - TEMP: Temp module analog front end does not power down when DATARDY event occurs. */
	NRF_TEMP->TASKS_STOP = 1; /** Stop the temperature measurement. */

	return temp;
//	NRF_LOG_INFO("Actual temperature: %d", (int)temp);
//	nrf_delay_ms(500);

//	NRF_LOG_FLUSH();

//    }



}


/**
 * @brief Runs everything.  Basically the main program
 */
int test_all(component_type components_used[])
{

    /** Initialize general stuff	**/
//    int err_cnt = 0;
//    int err_cnt_total = 0;
//	ret_code_t err_code;
    bsp_board_leds_init();
    // NRF_LOG setup
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    // TODO: Check USING_PLANTOWER && USING_HONEYWELL
	// ADC setup
    saadc_init();
//    // Serial Read setup, for HPM read. Commented sections are from example, but not sure if they're needed
//    	err_code = nrf_drv_clock_init();
//        APP_ERROR_CHECK(err_code);
//        err_code = nrf_drv_power_init(NULL);
//        APP_ERROR_CHECK(err_code);
//    nrf_drv_clock_lfclk_request(NULL);
//	err_code = app_timer_init();	// needed for serial timeout checking
//	APP_ERROR_CHECK(err_code);
//	err_code = nrf_serial_init(&serial_uart, &m_uart0_drv_config, &serial_config);
//	APP_ERROR_CHECK(err_code);

//	// Timer init
//    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
//    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
//    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
//    APP_ERROR_CHECK(err_code);

    // TWI (I2C) init
    twi_init();

    // Startup Message
	NRF_LOG_INFO("");
	NRF_LOG_INFO("-----------------");
	NRF_LOG_INFO("| Initial Setup |");
	NRF_LOG_INFO("-----------------");


    // Setup Watchdog Timer
	NRF_LOG_INFO("Start Watchdog Timer... WDT_TIMEOUT: %d", WDT_TIMEOUT);
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    // Default values: Pause in SLEEP, Run in HALT; 2000 ms; IRQ 7
    nrf_drv_wdt_config_t wdt_config = NRF_DRV_WDT_DEAFULT_CONFIG;
    wdt_config.reload_value = WDT_TIMEOUT;
    err_code = nrf_drv_wdt_init(&wdt_config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&wdt_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();



    // ADP, turn OFF all power: start with power off when entering loop.
	NRF_LOG_INFO("Starting with ADP Power OFF... LOW");
    nrf_gpio_cfg_output(ADP_PIN);
	nrf_gpio_pin_clear(ADP_PIN);	// Enable HIGH

	nrf_delay_ms(1000);



	// Prepare timer for startup wait (depends on each sensor)
    err_code = app_timer_create(&dht_startup_timer,
    							APP_TIMER_MODE_SINGLE_SHOT,
                                dht_startup_handler);	// sets a flag when timer expires
    err_code = app_timer_create(&plantower_startup_timer,
    							APP_TIMER_MODE_SINGLE_SHOT,
                                plantower_startup_handler);	// sets a flag when timer expires
    err_code = app_timer_create(&hpm_startup_timer,
    							APP_TIMER_MODE_SINGLE_SHOT,
                                hpm_startup_handler);	// sets a flag when timer expires
//    NRF_LOG_INFO("err_code: %d", err_code);
    APP_ERROR_CHECK(err_code);



//    int num_test_main = 3;	// Each loop takes ~ 12s
//	for (int i=0; i < num_test_main; i++) {
	while (1) {
		err_cnt = test_main(components_used);
		err_cnt_total += err_cnt;

		NRF_LOG_FLUSH();
		NRF_LOG_INFO("");
		NRF_LOG_INFO("-- SUMMARY --");
		NRF_LOG_INFO("timeNow = %d", timeNow);
		NRF_LOG_INFO("err_cnt = %d", err_cnt);
		NRF_LOG_INFO("err_cnt_total = %d", err_cnt_total);
		NRF_LOG_INFO("dht_error_cnt_total = %d", dht_error_cnt_total);
		NRF_LOG_INFO("hpm_error_cnt_total = %d", hpm_error_cnt_total);
		NRF_LOG_INFO("*** test_main() COMPLETE!, next loop_num: %d ***", loop_num);
		NRF_LOG_FLUSH();

		nrf_delay_ms(LOG_INTERVAL);
	}

	NRF_LOG_INFO("");
	NRF_LOG_INFO("***** LOOPING COMPLETE! *****");

	return NRF_SUCCESS;

}


/**
 * Test only the components needed for the SUM
 */
int test_SUM(int entering_deep_sleep) {

//    bsp_board_leds_init();
//    // NRF_LOG setup
//    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
//    NRF_LOG_DEFAULT_BACKENDS_INIT();


    // Startup Message
	NRF_LOG_INFO("");
	NRF_LOG_INFO("-----------------------");
	NRF_LOG_INFO("| TEST SUM COMPONENTS |");
	NRF_LOG_INFO("-----------------------");




    // ADP, turn ON all power
	NRF_LOG_INFO("");
	NRF_LOG_INFO("WAKE: Turning ON ADP Power...");
	NRF_LOG_INFO("-----------------------------");

    nrf_gpio_cfg_output(ADP_PIN);
    nrf_gpio_cfg_output(STATUS_LED);
    nrf_gpio_cfg_output(SAMPLE_LED);
	NRF_LOG_INFO("HIGH.");

	// FOR TESTING, CHECKING IF TURNING OFF, THEN ON WILL CHANGE THINGS
//	nrf_gpio_pin_clear(ADP_PIN);		// Enable HIGH
//	nrf_delay_ms(5000);


	nrf_gpio_pin_set(ADP_PIN);		// Enable HIGH
//	nrf_gpio_pin_clear(STATUS_LED);	// Enable LOW, Turn ON LED
	nrf_gpio_pin_set(STATUS_LED);	// Enable HIGH, Turn ON LED
	nrf_gpio_pin_set(SAMPLE_LED);	// Enable HIGH, Turn ON LED



//	nrf_delay_ms(5000);



	// Test the internal temp reading
	NRF_LOG_INFO("");
	NRF_LOG_INFO("Internal Temp Reading");
	NRF_LOG_INFO("---------------------");
    int32_t temp_nrf = 0;
//	while (1) {
		temp_nrf = get_temp_nrf();
		NRF_LOG_INFO("temp_nrf = %d", temp_nrf);
		nrf_delay_ms(500);
//	}


	// TWI (I2C) init
	twi_init();


	// RTC, TWI (I2C)
	NRF_LOG_INFO("");
	NRF_LOG_INFO("Testing RTC with I2C/TWI...");
	NRF_LOG_INFO("---------------------------");
//    twi_init();	// already initialized with Figaro CO2
//	nrf_delay_ms(500);
	err_code = read_rtc();
	NRF_LOG_INFO("timeNow: %d", timeNow);

	// TWI (I2C) UNinit
//	twi_init();
    nrf_drv_twi_disable(&m_twi);
    nrf_drv_twi_uninit(&m_twi);






	// Save the data to SD card
	nrf_delay_ms(1000);
//	save_data();

	// Disable SPI to save power, without these: 816uA (maybe 0.63mA) in deep sleep
//	NRF_SPI0->ENABLE = 0;	// Still has same current in deep sleep, but NEED this, o/w deep sleep current goes to 0.78mA
//	// Set pins LOW to save power, brings deep sleep current down to 120uA
//    nrf_gpio_cfg_output(SDC_CS_PIN);
//    nrf_gpio_cfg_output(SDC_MOSI_PIN);
//    nrf_gpio_cfg_output(SDC_MISO_PIN);
//    nrf_gpio_cfg_output(SDC_SCK_PIN);
//	nrf_gpio_pin_clear(SDC_CS_PIN);
//	nrf_gpio_pin_clear(SDC_MOSI_PIN);
//	nrf_gpio_pin_clear(SDC_MISO_PIN);
//	nrf_gpio_pin_clear(SDC_SCK_PIN);
//    // Setting pins HIGH makes the deep sleep current: 560uA (0.29mA)
//	nrf_gpio_pin_set(SDC_CS_PIN);
////	nrf_gpio_pin_set(SDC_MOSI_PIN);
////	nrf_gpio_pin_set(SDC_MISO_PIN);
////	nrf_gpio_pin_set(SDC_SCK_PIN);







//	nrf_delay_ms(5000);

    // ADP sleep, turn OFF all power
	NRF_LOG_INFO("");
	NRF_LOG_INFO("SLEEP: Turning OFF ADP Power...");
	NRF_LOG_INFO("-------------------------------");

    nrf_gpio_cfg_output(ADP_PIN);
    nrf_gpio_cfg_output(STATUS_LED);
    nrf_gpio_cfg_output(SAMPLE_LED);
	NRF_LOG_INFO("LOW.");


	// FOR TESTING, TURNING ON/OFF
	nrf_gpio_pin_clear(ADP_PIN);	// Enable HIGH, Turn OFF ADP



//	nrf_delay_ms(5000);


//	nrf_gpio_pin_set(STATUS_LED);	// Enable LOW, Turn OFF LED
	nrf_gpio_pin_clear(STATUS_LED);	// Enable HIGH, Turn OFF LED

//	nrf_delay_ms(5000);

//	nrf_gpio_pin_clear(SAMPLE_LED);	// Enable HIGH, Turn OFF LED
	nrf_gpio_pin_clear(SAMPLE_LED);	// Enable HIGH, Turn OFF LED

//	nrf_delay_ms(5000);




	if (entering_deep_sleep) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("DEEP SLEEP: Turning OFF CPU...");
		NRF_LOG_INFO("------------------------------");

		while (1) {
			// Wait for event.
			__WFE();
			NRF_LOG_INFO("Woke Temporarily");

			// Clear Event Register.
			__SEV();
	//		NRF_LOG_INFO("Woke Temporarily");
			__WFE();
	//		NRF_LOG_INFO("Woke Temporarily");
		}
	}


	return err_code;


}


/**
 * Main program.  Comment whatever major things you want to run.
 */
int main(void) {

	component_type components_used[] = {
////			DHT,		// DIG
////			GPIO,		// DIG
////			ADC,		// AIN
//			FIGARO_CO2,	// I2C,			579
//			RTC,		// I2C
			BME,		// I2C
//			SMALL_PLANTOWER,	// I2C,	2
//			SHARP,		// AIN + DIG,	70
//			SPEC_CO,	// AIN,			102
//			FIGARO_CO,	// AIN,			1643
//			BATTERY,	// AIN(no pin),	3279
////			HONEYWELL,			// SERIAL
	};
	components_used_size = sizeof(components_used) / sizeof(components_used[0]);

	err_code = test_all(components_used);



//    bsp_board_leds_init();
//    // NRF_LOG setup
//    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
//    NRF_LOG_DEFAULT_BACKENDS_INIT();
//
//	int test_num = 2;
//	for (int i=0; i<test_num; i++) {
////		NRF_SPI0->ENABLE = 1;
//
//		int entering_deep_sleep = (i == (test_num-1));	// wait forever only on the last iteration
//		err_code = test_SUM(entering_deep_sleep);
//		nrf_delay_ms(1000);
//	}

}




/** @} */
