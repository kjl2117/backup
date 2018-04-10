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
#include <string.h>
#include "nrf_drv_saadc.h"
#include <math.h>
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_drv_twi.h"
#include "Dht22.h"
#include "nrf_gpio.h"
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

// Taken from ble_app_uart
//#include <stdint.h>
//#include <string.h>
#include "nordic_common.h"
//#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
//#include "app_timer.h"
#include "ble_nus.h"
//#include "app_uart.h"
//#include "app_util_platform.h"
#include "bsp_btn_ble.h"
//#if defined (UART_PRESENT)
//#include "nrf_uart.h"
//#endif
//#if defined (UARTE_PRESENT)
//#include "nrf_uarte.h"
//#endif
//#include "nrf_log.h"
//#include "nrf_log_ctrl.h"
//#include "nrf_log_default_backends.h"



/** GLOBAL VARIABLES **/
//--------------------//

/** Overall **/
#define SETTING_TIME_MANUALLY		0		// set to 1, then set to 0 and flash; o/w will rewrite same time when reset
#define SD_FAIL_SHUTDOWN			0	// If true, will enter infinite loop when SD fails (and wdt will reset)
#define LOG_INTERVAL				4*1000		// ms between sleep/wake
#define NUM_SAMPLES_PER_ON_CYCLE	1	// 20
#define WAIT_BETWEEN_SAMPLES		0	// ms, waitin only if there are multiple samples
#define INITIAL_SETTLING_WAIT		1000	// <500 causes issues?
#define INITIAL_FUEL_GAUGE_WAIT		1000	// <500 causes issues?
#define DHT_STARTUP_WAIT_TIME			0.1*1000	//ms
#define PLANTOWER_STARTUP_WAIT_TIME		6*1000	//ms
#define HPM_STARTUP_WAIT_TIME			6*1000	//ms,	6s (10-15s recommended) for Honeywell; 10s for Plantower

// Counter variables
static uint32_t loop_num = 0;
 static uint32_t avoided_error_cnt = 0;
 static uint32_t avoided_error_cnt_total = 0;
 static uint32_t err_cnt = 0;
 static uint32_t err_cnt_total = 0;
 static uint32_t dht_error_cnt_total = 0;
 static uint32_t hpm_error_cnt_total = 0;
 static uint32_t figCO2_error_cnt_total = 0;
 static uint32_t bme_error_cnt_total = 0;
 static uint32_t rtc_error_cnt_total = 0;
 static uint32_t plantower_error_cnt_total = 0;
 static uint32_t fuel_gauge_error_cnt_total = 0;
static ret_code_t err_code;


/** SD Card Variables.  From example: peripheral/fatfs **/
#define FILE_NAME   "test.TXT"
#define MAX_OUT_STR_SIZE	200
#define FILE_HEADER	"Time,PM2_5,PM10,sharpPM,dhtTemp,dhtHum,specCO,figaroCO,figaroCO2,plantower_2_5_value,plantower_10_value, bme_temp_C, bme_humidity, bme_pressure, rtc_temp,temp_nrf,battery_value, fuel_v_cell, fuel_percent, err_cnt, dht_error_cnt_total, hpm_error_cnt_total\r\n"
#define SD_SYSTEM_RESET_WAIT	2000	//ms
static FATFS fs;
static FIL file;
FRESULT ff_result;
char* TEST_STRING = "SD card test, v09.\r\n";
static int header_is_written = 0;
static bool sd_write_failed = false;

/** ADC Card Variables.  From example: example_code/saadc_simpler	**/
#define SAMPLES_IN_BUFFER 1		// It will read each time, but only enter saadc_callback() after buffer is full
//static nrf_saadc_value_t m_buffer[SAMPLES_IN_BUFFER];
static nrf_saadc_value_t adc_value;

/** General TWI aka I2C **/
#define TWI_RETRY_NUM	3
#define TWI_RETRY_WAIT	500	//ms
#define TWI_INSTANCE_ID     1
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/** Figaro CO2, TWI aka I2C.  from example: peripheral/twi_sensor **/
static int figCO2_value;
const int figCO2_addr = 0x69; // 8bit I2C address, 0x69 for figaro CO2.  0x68 also available
#define FIG_CO2_XFER_WAIT_TIME	2

/** BME280 Variables, TWI **/
#define BME_MEAS_WAIT	10	// ms, need to wait for it to measure, then read
static int32_t bme_temp_C = 0;	// units: degC*100
static int32_t bme_humidity = 0;	// units: %RH*1000
static uint32_t bme_pressure = 0;	// units: Pa
const int bme_addr = 0x76;	//0x77; // 8bit I2C address, 0x76 or 0x77
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

/** RTC, for measuring Time (and Temp) **/
#define RTC_BUFF_SIZE	7
const int rtc_addr = 0x68; // 8bit I2C address, 0x68 for RTC
static int timeNow = 0;
static int rtc_temp = 0;	// units: degC*100, precision +/- 0.25C
static int time_was_set = 0;

/** NRF Internal temp **/
static int32_t temp_nrf = 0;	// units: degC*100, precision +/- 0.25C


/** DHT Variables **/
static int dht_temp_C = 0;
static int dht_humidity = 0;
#define DHT_RETRY_NUM	5

/** Timer variables **/
#define TIMER_NUM	0	// Which Timer to use: TIMER0 reserved for SoftDevice, maybe change sdk_config.h
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

/** Fuel Gauge Check Variables (TWI) **/
const int fuel_addr = 0x36; // 8bit I2C address, 0x68 for RTC
static uint16_t fuel_v_cell = 0;	// units: mV
static uint32_t fuel_percent = 0;	// units: percent*1000
#define MAX17043_to_mV	1.25
static bool has_quick_started_fuel_gauge = false;

/** UV Sensor SI1145 (TWI) **/
const int UV_SI1145_addr = 0x60; // 8bit I2C address, 0x68 for RTC
#define SI1145_REG_UCOEFF0  			0x13
#define SI1145_PARAM_CHLIST				0x01
#define SI1145_PARAM_CHLIST_ENUV 		0x80	// Enable UV sensor
#define SI1145_PARAM_CHLIST_ENAUX 		0x40	//
#define SI1145_PARAM_CHLIST_ENALSIR 	0x20
#define SI1145_PARAM_CHLIST_ENALSVIS 	0x10
#define SI1145_REG_MEASRATE0 			0x08
#define SI1145_REG_COMMAND  			0x18
#define SI1145_ALS_AUTO   				0x0E
static uint16_t UV_SI1145_VIS_value = 0;	// units:
static uint16_t UV_SI1145_IR_value = 0;		// units:
static uint16_t UV_SI1145_UV_value = 0;		// units: 100*UV Index

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
static int hpm_2_5_value = 0;
static int hpm_10_value = 0;


/** WATCHDOG TIMER (WDT) **/
nrf_drv_wdt_channel_id wdt_channel_id;
#define WDT_TIMEOUT		60*1000 + NUM_SAMPLES_PER_ON_CYCLE*(WAIT_BETWEEN_SAMPLES+1000) + (DHT_STARTUP_WAIT_TIME + PLANTOWER_STARTUP_WAIT_TIME + HPM_STARTUP_WAIT_TIME)	// ms, make it more than DHT and HPM delays
static int wdt_triggered = 0;

/** APP TIMERS **/
static int dht_startup_wait_done = 0;
APP_TIMER_DEF(dht_startup_timer);
//#define DHT_STARTUP_WAIT_TIME			0.1*1000	//ms
static int plantower_startup_wait_done = 0;
APP_TIMER_DEF(plantower_startup_timer);
//#define PLANTOWER_STARTUP_WAIT_TIME		6*1000	//ms
static int hpm_startup_wait_done = 0;
APP_TIMER_DEF(hpm_startup_timer);
//#define HPM_STARTUP_WAIT_TIME			6*1000	//ms,	6s (10-15s recommended) for Honeywell; 10s for Plantower

// Pins for the custom Sensen boards
#define TWI_SCL_PIN			27		///< 	P0.27 SCL pin.
#define TWI_SDA_PIN			26		///< 	P0.26 SDA pin.

#define DHT_PIN				16		///< 	P0.16 DHT pin
#define ADC_PIN				NRF_SAADC_INPUT_AIN0		///< 	P0.02 AIN0 pin.
#define GPIO_TEST_PIN		23		///< 	P0.23 random pin for testing

#define SHARP_PM_LED		15
#define SHARP_PM_PIN		NRF_SAADC_INPUT_AIN1	//NRF_SAADC_INPUT_AIN3			///< 	P0.05 AIN3 pin.
#define SPEC_CO_PIN			NRF_SAADC_INPUT_AIN3	//NRF_SAADC_INPUT_AIN1		///< 	P0.03 AIN1 pin.
#define FIG_CO_PIN			NRF_SAADC_INPUT_AIN7		///< 	P0.31 AIN7 pin.
#define BATTERY_PIN			NRF_SAADC_INPUT_VDD			///< 	NO PIN, VDD internally

#define HPM_RX_PIN_NUMBER	8		//8
#define HPM_TX_PIN_NUMBER	6		//6
#define SDC_SCK_PIN			25		///< 	SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN		23		///< 	SDC serial data in (DI) pin.
#define SDC_MISO_PIN		24		///< 	SDC serial data out (DO) pin.
#define SDC_CS_PIN			22		///< 	SDC chip select (CS) pin.
#define ADP1_PIN			20		///< 	20/19, ADP1/ADP2 pin for cutting power, sleep/wake;	BME, SD, RTC, SPEC_CO
#define ADP2_PIN			19		///< 	20/19, ADP1/ADP2 pin for cutting power, sleep/wake;	PLANTOWER, FIG_CO2
#define STATUS_LED			18		///< 	Red
#define SAMPLE_LED			17		///< 	Yellow



// The different sensors (used for startup handler)
typedef enum {
	DHT,		// DIG
	GPIO,		// DIG
	ADP,		// DIG
	TEMP_NRF,	// REG
	FIGARO_CO2,	// I2C
	RTC,		// I2C
	BME,		// I2C
	UV_SI1145,	// I2C
	ADC,		// AIN
	SHARP,		// AIN
	SPEC_CO,	// AIN
	FIGARO_CO,	// AIN
	BATTERY,	// AIN (no pin needed)
	FUEL_GAUGE,	// I2C
	SMALL_PLANTOWER,	// I2C
	HONEYWELL,			// SERIAL
	DEEP_SLEEP,
	SDC,		// SD card, SPIO
} component_type;

component_type twi_sensors[] = {
		FIGARO_CO2,
		RTC,
		BME,
		FUEL_GAUGE,
		SMALL_PLANTOWER,
};

//static component_type *components_used;
static int components_used_size;


/**
 * Function for checking if array contains a value (used for selecting sensor code)
 */
bool using_component(component_type val, component_type *arr) {
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
	wdt_triggered = 1;
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

    // Sending the cmd
    err_code = nrf_serial_write(&serial_uart, cmd, HPM_CMD_LEN, &serial_bytes_written, HPM_SERIAL_TIMEOUT);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("serial_bytes_written: %d", serial_bytes_written);


//    // Check if the HPM acknowledged
    char hpm_ack_rx[2] = {0x0, 0x0 };
	size_t serial_bytes_read;
    err_code = nrf_serial_read(&serial_uart, hpm_ack_rx, sizeof(hpm_ack_rx), &serial_bytes_read, HPM_SERIAL_TIMEOUT);
    NRF_LOG_INFO("hpm_ack_rx: 0x%x, 0x%x", hpm_ack_rx[0], hpm_ack_rx[1]);
    NRF_LOG_INFO("err_code: %d", err_code);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("hpm_ack_rx: 0x%x, 0x%x", hpm_ack_rx[0], hpm_ack_rx[1]);

    // TODO: make this check if 0xA5A5 was received, also check bytes read/written match
    return err_code;
}

int hpm_read_meas() {

    size_t serial_bytes_written;
    size_t serial_bytes_read;
	hpm_2_5_value = 0;
	hpm_10_value = 0;

    // Don't use autosend; should be more robust, but HPM is weird when power cycled
    if (!USING_AUTOSEND) {
		//TODO: use a static char[] and memset to clear each time (be careful memset does same size)
		char hpm_buff[HPM_BUFF_SIZE];		// this initializes everything as 0 (first one is 0, then fills remainder with 0)

		// Clear the RX from previous
		err_code = nrf_serial_rx_drain(&serial_uart);
		NRF_LOG_INFO("err_code: %d", hpm_read_meas_cmd);
	    APP_ERROR_CHECK(err_code);

		// Sending the read_meas command
		NRF_LOG_INFO("hpm_read_meas_cmd: 0x%x", hpm_read_meas_cmd);
		err_code = nrf_serial_write(&serial_uart, hpm_read_meas_cmd, HPM_CMD_LEN, &serial_bytes_written, HPM_SERIAL_TIMEOUT);
		APP_ERROR_CHECK(err_code);

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

		// Read back all data into a buffer
		for (int i=0; i < NUM_AUTOSEND_READ_TRIES; i++) {
			err_code = nrf_serial_read(&serial_uart, &hpm_buff[0], sizeof(char), &serial_bytes_read, HPM_SERIAL_TIMEOUT);
			// If it timed out, return and handle it there
			if (err_code == NRF_ERROR_TIMEOUT) return err_code;
			APP_ERROR_CHECK(err_code);

			// Check if we found the header (then start reading, or continue and keep looking)
			if (hpm_buff[0] == 0x42) {
				// Read the next one and verify it's the next header
				err_code = nrf_serial_read(&serial_uart, &hpm_buff[1], sizeof(char), &serial_bytes_read, HPM_SERIAL_TIMEOUT);
				// If it timed out, return and handle it there TODO: make this more robust
				if (err_code == NRF_ERROR_TIMEOUT) return err_code;
				APP_ERROR_CHECK(err_code);
				if (hpm_buff[1] == 0x4D) {
					found_header = 1;
					break;	// Move on to reading the data
				}
			} else {
				// Keep looking for header
				NRF_LOG_INFO("HAVEN'T FOUND HEADER YET..");

			}

		}

		// If we found the header, read the rest
		if (found_header) {
			NRF_LOG_INFO("FOUND HEADER..");

			err_code = nrf_serial_read(&serial_uart, &hpm_buff[2], HPM_BUFF_SIZE - 2, &serial_bytes_read, HPM_SERIAL_TIMEOUT);
			// If it timed out, return and handle it there TODO: make this more robust
			NRF_LOG_INFO("err_code: %d", err_code);
			if (err_code == NRF_ERROR_TIMEOUT) return err_code;
			APP_ERROR_CHECK(err_code);

			// calc values
			hpm_2_5_value = 256*hpm_buff[6] + hpm_buff[7];
			hpm_10_value = 256*hpm_buff[8] + hpm_buff[9];
			NRF_LOG_INFO("hpm_2_5_value: %d", hpm_2_5_value);
			NRF_LOG_INFO("hpm_10_value: %d", hpm_10_value);

			// calc checksum
			int checksum_calc = 0;
			for(int i = 0; i < HPM_BUFF_SIZE - 2; i++) {	// everything except last one (checksum value)
				checksum_calc += hpm_buff[i];
			}
			int checksum_value = 256*hpm_buff[HPM_BUFF_SIZE-2] + hpm_buff[HPM_BUFF_SIZE-1];

			// TODO: make this return a proper success code
			if (checksum_calc == checksum_value) {
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
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

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


/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
//    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = TWI_SCL_PIN,
       .sda                = TWI_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

//    nrf_drv_twi_disable(&m_twi);	// for saving power


}

// 2's Complement
int twosComp( uint8_t bit_msb, uint8_t bit_lsb) {
    int16_t myInt=0;
    myInt = (bit_msb << 8) | (bit_lsb & 0xff);
    return myInt;
}


// Read Figaro CO2 with TWI (I2C)
static ret_code_t read_figCO2()
{
    uint8_t readme_msb;
    uint8_t readme_lsb;

    nrf_drv_twi_enable(&m_twi);		// for saving power

    uint8_t reg = 0x03;
	err_code = nrf_drv_twi_tx(&m_twi, figCO2_addr, &reg, 1, true);
	//    APP_ERROR_CHECK(err_code);
    if (err_code) {	// handle error outside
    	return err_code;
    }

    err_code = nrf_drv_twi_rx(&m_twi, figCO2_addr, &readme_lsb, 1);
    APP_ERROR_CHECK(err_code);

    reg = 0x04;
    err_code = nrf_drv_twi_tx(&m_twi, figCO2_addr, &reg, 1, true);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_twi_rx(&m_twi, figCO2_addr, &readme_msb, 1);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_disable(&m_twi);	// for saving power

    figCO2_value = twosComp(readme_msb,readme_lsb);

    return err_code;

}


// Compensate the raw BME280 values using calibration values
static ret_code_t bme_read_calib_data() {

	uint8_t cmd[18];
	uint8_t reg0;

    nrf_drv_twi_enable(&m_twi);		// for saving power

	// Get all the temp calib data
    reg0 = 0x88;
    err_code = nrf_drv_twi_tx(&m_twi, bme_addr, &reg0, 1, true);
//    APP_ERROR_CHECK(err_code);
    if (err_code) {	// handle error outside
    	return err_code;
    }

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

    nrf_drv_twi_disable(&m_twi);	// for saving power

	// Convert the values
    dig_H1 = cmd[0];
    dig_H2 = (cmd[2] << 8) | cmd[1];
    dig_H3 = cmd[3];
    dig_H4 = (cmd[4] << 4) | (cmd[5] & 0x0f);
    dig_H5 = (cmd[6] << 4) | ((cmd[5]>>4) & 0x0f);
    dig_H6 = cmd[7];

    return err_code;
}


// Read BME280 TRH with TWI (I2C)
static ret_code_t bme_init() {

	// Read and store calibration data once for all future reads
	if (!has_read_calib_data) {
		err_code = bme_read_calib_data();
	    if (err_code) {	// handle error outside
	    	return err_code;
	    }
		has_read_calib_data = 1;
	}

	uint8_t cmd[2];

    nrf_drv_twi_enable(&m_twi);		// for saving power

	// Starting the measurements by editing control registers
    cmd[0] = 0xf2; // ctrl_hum
    cmd[1] = 0x01; // Humidity oversampling x1
    err_code = nrf_drv_twi_tx(&m_twi, bme_addr, cmd, 2, true);
//    APP_ERROR_CHECK(err_code);
    if (err_code) {	// handle error outside
    	return err_code;
    }


    cmd[0] = 0xf4; // ctrl_meas
    cmd[1] = 0x25; // Temparature oversampling x1, Pressure oversampling x1, Force mode (reads, then goes to standby)
    err_code = nrf_drv_twi_tx(&m_twi, bme_addr, cmd, 2, true);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_disable(&m_twi);	// for saving power

    return err_code;
}



// Read BME280 TRH with TWI (I2C)
static ret_code_t read_BME() {

	uint8_t bmebuff[BME_BUFF_SIZE];

    nrf_drv_twi_enable(&m_twi);		// for saving power

	// Tell it which register we want to start reading from
    uint8_t reg0 = 0xF7;
    err_code = nrf_drv_twi_tx(&m_twi, bme_addr, &reg0, 1, true);
//    APP_ERROR_CHECK(err_code);
    if (err_code) {	// handle error outside
    	return err_code;
    }
    // Read the value from that register
	err_code = nrf_drv_twi_rx(&m_twi, bme_addr, bmebuff, BME_BUFF_SIZE);
	APP_ERROR_CHECK(err_code);

    nrf_drv_twi_disable(&m_twi);	// for saving power

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

// Converts binary coded decimal to decimal (0x12 => 12)
static inline uint8_t bcd_to_dec(uint8_t hex) {
	uint8_t dec = ((hex & 0xF0) >> 4) * 10 + (hex & 0x0F);
    return dec;
}

// Converts decimal to binary coded decimal (12 => 0x12)
static inline uint8_t dec_to_bcd(uint8_t dec) {
	uint8_t bcd = ((dec / 10) << 4) | (dec % 10);
    return bcd;
}


// Set the time of the RTC with TWI (I2C)
static ret_code_t set_rtc(uint8_t sec, uint8_t min, uint8_t hour, uint8_t wday, uint8_t date, uint8_t mon, uint8_t year) {

	uint8_t rtcbuff[8];

	// setup buffer to overwrite registers
	rtcbuff[0] = 0x00;	// register address for Seconds
	rtcbuff[1] = dec_to_bcd(sec);
	rtcbuff[2] = dec_to_bcd(min);
	rtcbuff[3] = dec_to_bcd(hour);
	rtcbuff[4] = dec_to_bcd(wday);
	rtcbuff[5] = dec_to_bcd(date);
	rtcbuff[6] = dec_to_bcd(mon);
	rtcbuff[7] = dec_to_bcd(year);

    nrf_drv_twi_enable(&m_twi);		// for saving power

	// Starting the measurements by editing control registers
    err_code = nrf_drv_twi_tx(&m_twi, rtc_addr, rtcbuff, 8, true);
    if (err_code) {	// handle error outside
    	return err_code;
    }

    nrf_drv_twi_disable(&m_twi);	// for saving power

    return err_code;
}


// Read RTC crystal with TWI (I2C) TODO: pass TWI address as argument
static ret_code_t read_rtc()
{
	uint8_t readreg;
	uint8_t rtcbuff[RTC_BUFF_SIZE];
	uint8_t rtc_temp_buff[2];

    nrf_drv_twi_enable(&m_twi);		// for saving power

    // Checking the first register, make sure it's running (CH bit == 0)
    uint8_t regt = 0x00;
    err_code = nrf_drv_twi_tx(&m_twi, rtc_addr, &regt, 1, true);
    if (err_code) {	// handle error outside
    	return err_code;
    }
//    APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(&m_twi, rtc_addr, &readreg, 1);
    APP_ERROR_CHECK(err_code);
    uint8_t is_running = !(readreg>>7);		// TODO: add a check and correction action here
    NRF_LOG_INFO("is_running: %d", is_running );

    // Check another register that the change was made
    regt = 0x0e;
    err_code = nrf_drv_twi_tx(&m_twi, rtc_addr, &regt, 1, true);
    APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(&m_twi, rtc_addr, &readreg, 1);
    APP_ERROR_CHECK(err_code);

    // Turn off the square wave, it's not needed and wastes power
    regt = 0x0f;
    err_code = nrf_drv_twi_tx(&m_twi, rtc_addr, &regt, 1, true);
    APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(&m_twi, rtc_addr, &readreg, 1);
    APP_ERROR_CHECK(err_code);
    // Turn OFF the specific bits
    readreg &= ~(1UL << 3);	// EN32KHZ
    readreg &= ~(1UL << 6);	// BB32KHZ
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

    // Check that the change was made
    regt = 0x0f;
    err_code = nrf_drv_twi_tx(&m_twi, rtc_addr, &regt, 1, true);
    APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(&m_twi, rtc_addr, &readreg, 1);
    APP_ERROR_CHECK(err_code);

	// Tell it which register we want to start reading from
    uint8_t reg0 = 0x00;
    err_code = nrf_drv_twi_tx(&m_twi, rtc_addr, &reg0, 1, true);
    APP_ERROR_CHECK(err_code);
    // Read the value from that register
	err_code = nrf_drv_twi_rx(&m_twi, rtc_addr, rtcbuff, RTC_BUFF_SIZE);
	APP_ERROR_CHECK(err_code);


	// Get the Temperature too
    regt = 0x11;
    err_code = nrf_drv_twi_tx(&m_twi, rtc_addr, &regt, 1, true);
    APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(&m_twi, rtc_addr, rtc_temp_buff, 2);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_disable(&m_twi);	// for saving power

	// Convert bcd to dec, and print out the values
	for (int i = 0; i < RTC_BUFF_SIZE; i++) {
	    rtcbuff[i] = rtcbuff[i] - 6 * (rtcbuff[i] >> 4);
    }

    // Convert time and store
    struct tm t;
    t.tm_year = (int)rtcbuff[6]+100;
    t.tm_mon = (int)rtcbuff[5]-1;           // Month, 0 - jan
    t.tm_mday = (int)rtcbuff[4];          // Day of the month
    t.tm_hour = (int)rtcbuff[2];
    t.tm_min = (int)rtcbuff[1];
    t.tm_sec = (int)rtcbuff[0];
    t.tm_isdst = 0;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
    timeNow = mktime(&t);

    // Convert the temp and store
    int8_t temp3232a = rtc_temp_buff[0];	// signed int for integer portion of temp
    uint8_t temp3232b = rtc_temp_buff[1];	// 2 MSB are the decimal portion (multiples of 1/4)
    rtc_temp = temp3232a*100;
    //for positive temp
    if((temp3232b == 0x40) && (temp3232a >= 0)) rtc_temp += 25;
    if (temp3232b == 0x80) rtc_temp += 50;
    if((temp3232b == 0xc0) && (temp3232a >= 0)) rtc_temp += 75;
    //for negative temp
    if((temp3232b == 0x40) && (temp3232a < 0)) rtc_temp += 75;
    if((temp3232b == 0xc0) && (temp3232a < 0)) rtc_temp += 25;



    return err_code;

}


// Quick Start Fuel Gauge MAX17043 with TWI (I2C)
// This lets you pick when the "initial guess" of the internal SOC algorithm takes place
// Make sure this happens when there aren't initial turn-on fluctuations.
static ret_code_t fuel_gauge_quick_start() {

	uint8_t regt = 0x06;	// MODE register
	uint8_t cmd[] = {regt, 0x40, 0x00};	// write 0x4000 to quick start

    nrf_drv_twi_enable(&m_twi);		// for saving power

    // Get battery voltage
	NRF_LOG_INFO("--Q1");
    err_code = nrf_drv_twi_tx(&m_twi, fuel_addr, cmd, 3, false);
    if (err_code) {	// handle error outside
    	NRF_LOG_INFO("** WARNING in fuel_gauge_quick_start(), err_code: %d **", err_code);
    	return err_code;
    }
//    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_disable(&m_twi);		// for saving power

    return err_code;

}


// Sleep Fuel Gauge MAX17043 with TWI (I2C), for saving power
static ret_code_t fuel_gauge_sleep() {

	uint8_t regt = 0x0C;	// CONFIG register
	uint8_t cmd[] = { 	regt, 	// reg address to write to
						0x97, 	// default values
						0x80,	// sleep bit = 1, clears alert bit, alert threshold = 32%
	};

    nrf_drv_twi_enable(&m_twi);		// for saving power

    // Get battery voltage
    err_code = nrf_drv_twi_tx(&m_twi, fuel_addr, cmd, 3, false);
    if (err_code) {	// handle error outside
    	NRF_LOG_INFO("** WARNING in fuel_gauge_sleep(), err_code: %d **", err_code);
    	return err_code;
    }
//    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_disable(&m_twi);		// for saving power

    return err_code;

}

// Wake Fuel Gauge MAX17043 with TWI (I2C), for saving power
static ret_code_t fuel_gauge_wake() {

	uint8_t regt = 0x0C;	// CONFIG register
	uint8_t cmd[] = { 	regt, 	// reg address to write to
						0x97, 	// default values
						0x00,	// sleep bit = 0, clears alert bit, alert threshold = 32%
	};

    nrf_drv_twi_enable(&m_twi);		// for saving power

    // Get battery voltage
    err_code = nrf_drv_twi_tx(&m_twi, fuel_addr, cmd, 3, false);
    if (err_code) {	// handle error outside
    	NRF_LOG_INFO("** WARNING in fuel_gauge_wake(), err_code: %d **", err_code);
    	return err_code;
    }
//    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_disable(&m_twi);		// for saving power

    return err_code;

}

// Read Fuel Gauge MAX17043 with TWI (I2C) TODO: add quickstart, sleeping, waking
static ret_code_t read_fuel_gauge() {

	uint8_t cmd[2];
	uint8_t regt;

    nrf_drv_twi_enable(&m_twi);		// for saving power

    // Get battery voltage
    regt = 0x02;
    err_code = nrf_drv_twi_tx(&m_twi, fuel_addr, &regt, 1, true);
    if (err_code) {	// handle error outside
    	NRF_LOG_INFO("** WARNING in read_fuel_gauge(), err_code: %d **", err_code);
    	return err_code;
    }
//    APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(&m_twi, fuel_addr, cmd, 2);
    APP_ERROR_CHECK(err_code);
    // Convert and save
    fuel_v_cell = (cmd[0] << 8) | cmd[1];	// combine 2 bytes
    fuel_v_cell = (fuel_v_cell >> 4) * MAX17043_to_mV;	// last 4 bits are nothing, convert to mV

    // Get battery percentage SOC
    regt = 0x04;
    err_code = nrf_drv_twi_tx(&m_twi, fuel_addr, &regt, 1, true);
    APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(&m_twi, fuel_addr, cmd, 2);
    APP_ERROR_CHECK(err_code);
    // Convert and save
    fuel_percent = (cmd[0] << 8) | cmd[1];	// combine 2 bytes
	NRF_LOG_INFO("cmd[0] = 0x%x", cmd[0]);
	NRF_LOG_INFO("cmd[1] = 0x%x", cmd[1]);
//	NRF_LOG_INFO("fuel_percent = %d", fuel_percent);
    fuel_percent = fuel_percent/256.0 * 1000;	// convert to float, but leave 3 decimal places
////    fuel_percent = fuel_percent * (1000.0/256.0);	// convert to float, but leave 3 decimal places
//	NRF_LOG_INFO("fuel_percent = %d", fuel_percent);
//	NRF_LOG_INFO("cmd[0]*1000 + cmd[1]*(1000.0/256.0) = %d", cmd[0]*1000 + cmd[1]*(1000.0/256.0) );
//	fuel_percent = cmd[0]*1000 + cmd[1]*(1000.0/256.0);	// Need *1000 for 3 decimal places, /256 to get fractional part


    nrf_drv_twi_disable(&m_twi);		// for saving power

    return err_code;

}


// UV Sensor SI1145 Initialization
static ret_code_t UV_SI1145_init() {

	uint8_t regt;

    nrf_drv_twi_enable(&m_twi);		// for saving power

    // Write default coefficients, maybe later read and overwrite with product-calibrated ones
    regt = SI1145_REG_UCOEFF0;
    uint8_t cmd0[] = {regt, 0x29, 0x89, 0x02, 0x00	};
    err_code = nrf_drv_twi_tx(&m_twi, UV_SI1145_addr, cmd0, 5, false);
    if (err_code) {	// handle error outside
    	NRF_LOG_INFO("** WARNING in UV_SI1145_init(), err_code: %d **", err_code);
    	return err_code;
    }

    // Enable measurements
    regt = SI1145_PARAM_CHLIST;
    uint8_t cmd1[] = {	regt, SI1145_PARAM_CHLIST_ENUV |
			  SI1145_PARAM_CHLIST_ENALSIR | SI1145_PARAM_CHLIST_ENALSVIS	};
    err_code = nrf_drv_twi_tx(&m_twi, UV_SI1145_addr, cmd1, 4, false);

    // Setup for auto run
    regt = SI1145_REG_MEASRATE0;	// measurement rate for auto
    uint8_t cmd2[] = {	regt, 0xFF	};	// 255 * 31.25uS = 8ms
    err_code = nrf_drv_twi_tx(&m_twi, UV_SI1145_addr, cmd2, 2, false);
    regt = SI1145_REG_COMMAND;		// setup auto send for sensors
    uint8_t cmd3[] = {	regt, SI1145_ALS_AUTO	};
    err_code = nrf_drv_twi_tx(&m_twi, UV_SI1145_addr, cmd3, 2, false);


//    err_code = nrf_drv_twi_tx(&m_twi, UV_SI1145_addr, &regt, 1, true);
//    if (err_code) {	// handle error outside
//    	NRF_LOG_INFO("** WARNING in read_fuel_gauge(), err_code: %d **", err_code);
//    	return err_code;
//    }
////    APP_ERROR_CHECK(err_code);
//	err_code = nrf_drv_twi_rx(&m_twi, UV_SI1145_addr, cmd, 2);
//    APP_ERROR_CHECK(err_code);
//    // Convert and save
//    fuel_v_cell = (cmd[0] << 8) | cmd[1];	// combine 2 bytes
//    fuel_v_cell = (fuel_v_cell >> 4) * MAX17043_to_mV;	// last 4 bits are nothing, convert to mV



    nrf_drv_twi_disable(&m_twi);		// for saving power

    return err_code;


}



// UV Sensor SI1145 read: UV, VIS, IR sensors
static ret_code_t UV_SI1145_read() {

	uint8_t regt;
    uint8_t cmd[4];

    nrf_drv_twi_enable(&m_twi);		// for saving power


    // Read the IR and VIS data
    regt = 0x22;	// start of VIS, then IR data
    err_code = nrf_drv_twi_tx(&m_twi, UV_SI1145_addr, &regt, 1, true);
    if (err_code) {	// handle error outside
    	NRF_LOG_INFO("** WARNING in UV_SI1145_read(), err_code: %d **", err_code);
    	return err_code;
    }
//    APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(&m_twi, UV_SI1145_addr, cmd, 4);
    APP_ERROR_CHECK(err_code);
    // Convert and save
    UV_SI1145_VIS_value = 	cmd[0] | (cmd[1] << 8);	// combine 2 bytes (MSB is 2nd)
    UV_SI1145_IR_value = 	cmd[2] | (cmd[3] << 8);	// combine 2 bytes (MSB is 2nd)
	NRF_LOG_INFO("cmd[0] = 0x%x", cmd[0]);
	NRF_LOG_INFO("cmd[1] = 0x%x", cmd[1]);
	NRF_LOG_INFO("cmd[2] = 0x%x", cmd[2]);
	NRF_LOG_INFO("cmd[3] = 0x%x", cmd[3]);



    // Read the UV data
    regt = 0x2C;	// start of UV data
    err_code = nrf_drv_twi_tx(&m_twi, UV_SI1145_addr, &regt, 1, true);
    APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(&m_twi, UV_SI1145_addr, cmd, 2);
    APP_ERROR_CHECK(err_code);
    // Convert and save
    UV_SI1145_UV_value = 	cmd[0] | (cmd[1] << 8);	// combine 2 bytes (MSB is 2nd)
	NRF_LOG_INFO("cmd[0] = 0x%x", cmd[0]);
	NRF_LOG_INFO("cmd[1] = 0x%x", cmd[1]);


    nrf_drv_twi_disable(&m_twi);		// for saving power

    return err_code;


}


// Read Small Plantower with TWI (I2C)
static ret_code_t read_plantower_twi()
{
	uint8_t plantower_buff[PLANTOWER_TWI_BUFF_SIZE];

    nrf_drv_twi_enable(&m_twi);		// for saving power

	// Tell it which register we want to start reading from
    uint8_t reg0 = 0x00;
    err_code = nrf_drv_twi_tx(&m_twi, plantower_twi_addr, &reg0, 1, true);
    if (err_code) {	// handle error outside
    	return err_code;
    }
//    APP_ERROR_CHECK(err_code);
    // Read the value from that register
	err_code = nrf_drv_twi_rx(&m_twi, plantower_twi_addr, plantower_buff, PLANTOWER_TWI_BUFF_SIZE);
    if (err_code) {	// handle error outside
    	return err_code;
    }
//	APP_ERROR_CHECK(err_code);

    nrf_drv_twi_disable(&m_twi);	// for saving power

	// calc values
	plantower_2_5_value = 256*plantower_buff[6] + plantower_buff[7];
	plantower_10_value = 256*plantower_buff[8] + plantower_buff[9];

	// calc checksum
	int checksum_calc = 0;
	for(int i = 0; i < PLANTOWER_TWI_BUFF_SIZE - 2; i++) {	// everything except last one (checksum value)
		checksum_calc += plantower_buff[i];
	}
	int checksum_value = 256*plantower_buff[PLANTOWER_TWI_BUFF_SIZE-2] + plantower_buff[PLANTOWER_TWI_BUFF_SIZE-1];

	// TODO: make this return a proper success code
	if (checksum_calc == checksum_value) {
		return 0;
	} else {
		NRF_LOG_INFO("** ERROR: HPM checksum ERROR! **");
		return 2;
	}

    return err_code;
}




/**
 * For getting the Temp internally through the nRF
 */
int32_t get_temp_nrf(void) {

    // This function contains workaround for PAN_028 rev2.0A anomalies 28, 29,30 and 31.
    int32_t volatile temp;

    nrf_temp_init();

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
}




// Initialize SD
void sd_init() {
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

	DSTATUS disk_state = STA_NOINIT;

	// Initialize FATFS disk I/O interface by providing the block device.
	static diskio_blkdev_t drives[] =
	{
			DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
	};

	diskio_blockdev_register(drives, ARRAY_SIZE(drives));

	NRF_LOG_INFO("Initializing disk 0 (SDC)...");
	for (uint32_t retries = 3; retries && disk_state; --retries)
	{
		disk_state = disk_initialize(0);
	}
	if (disk_state)
	{
		NRF_LOG_INFO("Disk initialization failed.");
	}

}


// Mounting SD
void sd_mount() {

    NRF_LOG_INFO("Mounting volume...");
    ff_result = f_mount(&fs, "", 1);
    if (ff_result) {
        NRF_LOG_INFO("Mount failed.");
    }
}

// Open SD
void sd_open() {

    // Open SD
    NRF_LOG_INFO("Writing to file " FILE_NAME "...");
    ff_result = f_open(&file, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    NRF_LOG_INFO("--AFO");
    if (ff_result != FR_OK) {
        NRF_LOG_INFO("Unable to open or create file: " FILE_NAME ".");
    }

}


void sd_write_str(const void* buff) {

	uint32_t bytes_written;

	ff_result = f_write(&file, buff, strlen(buff), (UINT *) &bytes_written);
	if (ff_result != FR_OK)	{
		NRF_LOG_INFO("** ERROR: Write failed, ff_result: %d **.", ff_result);
		// Reset the system if it's not working properly
		sd_write_failed = true;
		err_cnt++;
//		nrf_delay_ms(SD_SYSTEM_RESET_WAIT);
//		NVIC_SystemReset();

	}
	else {
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
    sd_open();
    NRF_LOG_INFO("--AO");
    // Write to SD
    if (!header_is_written) {
    	sd_write_str(FILE_HEADER);
    	header_is_written = 1;
    }

    char out_str[MAX_OUT_STR_SIZE];
	NRF_LOG_INFO("sharpPM_value*adc_to_V/MBED_VREF: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(sharpPM_value*adc_to_V/MBED_VREF));
	NRF_LOG_FLUSH();
    int out_str_size = sprintf(out_str, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%lu,%d,%ld,%d,%d,%lu,%lu,%lu,%lu\r\n",timeNow,hpm_2_5_value,hpm_10_value,(int) (sharpPM_value*adc_to_V/MBED_VREF*1000),dht_temp_C,dht_humidity,(int) (specCO_value*adc_to_V/MBED_VREF*1000),(int) (figCO_value*adc_to_V/MBED_VREF*1000),figCO2_value, plantower_2_5_value, plantower_10_value, bme_temp_C, bme_humidity, bme_pressure, rtc_temp, temp_nrf, (int) (battery_value*adc_to_V*1000), fuel_v_cell, fuel_percent, err_cnt, dht_error_cnt_total, hpm_error_cnt_total);
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
	// Turn on Sample LED
    nrf_gpio_cfg_output(SAMPLE_LED);
	nrf_gpio_pin_set(SAMPLE_LED);	// Enable HIGH, Turn ON LED
//    // TWI (I2C) init
//    twi_init();
	// Sharp PM, Turn LED OFF (high)
	if (using_component(SHARP, components_used)) {
	    nrf_gpio_cfg_output(SHARP_PM_LED);
		nrf_gpio_pin_set(SHARP_PM_LED);
	}

	// DHT sensor
	if (using_component(DHT, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing DHT...");
		NRF_LOG_INFO("--------------");
		dht_init(DHT_PIN);

		// Wait for sensor to settle from when ADP turned on
		while (!dht_startup_wait_done) {}


		err_code = DHTLIB_ERROR_TIMEOUT;
		for (int i=0; (err_code == DHTLIB_ERROR_TIMEOUT) && (i < DHT_RETRY_NUM); i++) {
			NRF_LOG_INFO("Start DHT read..");
			err_code = dht_read();
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
	if (using_component(GPIO, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing GPIO...");
		NRF_LOG_INFO("---------------");
		nrf_gpio_cfg_output(GPIO_TEST_PIN);

		NRF_LOG_INFO("LOW.");
		nrf_gpio_pin_clear(GPIO_TEST_PIN);
		nrf_delay_ms(GPIO_TEST_DELAY);

		NRF_LOG_INFO("HIGH.");
		nrf_gpio_pin_set(GPIO_TEST_PIN);
		nrf_delay_ms(2*GPIO_TEST_DELAY);
	}


	// NRF Internal temp reading
	if (using_component(TEMP_NRF, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Internal Temp Reading");
		NRF_LOG_INFO("---------------------");
//		int32_t temp_nrf = 0;
		temp_nrf = get_temp_nrf();
		NRF_LOG_INFO("temp_nrf = %d", temp_nrf);
	}


	// Figaro CO2, TWI (I2C)
	if (using_component(FIGARO_CO2, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing Figaro CO2 with I2C/TWI...");
		NRF_LOG_INFO("----------------------------------");
		NRF_LOG_FLUSH();

		err_code = 1;
		for (int i=0; (err_code) && (i < TWI_RETRY_NUM); i++) {
			err_code = read_figCO2();
			if (err_code) {
				NRF_LOG_INFO("* RETRY: FIGARO ERROR, err_code=%d *", err_code);
				avoided_error_cnt++;
				nrf_delay_ms(TWI_RETRY_WAIT);
			}
		}

		if (err_code) {
			NRF_LOG_INFO("** ERROR: Figaro CO2 read, err_code=%d **", err_code);
			figCO2_value = 0;
			err_cnt++;
			figCO2_error_cnt_total++;
		}
		NRF_LOG_INFO("figCO2_value: %d", figCO2_value);
	}


	// BME280 TRH, TWI (I2C)
	if (using_component(BME, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing BME280 TRH with I2C/TWI...");
		NRF_LOG_INFO("----------------------------------");

		err_code = 1;
		for (int i=0; (err_code) && (i < TWI_RETRY_NUM); i++) {
			err_code = bme_init();
			nrf_delay_ms(BME_MEAS_WAIT);	// Wait for the measurements to end. TODO: maybe disable TWI before and after wait
			err_code = read_BME();
			if (err_code) {
				NRF_LOG_INFO("* RETRY: BME ERROR, err_code=%d *", err_code);
				avoided_error_cnt++;
				nrf_delay_ms(TWI_RETRY_WAIT);
			}
		}

		if (err_code) {
			NRF_LOG_INFO("** ERROR: BME280 TRH read, err_code=%d **", err_code);
			bme_temp_C = 0;
			bme_humidity = 0;
			bme_pressure = 0;
			err_cnt++;
			bme_error_cnt_total++;
		}
		NRF_LOG_INFO("bme_temp_C: %d", bme_temp_C);
		NRF_LOG_INFO("bme_humidity: %d", bme_humidity);
		NRF_LOG_INFO("bme_pressure: %d", bme_pressure);
	}


	// RTC, TWI (I2C)
	if (using_component(RTC, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing RTC with I2C/TWI...");
		NRF_LOG_INFO("---------------------------");

		if (SETTING_TIME_MANUALLY && !time_was_set) {
			NRF_LOG_INFO("** WARNING: SETTING TIME MANUALLY **");
//			set_rtc(00, 44, 21, 3, 6, 3, 18);	// 2018-03-06 Tues, 9:44:00 pm, NOTE: GMT!!!
			set_rtc(00, 59, 15, 3, 10, 4, 18);	// about 11 seconds of delay
			time_was_set = 1;
			// NOTE: turn OFF SETTING_TIME_MANUALLY after
		}
		err_code = 1;
		for (int i=0; (err_code) && (i < TWI_RETRY_NUM); i++) {
			err_code = read_rtc();
			if (err_code) {
				NRF_LOG_INFO("* RETRY: RTC ERROR, err_code=%d *", err_code);
				avoided_error_cnt++;
				nrf_delay_ms(TWI_RETRY_WAIT);
			}
		}

		if (err_code) {
			NRF_LOG_INFO("** ERROR: RTC read, err_code=%d **", err_code);
			timeNow = 0;
			err_cnt++;
			rtc_error_cnt_total++;
		}

		// Print the reading
		NRF_LOG_INFO("timeNow: %d", timeNow);
		NRF_LOG_INFO("rtc_temp: %d", rtc_temp);
	}



	// Trying to read sample from ADC
	if (using_component(ADC, components_used)) {
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
	}



	// Sharp PM, Analog read.  TODO: implement sample LED
	if (using_component(SHARP, components_used)) {
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
		nrf_gpio_pin_set(SHARP_PM_LED);
		nrf_delay_us(9680);
		NRF_LOG_INFO("LED turned OFF (high)");

		NRF_LOG_INFO("Sample 1: %d", sharpPM_value);
		NRF_LOG_INFO("sharpPM_value (mV): %d", sharpPM_value*adc_to_V*1000);
	}





	// Spec CO, Analog read.  TODO: implement averaging over 128 samples
	if (using_component(SPEC_CO, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing Spec CO...");
		NRF_LOG_INFO("------------------");

		// Pre-read wait, prevents garbage reading
		nrf_saadc_value_t specCO_temp;
		nrf_drv_saadc_sample_convert(SPEC_CO_CHANNEL_NUM, &specCO_temp);
		nrf_delay_ms(PRE_READ_WAIT);

		NRF_LOG_INFO("Sampling...");
		int specCO_total = 0;
		// read it a bunch of times and then average
		for (int i = 0; i < 128; i++) {
			nrf_drv_saadc_sample_convert(SPEC_CO_CHANNEL_NUM, &specCO_temp);
			specCO_total += specCO_temp;

			nrf_delay_ms(SPEC_CO_DELAY);
		}

		specCO_value = specCO_total/128.0f;
		NRF_LOG_INFO("specCO_value: %d", specCO_value);
		NRF_LOG_INFO("specCO_value (mV): %d", specCO_value*adc_to_V*1000);
	}


	// Figaro CO, Analog read.
	if (using_component(FIGARO_CO, components_used)) {
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


	// Check nRF Battery Level
	if (using_component(BATTERY, components_used)) {
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




	// Check Fuel Gauge (LiPo battery level), TWI (I2C)
	if (using_component(FUEL_GAUGE, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Checking Fuel Gauge with I2C/TWI...");
		NRF_LOG_INFO("-----------------------------------");

		err_code = 1;
		for (int i=0; (err_code) && (i < TWI_RETRY_NUM); i++) {

			err_code = read_fuel_gauge();

//			nrf_delay_ms(1000);

			if (err_code) {
				NRF_LOG_INFO("* RETRY: Fuel Gauge error, err_code=%d *", err_code);
				avoided_error_cnt++;
				nrf_delay_ms(TWI_RETRY_WAIT);
			}
		}

		if (err_code) {
			NRF_LOG_INFO("** ERROR: Fuel Gauge read, err_code=%d **", err_code);
			fuel_v_cell = 0;
			fuel_percent = 0;
			err_cnt++;
			fuel_gauge_error_cnt_total++;
		}

		// Read Fuel
		NRF_LOG_INFO("fuel_v_cell: %d", fuel_v_cell);
		NRF_LOG_INFO("fuel_percent: %d", fuel_percent);
	}


	// UV Sensor SI1145, TWI (I2C)
	if (using_component(UV_SI1145, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing UV Sensor SI1145 with I2C/TWI...");
		NRF_LOG_INFO("----------------------------------------");

		// Init sensor
		UV_SI1145_init();
		nrf_delay_ms(100);	// wait for a few auto reads

		// Read the data
		UV_SI1145_read();
		NRF_LOG_INFO("UV_SI1145_VIS_value: %d", UV_SI1145_VIS_value);
		NRF_LOG_INFO("UV_SI1145_IR_value: %d", UV_SI1145_IR_value);
		NRF_LOG_INFO("UV_SI1145_UV_value: %d", UV_SI1145_UV_value);
	}


	// Small Plantower, TWI (I2C)
	if (using_component(SMALL_PLANTOWER, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing Small Plantower with I2C/TWI...");
		NRF_LOG_INFO("---------------------------------------");
		// Wait for sensor to settle from when ADP turned on
		NRF_LOG_INFO("Waiting for startup wait..");
		while (!plantower_startup_wait_done) {}

		err_code = 1;
		for (int i=0; (err_code) && (i < TWI_RETRY_NUM); i++) {
			err_code = read_plantower_twi();
			if (err_code) {
				NRF_LOG_INFO("* RETRY: PLANTOWER ERROR, err_code=%d *", err_code);
				avoided_error_cnt++;
				nrf_delay_ms(TWI_RETRY_WAIT);
			}
		}

		if (err_code) {
			NRF_LOG_INFO("** ERROR: PLANTOWER read, err_code=%d **", err_code);
			plantower_2_5_value = 0;
			plantower_10_value = 0;
			err_cnt++;
			plantower_error_cnt_total++;
		}

		NRF_LOG_INFO("plantower_2_5_value = %d", plantower_2_5_value);
		NRF_LOG_INFO("plantower_10_value = %d", plantower_10_value);
	}


	/** Test HPM sensor (Serial comm) **/
	if (using_component(HONEYWELL, components_used)) {
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
				APP_ERROR_CHECK(err_code);
			}
			if (!nrf_drv_power_init_check() ) {
				NRF_LOG_INFO("Need to initialize the power driver..");
				err_code = nrf_drv_power_init(NULL);	// TODO: Maybe remove, unnecessary?
				APP_ERROR_CHECK(err_code);
			}
		nrf_drv_clock_lfclk_request(NULL);
		err_code = app_timer_init();	// needed for serial timeout checking
		NRF_LOG_INFO("err_code = %d", err_code);
		APP_ERROR_CHECK(err_code);
		err_code = nrf_serial_init(&serial_uart, &m_uart0_drv_config, &serial_config);
		NRF_LOG_INFO("err_code = %d", err_code);
		APP_ERROR_CHECK(err_code);

		// Need to send commands if not using autosend
		if (!USING_AUTOSEND) {

			NRF_LOG_INFO("Sending Command.. Disable Autosend");
			err_code = hpm_cmd_and_ack(hpm_stop_autosend_cmd);	//stop HPM autosend

			NRF_LOG_INFO("Sending Command.. Disable Autosend");
			err_code = hpm_cmd_and_ack(hpm_stop_autosend_cmd);	//Do again, since HPM may still be autosending, which may confuse cmd_ack
			NRF_LOG_INFO("Sending Command.. Start Measurement");
			err_code = hpm_cmd_and_ack(hpm_start_meas_cmd);	//start HPM
		}

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
	// Sharp PM, Turn LED ON (low)
	if (using_component(SHARP, components_used)) {
		nrf_gpio_cfg_output(SHARP_PM_LED);
		nrf_gpio_pin_clear(SHARP_PM_LED);
	}
//	// TWI (I2C) UNinit
//    nrf_drv_twi_uninit(&m_twi);
	// Turn OFF Sample LED
    nrf_gpio_cfg_output(SAMPLE_LED);
	nrf_gpio_pin_clear(SAMPLE_LED);	// Enable HIGH, Turn OFF LED

}


/**
 * @brief Test function.
 */
int test_main(component_type components_used[]) {

	// Start basic stuff
	NRF_LOG_INFO("");
	NRF_LOG_INFO("");
	NRF_LOG_INFO("-----------------------");
	NRF_LOG_INFO("| Testing test_main() |");
	NRF_LOG_INFO("-----------------------");
	NRF_LOG_INFO("loop_num = %d", loop_num);
	NRF_LOG_INFO("wdt_triggered = %d", wdt_triggered);


    // ADP, turn ON all power
	NRF_LOG_INFO("");
	NRF_LOG_INFO("WAKE: Turning ON ADP Power...");
	NRF_LOG_INFO("-----------------------------");

    nrf_gpio_cfg_output(ADP1_PIN);
    nrf_gpio_cfg_output(ADP2_PIN);
    nrf_gpio_cfg_output(STATUS_LED);
//	nrf_gpio_pin_clear(STATUS_LED);	// Enable LOW, Turn ON LED

	nrf_gpio_pin_set(ADP1_PIN);		// Enable HIGH
	NRF_LOG_INFO("ADP1_PIN: HIGH.");

//	while (1) {
//		// Wait for event.
//		__WFE();
//		NRF_LOG_INFO("Woke Temporarily");
//
//		// Clear Event Register.
//		__SEV();
////		NRF_LOG_INFO("Woke Temporarily");
//		__WFE();
////		NRF_LOG_INFO("Woke Temporarily");
//	}
//
//	nrf_delay_ms(10*1000);

	nrf_gpio_pin_set(ADP2_PIN);		// Enable HIGH
	NRF_LOG_INFO("ADP2_PIN: HIGH.");

//	nrf_delay_ms(10*1000);

	nrf_gpio_pin_set(STATUS_LED);	// Enable HIGH, Turn ON LED
	NRF_LOG_INFO("STATUS_LED: HIGH.");


//	nrf_delay_ms(500);
//	NRF_LOG_FLUSH();
//	nrf_delay_ms(500);


	// Start timer for DHT startup wait (settling time is ~1.5s)
	dht_startup_wait_done = 0;
	if (using_component(DHT, components_used)) {
		err_code = app_timer_start(dht_startup_timer, APP_TIMER_TICKS(DHT_STARTUP_WAIT_TIME), NULL);
		APP_ERROR_CHECK(err_code);
	}
    plantower_startup_wait_done = 0;
	if (using_component(SMALL_PLANTOWER, components_used)) {
		err_code = app_timer_start(plantower_startup_timer, APP_TIMER_TICKS(PLANTOWER_STARTUP_WAIT_TIME), NULL);
		APP_ERROR_CHECK(err_code);
	}
    hpm_startup_wait_done = 0;
	if (using_component(HONEYWELL, components_used)) {
		err_code = app_timer_start(hpm_startup_timer, APP_TIMER_TICKS(HPM_STARTUP_WAIT_TIME), NULL);
		APP_ERROR_CHECK(err_code);
	}


	// Initialize things before getting data
	// Initial delay so things can settle
	nrf_delay_ms(INITIAL_SETTLING_WAIT);
    // TWI (I2C) init
    twi_init();
    // Init Fuel Gauge
	if (using_component(FUEL_GAUGE, components_used)) {
		fuel_gauge_wake();	// Turn on after things have settled, but give it time to estimate
	    // Quick start once for initial guess
//		if (!has_quick_started_fuel_gauge) {
		if (0) {
			NRF_LOG_INFO("Quick starting fuel gauge..");
			fuel_gauge_quick_start();
			has_quick_started_fuel_gauge = true;
		}

	}


	// All the measurements happen here
	err_cnt = 0;
	avoided_error_cnt = 0;
	for (int i=0; i < NUM_SAMPLES_PER_ON_CYCLE; i++) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("-- SAMPLE #%d/%d --", i+1, NUM_SAMPLES_PER_ON_CYCLE);

		// Read all of the sensors
		get_data(components_used);
		// Save the data to SD card
		if (using_component(SDC, components_used)) {
			save_data();
		}

		// Wait between multiple samples as a buffer time, just in case
		if (NUM_SAMPLES_PER_ON_CYCLE > 1) {
			NRF_LOG_INFO("");
			NRF_LOG_INFO("Wait before next sample: %d ms", WAIT_BETWEEN_SAMPLES);
			nrf_delay_ms(WAIT_BETWEEN_SAMPLES);
		}

	}


	// Uninitialize things
	if (using_component(FUEL_GAUGE, components_used)) {
		fuel_gauge_sleep();	// Turn off to save power
	}
	// TWI (I2C) UNinit
    nrf_drv_twi_uninit(&m_twi);



    // ADP sleep, turn OFF all power
	if (using_component(ADP, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("SLEEP: Turning OFF ADP Power...");
		NRF_LOG_INFO("-------------------------------");

		nrf_gpio_cfg_output(ADP1_PIN);
		nrf_gpio_cfg_output(ADP2_PIN);
		NRF_LOG_INFO("LOW.");
		nrf_gpio_pin_clear(ADP1_PIN);	// Enable HIGH, Turn OFF ADP
		nrf_gpio_pin_clear(ADP2_PIN);	// Enable HIGH, Turn OFF ADP
	}
	nrf_gpio_cfg_output(STATUS_LED);
//	nrf_gpio_pin_set(STATUS_LED);	// Enable LOW, Turn OFF LED
	nrf_gpio_pin_clear(STATUS_LED);	// Enable HIGH, Turn OFF LED


	// FOR TESTING: Deep Sleep, NOTE: stuck here forever!
	if (using_component(DEEP_SLEEP, components_used)) {
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


	// Feed the Watchdog Timer
	NRF_LOG_INFO("");
	NRF_LOG_INFO("Feeding the Watchdog..");
	NRF_LOG_INFO("----------------------");
	NRF_LOG_INFO("WDT_TIMEOUT: %d ms", WDT_TIMEOUT);
	// check if SD card failed
	if (sd_write_failed && SD_FAIL_SHUTDOWN) {
		NRF_LOG_INFO("** FATAL ERROR: sd_write_failed: %d **", sd_write_failed);
//		NRF_LOG_FLUSH();
		// wait until wdt runs out
		while (1) {}
	}
    nrf_drv_wdt_channel_feed(wdt_channel_id);

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
 * @brief Runs everything.  Basically the main program
 */
int test_all(component_type components_used[])
{

	// Initial delay so Fuel Gauge starts with good initial guess
	if (using_component(FUEL_GAUGE, components_used)) {
		nrf_delay_ms(INITIAL_FUEL_GAUGE_WAIT);
	}


    /** Initialize general stuff	**/
    bsp_board_leds_init();
	// Turn OFF LED's
    nrf_gpio_cfg_output(SAMPLE_LED);
	nrf_gpio_pin_clear(SAMPLE_LED);	// Enable HIGH, Turn OFF LED
	nrf_gpio_cfg_output(STATUS_LED);
	nrf_gpio_pin_clear(STATUS_LED);	// Enable HIGH, Turn OFF LED

    // NRF_LOG setup
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();



    // TODO: Check USING_PLANTOWER && USING_HONEYWELL
	// ADC setup
    saadc_init();
//    // TWI (I2C) init
//    twi_init();
//    nrf_delay_ms(1000);
//    nrf_drv_twi_uninit(&m_twi);


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
    nrf_gpio_cfg_output(ADP1_PIN);
	nrf_gpio_pin_clear(ADP1_PIN);	// Enable HIGH
    nrf_gpio_cfg_output(ADP2_PIN);
	nrf_gpio_pin_clear(ADP2_PIN);	// Enable HIGH


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
		NRF_LOG_INFO("LOG_INTERVAL = %d", LOG_INTERVAL);
		NRF_LOG_INFO("*** test_main() COMPLETE!, next loop_num: %d ***", loop_num);
		NRF_LOG_FLUSH();

		nrf_delay_ms(LOG_INTERVAL);
	}

	NRF_LOG_INFO("");
	NRF_LOG_INFO("***** ALL LOOPING COMPLETE! *****");

	return NRF_SUCCESS;

}


/**
 * Test only the components needed for the SUM
 */
//int test_SUM(int entering_deep_sleep) {
//
//    // Startup Message
//	NRF_LOG_INFO("");
//	NRF_LOG_INFO("-----------------------");
//	NRF_LOG_INFO("| TEST SUM COMPONENTS |");
//	NRF_LOG_INFO("-----------------------");
//
//
//    // ADP, turn ON all power
//	NRF_LOG_INFO("");
//	NRF_LOG_INFO("WAKE: Turning ON ADP Power...");
//	NRF_LOG_INFO("-----------------------------");
//    nrf_gpio_cfg_output(ADP1_PIN);
//    nrf_gpio_cfg_output(ADP2_PIN);
//    nrf_gpio_cfg_output(STATUS_LED);
//    nrf_gpio_cfg_output(SAMPLE_LED);
//	NRF_LOG_INFO("HIGH.");
//
//	nrf_gpio_pin_set(ADP1_PIN);		// Enable HIGH
//	nrf_gpio_pin_set(ADP2_PIN);		// Enable HIGH
////	nrf_gpio_pin_clear(STATUS_LED);	// Enable LOW, Turn ON LED
//	nrf_gpio_pin_set(STATUS_LED);	// Enable HIGH, Turn ON LED
//	nrf_gpio_pin_set(SAMPLE_LED);	// Enable HIGH, Turn ON LED
//
//
//	// Test the internal temp reading
//	NRF_LOG_INFO("");
//	NRF_LOG_INFO("Internal Temp Reading");
//	NRF_LOG_INFO("---------------------");
//    int32_t temp_nrf = 0;
//	temp_nrf = get_temp_nrf();
//	NRF_LOG_INFO("temp_nrf = %d", temp_nrf);
////		nrf_delay_ms(500);
//
//
//	// Save the data to SD card
//	save_data();
//
//    // ADP sleep, turn OFF all power
//	NRF_LOG_INFO("");
//	NRF_LOG_INFO("SLEEP: Turning OFF ADP Power...");
//	NRF_LOG_INFO("-------------------------------");
//
//    nrf_gpio_cfg_output(ADP1_PIN);
//    nrf_gpio_cfg_output(ADP2_PIN);
//    nrf_gpio_cfg_output(STATUS_LED);
//    nrf_gpio_cfg_output(SAMPLE_LED);
//	NRF_LOG_INFO("LOW.");
//
//	nrf_gpio_pin_clear(ADP1_PIN);	// Enable HIGH, Turn OFF ADP
//	nrf_gpio_pin_clear(ADP2_PIN);	// Enable HIGH, Turn OFF ADP
//	nrf_gpio_pin_clear(STATUS_LED);	// Enable HIGH, Turn OFF LED
//	nrf_gpio_pin_clear(SAMPLE_LED);	// Enable HIGH, Turn OFF LED
//
//
//	// Deep Sleep
//	if (entering_deep_sleep) {
//		NRF_LOG_INFO("");
//		NRF_LOG_INFO("DEEP SLEEP: Turning OFF CPU...");
//		NRF_LOG_INFO("------------------------------");
//
//		while (1) {
//			// Wait for event.
//			__WFE();
//			NRF_LOG_INFO("Woke Temporarily");
//
//			// Clear Event Register.
//			__SEV();
//	//		NRF_LOG_INFO("Woke Temporarily");
//			__WFE();
//	//		NRF_LOG_INFO("Woke Temporarily");
//		}
//	}
//
//
//	return err_code;
//
//
//}


/**
 * Main program.  Comment whatever major things you want to run.
 */
int main(void) {

	component_type components_used[] = {
//			ADP,		// DIG
			SDC,		// SD card, SPIO
			BATTERY,	// AIN(no pin),	3279
			FUEL_GAUGE,	// I2C
			TEMP_NRF,	// REGISTER
			RTC,		// I2C
//			UV_SI1145,		// I2C
			BME,		// I2C
////			SHARP,		// AIN + DIG,	70
////			FIGARO_CO,	// AIN,			1643
			SMALL_PLANTOWER,	// I2C,	2
			SPEC_CO,	// AIN,			102
//			FIGARO_CO2,	// I2C,			579
////			DEEP_SLEEP,
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
