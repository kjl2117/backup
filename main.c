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

/**
 * Some function prototypes so we can rearrange code below
 */
static void test_all();
static ret_code_t set_rtc(uint8_t sec, uint8_t min, uint8_t hour, uint8_t wday, uint8_t date, uint8_t mon, uint8_t year);
void twi_init (void);
//static void restart_measurements(uint32_t temp_log_interval);
static void restart_measurements();
static void stop_measurements();



#define SUM				0
#define HAP				1
#define BATTERY_TEST	2
#define WAIT_TIME_TEST	3
#define CUSTOM			4



/** GLOBAL VARIABLES **/
//--------------------//

/** Overall **/
#define PRODUCT_TYPE	HAP	//	OPTIONS: SUM, HAP, BATTERY_TEST, WAIT_TIME_TEST, CUSTOM,
#define SETTING_TIME_MANUALLY		0		// set to 1, then set to 0 and flash; o/w will rewrite same time when reset
#define DELETE_ALL_FILES			0	// If we want to clear the SD card
#define RESET_VALUES_FILE			0	// If we want to delete the initial values, to use FW values instead
#define SD_FAIL_SHUTDOWN			1	// If true, will enter infinite loop when SD fails (and wdt will reset)
#define READING_VALUES_FILE			1	// If we want to read in saved values from the config file
static bool on_logging = false;	// true;	false;	Start with logging on or off (Also App can control this)
static int32_t log_interval = 300*1000;	//60*1000;	// units: ms
//static uint32_t log_interval = 10*1000;	// units: ms
#define PLANTOWER_STARTUP_WAIT_TIME		12*1000	//ms	~2.5s is min,	Total response time < 10s (30s after wakeup)
//#define PLANTOWER_STARTUP_WAIT_TIME		5*1000	//ms	~2.5s is min,	Total response time < 10s (30s after wakeup)
#define SPEC_CO_STARTUP_WAIT_TIME		25*1000	//ms,	Response time < 30s (15s typical)
//#define SPEC_CO_STARTUP_WAIT_TIME		5*1000	//ms,	Response time < 30s (15s typical)
#define FUEL_PERCENT_THRESHOLD			20	//20	// Start extrapolating after this threshold (%)
#define MIN_BATTERY_LEVEL				3400	//units: percent*1000, NOTE: set to 0 for BATTERY_TEST
//static uint16_t min_battery_level = 10*1000;	// units: percent*1000
#define FIGARO_CO2_STARTUP_WAIT_TIME	5*1000	// 2*1000 is too small, only reading value==360
//#define FIGARO_CO2_STARTUP_WAIT_TIME	3*1000	// 2*1000 is too small, only reading value==360
#define FUEL_GAUGE_RCOMP	0x97	// Config value to adjust for Custom batteries

#define NUM_SAMPLES_PER_ON_CYCLE	1	//1	//150	// 1,	20
//#define NUM_SAMPLES_PER_ON_CYCLE	150	//1	//150	// 1,	20
#define WAIT_BETWEEN_SAMPLES		0	//0	//200	// ms, waitin only if there are multiple samples
//#define WAIT_BETWEEN_SAMPLES		200	//0	//200	// ms, waitin only if there are multiple samples
#define INITIAL_FUEL_GAUGE_WAIT		1000	// <500 causes issues?
#define INITIAL_MSG_WAIT			1000	// <500 causes issues?
#define INITIAL_SETTLING_WAIT		1000	// <500 causes issues?

//#define LOG_INTERVAL				10*1000		// ms between sleep/wake
//#define LOG_INTERVAL				1000*1000		// ms between sleep/wake
//#define DEVICE_NAME                     "SENSEN_UART"                               /**< Name of device. Will be included in the advertising data. */
//#define DEVICE_NAME                     "BATTERY_UART"                               /**< Name of device. Will be included in the advertising data. */
#define VALUES_FILE_NAME   "_values.txt"
#define INFO_FILE_NAME   "info.txt"
#define EXTRA_LOG_FILE_NAME   "extralog.txt"
//#define BLE_TEST_FILE_NAME   "ble_test.TXT"
//#define BLE_TEST_FILE_NAME   "ble_test_100.TXT"
#define BLE_TEST_FILE_NAME   "ble_150k.TXT"	//"ble_150k.TXT"	//"ble_2000.TXT"	//"ble_100.TXT"	"ble_2000.TXT"
#define LOG_FILE_NAME   "datalog.txt"
static char ble_file_name[] = LOG_FILE_NAME;	//BLE_TEST_FILE_NAME;

#define DHT_STARTUP_WAIT_TIME			0.1*1000	//ms
#define HPM_STARTUP_WAIT_TIME			6*1000	//ms,	6s (10-15s recommended) for Honeywell; 10s for Plantower
#define UVA_VEML_MEAS_DELAY		500	// Time to wait before measuring to allow for integration: <500 doesn't really measure anything


// The different sensors (used for choosing which sensor code to run)
typedef enum {
	DHT,		// DIG
	GPIO,		// DIG
	ADP,		// DIG
	ADP_HIGH,	// DIG
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
	AMBIENT_LTR,	// I2C
	UVA_VEML,			// I2C	(NOTE: has 2 addr)
	DEEP_SLEEP,
	SDC,		// SD card, SPIO
} component_type;

typedef enum {
	BAT_LIPO_1200mAh,
	BAT_LIPO_2000mAh,
	BAT_LIPO_10Ah,
} battery_type;
//static battery_type battery_type_used = BAT_LIPO_1200mAh;

#if PRODUCT_TYPE == SUM
	static component_type components_used[] = {
		ADP,		// DIG
//		ADP_HIGH,	// DIG, for switching only High Power sensors
		SDC,		// SD card, SPIO
		BATTERY,	// AIN(no pin),	3279
		FUEL_GAUGE,	// I2C
		TEMP_NRF,	// REGISTER
		RTC,		// I2C
	//////				UV_SI1145,		// I2C
		AMBIENT_LTR,	// I2C
		UVA_VEML		// I2C
	};
	static int components_used_size = sizeof(components_used) / sizeof(components_used[0]);
	static battery_type battery_type_used = BAT_LIPO_1200mAh;
	static uint16_t min_battery_level = MIN_BATTERY_LEVEL;	// units: percent*1000
	#define DEVICE_NAME                     "SUM_SENSEN"                               /**< Name of device. Will be included in the advertising data. */
	#define PRODUCT_STR		"SUM"
	static char product_FW_version[] = 		"SUM_v1.00";
	#define LIVE_STREAM_LOG_INTERVAL	5*1000	//ms
	static uint32_t max_sensor_wait_ms = 2*UVA_VEML_MEAS_DELAY;
#elif PRODUCT_TYPE == HAP
	static component_type components_used[] = {
		ADP,		// DIG
		ADP_HIGH,	// DIG, for switching only High Power sensors
		SDC,		// SD card, SPIO
		BATTERY,	// AIN(no pin),	3279
		FUEL_GAUGE,	// I2C
		TEMP_NRF,	// REGISTER
		RTC,		// I2C
			BME,		// I2C
			SMALL_PLANTOWER,	// I2C,	2
			SPEC_CO,	// AIN,			102
			FIGARO_CO2,	// I2C,			579
	};
	static int components_used_size = sizeof(components_used) / sizeof(components_used[0]);
	static battery_type battery_type_used = BAT_LIPO_10Ah;
	static uint16_t min_battery_level = MIN_BATTERY_LEVEL;	// units: percent*1000
	#define DEVICE_NAME                     "HAP_SENSEN"                               /**< Name of device. Will be included in the advertising data. */
	#define PRODUCT_STR		"HAP"
	static char product_FW_version[] = 		"HAP_v1.00";
	#define LIVE_STREAM_LOG_INTERVAL	5*1000	//ms
	static uint32_t max_sensor_wait_ms = (PLANTOWER_STARTUP_WAIT_TIME > SPEC_CO_STARTUP_WAIT_TIME) ? PLANTOWER_STARTUP_WAIT_TIME : SPEC_CO_STARTUP_WAIT_TIME;
#elif PRODUCT_TYPE == BATTERY_TEST
	static component_type components_used[] = {
//		ADP,		// DIG
//		ADP_HIGH,	// DIG, for switching only High Power sensors
		SDC,		// SD card, SPIO
		BATTERY,	// AIN(no pin),	3279
		FUEL_GAUGE,	// I2C
		TEMP_NRF,	// REGISTER
		RTC,		// I2C
	//////				UV_SI1145,		// I2C
			BME,		// I2C
			SMALL_PLANTOWER,	// I2C,	2
			SPEC_CO,	// AIN,			102
			FIGARO_CO2,	// I2C,			579
	//////			DEEP_SLEEP,
	};
	static int components_used_size = sizeof(components_used) / sizeof(components_used[0]);
	static battery_type battery_type_used = BAT_LIPO_1200mAh;	//BAT_LIPO_1200mAh;
	static uint16_t min_battery_level = 0;	//MIN_BATTERY_LEVEL;	// units: percent*1000
	#define DEVICE_NAME                     "BATT_SENSEN"                               /**< Name of device. Will be included in the advertising data. */
	#define PRODUCT_STR		"BATTERY_TEST"
	static char product_FW_version[] = 		"BATTERY_TEST_v1.00";
	#define LIVE_STREAM_LOG_INTERVAL	5*1000	//ms
	static uint32_t max_sensor_wait_ms = (PLANTOWER_STARTUP_WAIT_TIME > SPEC_CO_STARTUP_WAIT_TIME) ? PLANTOWER_STARTUP_WAIT_TIME : SPEC_CO_STARTUP_WAIT_TIME;
#elif PRODUCT_TYPE == WAIT_TIME_TEST
	static component_type components_used[] = {
		ADP,		// DIG
//		ADP_HIGH,	// DIG, for switching only High Power sensors
		SDC,		// SD card, SPIO
		BATTERY,	// AIN(no pin),	3279
		FUEL_GAUGE,	// I2C
		TEMP_NRF,	// REGISTER
		RTC,		// I2C
	//////				UV_SI1145,		// I2C
			BME,		// I2C
			SMALL_PLANTOWER,	// I2C,	2
			SPEC_CO,	// AIN,			102
			FIGARO_CO2,	// I2C,			579
	//////			DEEP_SLEEP,
	};
	static int components_used_size = sizeof(components_used) / sizeof(components_used[0]);
	static battery_type battery_type_used = BAT_LIPO_10Ah;	//BAT_LIPO_1200mAh;
	static uint16_t min_battery_level = 0;	// units: percent*1000
	#define DEVICE_NAME                     "WAIT_SENSEN"                               /**< Name of device. Will be included in the advertising data. */
	#define PRODUCT_STR		"WAIT_TIME_TEST"
	static char product_FW_version[] = 		"WAIT_TIME_TEST_v1.00";
	#define LIVE_STREAM_LOG_INTERVAL	5*1000	//ms
	static uint32_t max_sensor_wait_ms = (PLANTOWER_STARTUP_WAIT_TIME > SPEC_CO_STARTUP_WAIT_TIME) ? PLANTOWER_STARTUP_WAIT_TIME : SPEC_CO_STARTUP_WAIT_TIME;
#else
	static component_type components_used[] = {
		ADP,		// DIG
		ADP_HIGH,	// DIG, for switching only High Power sensors
		SDC,		// SD card, SPIO
		BATTERY,	// AIN(no pin),	3279
		FUEL_GAUGE,	// I2C
		TEMP_NRF,	// REGISTER
		RTC,		// I2C
	//////				UV_SI1145,		// I2C
		AMBIENT_LTR,	// I2C
		UVA_VEML,		// I2C
			BME,		// I2C
			SMALL_PLANTOWER,	// I2C,	2
			SPEC_CO,	// AIN,			102
			FIGARO_CO2,	// I2C,			579
	//////			DEEP_SLEEP,
	};
	static int components_used_size = sizeof(components_used) / sizeof(components_used[0]);
	static battery_type battery_type_used = BAT_LIPO_10Ah;
	static uint16_t min_battery_level = MIN_BATTERY_LEVEL;	// units: percent*1000
	#define DEVICE_NAME                     "UART_SENSEN"                               /**< Name of device. Will be included in the advertising data. */
	#define PRODUCT_STR		"CUSTOM"
	static char product_FW_version[] = 		"CUSTOM_v1.00";
	#define LIVE_STREAM_LOG_INTERVAL	5*1000	//ms
	static uint32_t max_sensor_wait_ms = (PLANTOWER_STARTUP_WAIT_TIME > SPEC_CO_STARTUP_WAIT_TIME) ? PLANTOWER_STARTUP_WAIT_TIME : SPEC_CO_STARTUP_WAIT_TIME;
#endif

// NOTE: This MUST correspond to battery_type above!
//static float battery_scale_factors[] = {
//		2.068,	// BAT_LIPO_1200mAh
//		1.0,	// BAT_LIPO_2000mAh,	NOT USED
//		2.589,	// BAT_LIPO_10Ah
//};
//static float battery_scale_factor = battery_scale_factors[battery_type_used];
static float m_batt[] = {
		0.001344399,	// BAT_LIPO_1200mAh
		1.0,			// BAT_LIPO_2000mAh,	NOT USED
		0.001311254,	// BAT_LIPO_10Ah
};
static float b_batt[] = {
		-4.473748806,	// BAT_LIPO_1200mAh
		0.0,			// BAT_LIPO_2000mAh,	NOT USED
		-4.37601246,	// BAT_LIPO_10Ah
};


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
#define MAX_OUT_STR_SIZE	200
#if PRODUCT_TYPE == SUM
	#define FILE_HEADER			"Time, rtc_temp, ambient_CH0, ambient_CH1, uva_value, fuel_v_cell, fuel_percent\r\n"
	#define FILE_HEADER_EXTRA	"Time, temp_nrf, battery_value, fuel_percent, fuel_percent_raw, err_cnt\r\n"
#elif PRODUCT_TYPE == HAP
	#define FILE_HEADER			"Time, specCO, figaroCO2, plantower_2_5_value, plantower_10_value, bme_temp_C, bme_humidity, fuel_v_cell, fuel_percent\r\n"
	#define FILE_HEADER_EXTRA	"Time, bme_pressure, rtc_temp, temp_nrf, battery_value, fuel_percent, fuel_percent_raw, err_cnt\r\n"
#else
	#define FILE_HEADER			"Time,PM2_5,PM10,sharpPM,dhtTemp,dhtHum,specCO,figaroCO,figaroCO2,plantower_2_5_value,plantower_10_value, bme_temp_C, bme_humidity, bme_pressure, rtc_temp,temp_nrf, ambient_CH0, ambient_CH1, uva_value, battery_value, fuel_v_cell, fuel_percent, fuel_percent_raw, runtime_estimate, fuel_t0, fuel_p0, t0, err_cnt, dht_error_cnt_total, hpm_error_cnt_total\r\n"
	#define FILE_HEADER_EXTRA	"Time,PM2_5,PM10,sharpPM,dhtTemp,dhtHum,specCO,figaroCO,figaroCO2,plantower_2_5_value,plantower_10_value, bme_temp_C, bme_humidity, bme_pressure, rtc_temp,temp_nrf, ambient_CH0, ambient_CH1, uva_value, battery_value, fuel_v_cell, fuel_percent, fuel_percent_raw, runtime_estimate, fuel_t0, fuel_p0, t0, err_cnt, dht_error_cnt_total, hpm_error_cnt_total\r\n"
#endif
static bool testing_sensors = false;	// Run through all sensors once in the beginning to check (but don't save to SDC)

/** SD Card Data Offload Variables **/
#define SDC_BUFF_SIZE		10*1000	//100
static uint8_t sdc_buff[SDC_BUFF_SIZE];	// = {0};
static uint16_t sdc_buff_current_pos = 0;
static uint32_t bytes_remaining = 0;
static uint16_t packet_length = 0;
//static bool resending_packets = false;
static bool start_sending_sdc_data = false;
//static bool keep_sending_sdc_packets = false;
static bool done_reading_sdc = true;
static bool done_sending_sdc = true;
static int sdc_read_num = 0;
//static uint32_t sdc_bytes_read = 0;
static uint32_t start_byte = 0;
static uint32_t end_byte = 0;
//static bool updating_end_byte = false;
static bool in_measuring_loop = false;
static bool is_offloading = false;
//static uint8_t * broadcast_data;
typedef struct
{
	uint32_t err_cnt_total;
	bool is_logging;
	uint32_t fuel_percent;
} sensen_broadcast_data;
static sensen_broadcast_data broadcast_data;
//static uint8_t broadcast_data = 0xFF;
//static uint8_array_t broadcast_data = {0};

/** SD Card Config File Variables **/
static bool updating_values_file = false;

/** FatFs Variables **/
static FATFS fs;
static FIL file;
FRESULT ff_result;
char* TEST_STRING = "SD card test, v09.\r\n";
static int header_is_written = 0;
static bool sd_write_failed = false;
static bool skipping_sdc_write = false;

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
static int16_t figCO2_value;	// units: ppm
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
static int32_t time_now = 0;
static int32_t t0 = 0;
static int16_t rtc_temp = 0;	// units: degC*100, precision +/- 0.25C
static int time_was_set = 0;
static bool setting_new_time = false;
static bool adjusting_timer_start = false;

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
static float adc_to_mV;	// 0.003515625 == 1.0f / ((ADC_GAIN_VALUE / ADC_REFERENCE_VOLTAGE) * pow(2, ADC_RESOLUTION_BITS));
static int V_to_adc_1000;	// 284.166*1000 == (ADC_GAIN_VALUE / ADC_REFERENCE_VOLTAGE) * pow(2, ADC_RESOLUTION_BITS);
#define PRE_READ_WAIT			5	// ms, wait before an Analog read to let transients settle after switching to high impedance of AIN

/** GPIO Variables **/
#define GPIO_TEST_DELAY		2	// LED on time (ms)

/** Sharp PM Variables **/
#define SHARP_PM_CHANNEL_NUM		1
static nrf_saadc_value_t sharpPM_value;

/** Spec CO Variables **/
#define SPEC_CO_CHANNEL_NUM		2
#define SPEC_CO_DELAY			2	//ms, wait between samples that will be avg'ed
static int16_t specCO_value;	// units: ppm
//#define SPEC_CO_WAIT_BETWEEN_SAMPLES			10*1000	//ms, wait between multiple samples
//static int16_t specCO_value_10;	// units: ppm
//static int16_t specCO_value_20;	// units: ppm
//static int16_t specCO_value_30;	// units: ppm
//static int16_t specCO_value_40;	// units: ppm
//static int16_t specCO_value_50;	// units: ppm
//static int16_t specCO_value_60;	// units: ppm

/** Figaro CO Variables **/
#define FIG_CO_CHANNEL_NUM		3
static nrf_saadc_value_t figCO_value;

/** Battery Check Variables **/
#define BATTERY_CHANNEL_NUM		4
static nrf_saadc_value_t battery_value;

/** Fuel Gauge Check Variables (TWI) **/
#if PRODUCT_TYPE == SUM
	const int fuel_addr = 0x36;	//0x32;	// 0x36; // 8bit I2C address, 0x68 for RTC
#elif PRODUCT_TYPE == HAP
	const int fuel_addr = 0x32;	//0x32;	// 0x36; // 8bit I2C address, 0x68 for RTC
#elif PRODUCT_TYPE == BATTERY_TEST
	const int fuel_addr = 0x32;	//0x32;	// 0x36; // 8bit I2C address, 0x68 for RTC
#elif PRODUCT_TYPE == WAIT_TIME_TEST
	const int fuel_addr = 0x32;	//0x32;	// 0x36; // 8bit I2C address, 0x68 for RTC
#else
//	const int fuel_addr = 0x36;	//0x32;	// 0x36; // 8bit I2C address, 0x68 for RTC
	const int fuel_addr = 0x32;	//0x32;	// 0x36; // 8bit I2C address, 0x68 for RTC
#endif
static uint16_t fuel_v_cell = 0;	// units: mV
static uint32_t fuel_percent = 0;	// units: percent*1000
static uint32_t fuel_percent_raw = 0;	// units: percent*1000
//#define FUEL_SCALE_FACTOR		2.589	// Factor of how much time expansion
//#define FLOAT_FACTOR			1000
//static uint32_t runtime_estimate = 0;	// units: percent*1000
//static uint32_t fuel_p0 = 0;	// units: percent*1000
//static uint32_t fuel_t0 = 0;	// units: percent*1000
#define MAX17043_to_mV	1.25
//static bool has_quick_started_fuel_gauge = false;

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

/** Ambient Light Sensor LTR-329ALS-01 (TWI) **/
#define LTR303_ADDR   0x29 // default address
#define LTR303_CONTR         0x80
#define LTR303_MEAS_RATE     0x85
#define LTR303_DATA_CH1_0    0x88
#define LTR303_DATA_SIZE	4
//#define LTR303_DATA_CH1_1    0x89
//#define LTR303_DATA_CH0_0    0x8A
//#define LTR303_DATA_CH0_1    0x8B
static uint8_t LTR329_gain = 0;					// default 0: 1X
static uint8_t LTR329_integration_time = 1;		// default 0: 100ms, but also try 1: 50ms
static uint8_t LTR329_meas_rate = 1;			// default 3: 500ms, but also try 1: 100ms
static uint16_t ambient_CH0;
static uint16_t ambient_CH1;

/** UVA Light Sensor VEML6070 (TWI) **/
#define VEML6070_ADDR_H 0x39 ///< High address
#define VEML6070_ADDR_L 0x38 ///< Low address
static uint8_t UVA_integration_time = 3;	// default 1
static uint16_t uva_value;



/** Small Plantower (TWI) **/
#define PLANTOWER_TWI_BUFF_SIZE		32
const int plantower_twi_addr = 0x12; // 8bit I2C address, 0x12 for Small Plantower
static int16_t plantower_2_5_value = 0;	// units: ug/m^3
static int16_t plantower_10_value = 0;	// units: ug/m^3


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
nrf_drv_wdt_channel_id wdt_meas_channel_id;
nrf_drv_wdt_channel_id wdt_sleep_channel_id;
#define WDT_TIMEOUT_MEAS		60*1000 + NUM_SAMPLES_PER_ON_CYCLE*(WAIT_BETWEEN_SAMPLES+1000) + (DHT_STARTUP_WAIT_TIME + PLANTOWER_STARTUP_WAIT_TIME + SPEC_CO_STARTUP_WAIT_TIME + HPM_STARTUP_WAIT_TIME + FIGARO_CO2_STARTUP_WAIT_TIME)	// ms, make it more than DHT and HPM delays
//#define WDT_TIMEOUT_MEAS		15*1000
//#define WDT_TIMEOUT_SLEEP		LOG_INTERVAL + WDT_TIMEOUT_MEAS
static int wdt_triggered = 0;

/** APP TIMERS **/
static int dht_startup_wait_done = 0;
APP_TIMER_DEF(dht_startup_timer);
static int hpm_startup_wait_done = 0;
static volatile bool skipping_next_meas_loop = false;
APP_TIMER_DEF(hpm_startup_timer);
static volatile bool meas_loop_wait_done = false;
static volatile bool plantower_startup_wait_done = false;
static volatile bool specCO_startup_wait_done = false;
static volatile bool figCO2_startup_wait_done = false;
static volatile bool start_adjustment_wait_done = false;
APP_TIMER_DEF(meas_loop_timer);
APP_TIMER_DEF(plantower_startup_timer);
APP_TIMER_DEF(specCO_startup_timer);
APP_TIMER_DEF(figCO2_startup_timer);
APP_TIMER_DEF(start_adjustment_timer);
static uint32_t plantower_startup_wait_ms = PLANTOWER_STARTUP_WAIT_TIME;
static uint32_t specCO_startup_wait_ms = SPEC_CO_STARTUP_WAIT_TIME;
static uint32_t figaroCO2_startup_wait_ms = FIGARO_CO2_STARTUP_WAIT_TIME;


//APP_TIMER_DEF(m_our_char_timer_id);

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




/**
 * BLE defines from ble_app_uart
 */
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

//#define DEVICE_NAME                     "BATTERY_UART"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

//#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
//#define APP_ADV_INTERVAL                480                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_INTERVAL                MSEC_TO_UNITS(1000, UNIT_0_625_MS)                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 1000 ms). */
//#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */
//#define APP_ADV_TIMEOUT_IN_SECONDS      500                                         /**< The advertising timeout (in units of seconds). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                         /**< The advertising timeout (in units of seconds). */

//#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
//#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(8, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(12, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */

#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
//#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(10000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */
//#define MAX_CONN_PARAMS_UPDATE_COUNT    10                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//BLE_NUS_DEF(m_nus);                                                                 /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
//static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
//{
//    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
//};

#define BLE_TX_PACKET_SIZE			20
static ble_gap_addr_t ble_gap_address;


/**
 * BLE Sensen services
 */
//#define BLE_UUID_SENSEN_BASE_UUID              {0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00} // 128-bit base UUID
//#define BLE_UUID_SENSEN_BASE_UUID              {{0x00, 0x00, 0x73, 0x76, 0x73, 0x65, 0x73, 0x65, 0x6e, 0x73, 0x65, 0x6e, 0x20, 0x69, 0x6e, 0x63}} // 128-bit base UUID
//#define BLE_UUID_SENSEN_BASE_UUID              {{0x63, 0x6e, 0x69, 0x20, 0x6e, 0x65, 0x73, 0x6e, 0x65, 0x73, 0x65, 0x73, 0x76, 0x73, 0x00, 0x00}} // 128-bit base UUID
#define BLE_UUID_SENSEN_BASE_UUID              {{0x65, 0x6c, 0x62, 0x63, 0x6e, 0x69, 0x6e, 0x65, 0x73, 0x6e, 0x65, 0x73, 0x70, 0x68, 0x64, 0x69}} // 128-bit base UUID, "idhpsensenincble"
//#define BLE_UUID_SENSEN_BASE_UUID              0x00007376736573656e73656e20696e63 // 128-bit base UUID
//#define BLE_UUID_SENSEN_SERVICE			0x7376 // Just a random, but recognizable value
#define BLE_UUID_SENSEN_SERVICE				0x7373 // "ss"
#define BLE_UUID_SUM_SERVICE				0x736d // "sm"
#define BLE_UUID_HAP_SERVICE				0x6870 // "hp"
#define BLE_UUID_CUSTOM_SERVICE				0x636d // "cm"

#define BLE_UUID_OUR_CHARACTERISTIC_UUID		0xBEEF // Custom (from example)
#define BLE_UUID_FW_VER_CHARACTERISTIC			0x0000 // The Firmware Version

// Sensen Service specific chars
#define BLE_UUID_ON_LOGGING_CHARACTERISTIC		0x0010 // The Firmware Version
#define BLE_UUID_LOG_INTERVAL_CHARACTERISTIC	0x0011 // The Firmware Version
#define BLE_UUID_BATT_PERCENT_CHARACTERISTIC	0x0012 // The Firmware Version
#define BLE_UUID_MIN_BATT_CHARACTERISTIC		0x0013 // The Firmware Version
#define BLE_UUID_RTC_CHARACTERISTIC				0x0014 // The Firmware Version
#define BLE_UUID_DATA_OFFLOAD_CHARACTERISTIC	0x0015 // The Firmware Version
#define BLE_UUID_START_BYTE_CHARACTERISTIC		0x0016 // The Firmware Version
#define BLE_UUID_END_BYTE_CHARACTERISTIC		0x0017 // The Firmware Version
#define BLE_UUID_IS_OFFLOADING_CHARACTERISTIC	0x0018 // The Firmware Version
#define BLE_UUID_BATT_VOLTAGE_CHARACTERISTIC	0x0019 // The Firmware Version

// Product specific chars
#define BLE_UUID_PM2_5_CHARACTERISTIC			0x0020 // plantower_2_5_value
#define BLE_UUID_PM10_CHARACTERISTIC			0x0021 // The Firmware Version
#define BLE_UUID_CO_CHARACTERISTIC				0x0022 // The Firmware Version
#define BLE_UUID_CO2_CHARACTERISTIC				0x0023 // The Firmware Version
#define BLE_UUID_RH_CHARACTERISTIC				0x0024 // The Firmware Version
#define BLE_UUID_TEMP_BME_CHARACTERISTIC		0x0025 // The Firmware Version
#define BLE_UUID_TEMP_RTC_CHARACTERISTIC		0x0026 // The Firmware Version
#define BLE_UUID_TEMP_NRF_CHARACTERISTIC		0x0027 // The Firmware Version
#define BLE_UUID_AMBIENT_CH0_CHARACTERISTIC		0x0028 // The Firmware Version
#define BLE_UUID_AMBIENT_CH1_CHARACTERISTIC		0x0029 // The Firmware Version
#define BLE_UUID_UVA_CHARACTERISTIC				0x002A // The Firmware Version
#define BLE_UUID_PLANTOWER_WAIT_CHARACTERISTIC	0x002B // The Firmware Version
#define BLE_UUID_SPEC_CO_WAIT_CHARACTERISTIC	0x002C // The Firmware Version
#define BLE_UUID_FIGARO_CO2_WAIT_CHARACTERISTIC	0x002D // The Firmware Version

// FW versions
static char sensen_FW_version[] = 	"SS_v1.00";
//static char SUM_FW_version[] = 		"SUM_v1.00";
//static char HAP_FW_version[] = 		"HAP_v1.00";

// Sensen Service values
static bool is_logging = false;			// Whether we are currently logging
//static bool on_logging = false;	// Whether the App wants to turn on logging
//static uint32_t log_interval = LOG_INTERVAL;	// units: ms
//static uint32_t min_battery_level = 0;//10*1000;	// units: percent*1000
static int32_t time_to_be_set = 0;
static bool is_live_streaming = false;	// Used to skip stuff we don't want when live streaming
static bool using_live_stream_interval = false;
//#define LIVE_STREAM_LOG_INTERVAL	5*1000	//ms
#define APP_PUSH_RETRY_NUM			5
#define APP_PUSH_RETRY_WAIT			100

#define DATA_OFFLOAD_PACKET_SIZE	20

// LiveStream variables
static int16_t previous_plantower_2_5_value = 0; // Declare a variable to store current temperature until next measurement.
static int16_t previous_plantower_10_value = 0;
static int16_t previous_specCO_value = 0;
static int16_t previous_figCO2_value = 0;
static int16_t previous_bme_humidity = 0;
static int16_t previous_bme_temp_C = 0;
static int16_t previous_rtc_temp = 0;
static int16_t previous_temp_nrf = 0;
static int16_t previous_ambient_CH0 = 0;
static int16_t previous_ambient_CH1 = 0;
static int16_t previous_UVA_value = 0;


// Structure for our custom service (generic)
// NOTE: bool values are by default set to 0 in custom_service_init()
typedef struct
{
	// Common chars for all
	uint16_t                    conn_handle;
	uint16_t                    service_handle;
	ble_gatts_char_handles_t    char_handles;
	ble_gatts_char_handles_t    fw_ver_handles;
	// Sensen Service specific chars
	ble_gatts_char_handles_t    on_logging_handles;
	ble_gatts_char_handles_t    log_rate_handles;
	ble_gatts_char_handles_t    batt_percent_handles;
	ble_gatts_char_handles_t    min_batt_handles;
	ble_gatts_char_handles_t    rtc_handles;
	ble_gatts_char_handles_t    data_offload_handles;
	ble_gatts_char_handles_t    start_byte_handles;
	ble_gatts_char_handles_t    end_byte_handles;
	ble_gatts_char_handles_t    is_offloading_handles;
	ble_gatts_char_handles_t    batt_voltage_handles;
	// Product specific chars
	ble_gatts_char_handles_t    pm2_5_handles;
	ble_gatts_char_handles_t    pm10_handles;
	ble_gatts_char_handles_t    co_handles;
	ble_gatts_char_handles_t    co2_handles;
	ble_gatts_char_handles_t    rh_handles;
	ble_gatts_char_handles_t    temp_bme_handles;
	ble_gatts_char_handles_t    temp_rtc_handles;
	ble_gatts_char_handles_t    temp_nrf_handles;
	ble_gatts_char_handles_t    plantower_wait_handles;
	ble_gatts_char_handles_t    specCO_wait_handles;
	ble_gatts_char_handles_t    figaroCO2_wait_handles;
	ble_gatts_char_handles_t    ambient_CH0_handles;
	ble_gatts_char_handles_t    ambient_CH1_handles;
	ble_gatts_char_handles_t    uva_handles;
} ble_custom_service_t;

static ble_custom_service_t m_SS_service;
//static ble_custom_service_t * p_product_specific_service;
static ble_custom_service_t product_service;


//static ble_custom_service_t m_SUM_service;
//static ble_custom_service_t m_HAP_service;
//
//#if PRODUCT_TYPE == SUM
//	static ble_custom_service_t * p_product_specific_service = &m_SUM_service;
//#elif PRODUCT_TYPE == HAP
//	static ble_custom_service_t * p_product_specific_service = &m_HAP_service;
//#elif PRODUCT_TYPE == BATTERY_TEST
//	static ble_custom_service_t * p_product_specific_service = &m_HAP_service;
//#elif PRODUCT_TYPE == WAIT_TIME_TEST
//	static ble_custom_service_t * p_product_specific_service = &m_HAP_service;
//#else
//	static ble_custom_service_t * p_product_specific_service = &m_HAP_service;
//#endif

//static ble_custom_service_t * p_product_specific_service = &m_HAP_service;
////static ble_custom_service_t * p_product_specific_service = &m_SUM_service;

//#define LIVE_STREAM_UPDATE_INTERVAL		1000


///**@brief Function for adding our new characterstic to "Our service" that we initiated in the previous tutorial.
// *
// * @param[in]   p_our_service        Our Service structure.
// *
// */
//static uint32_t our_char_add(ble_custom_service_t * p_our_service)
//{
//    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions
//
//    // OUR_JOB: Step 2.A, Add a custom characteristic UUID
//    ble_uuid_t          char_uuid;
//    ble_uuid128_t       base_uuid = BLE_UUID_SENSEN_BASE_UUID;
//    char_uuid.uuid      = BLE_UUID_OUR_CHARACTERISTIC_UUID;
//    sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
//    APP_ERROR_CHECK(err_code);
//
//    // OUR_JOB: Step 2.F Add read/write properties to our characteristic
//    // NOTE: R/W permission is set here, but is not necessarily enforced by SoftDevice
//    ble_gatts_char_md_t char_md;
//    memset(&char_md, 0, sizeof(char_md));
//    char_md.char_props.read = 1;
//    char_md.char_props.write = 1;
//
//
//    // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
//    ble_gatts_attr_md_t cccd_md;
//    memset(&cccd_md, 0, sizeof(cccd_md));
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
//    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;
//    char_md.p_cccd_md           = &cccd_md;
//    char_md.char_props.notify   = 1;
//
//
//    // OUR_JOB: Step 2.B, Configure the attribute metadata
//    ble_gatts_attr_md_t attr_md;
//    memset(&attr_md, 0, sizeof(attr_md));
//    attr_md.vloc        = BLE_GATTS_VLOC_STACK;
//    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
//    // NOTE: R/W permission is set here
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
//
//
//    // OUR_JOB: Step 2.C, Configure the characteristic value attribute
//    ble_gatts_attr_t    attr_char_value;
//    memset(&attr_char_value, 0, sizeof(attr_char_value));
//    attr_char_value.p_uuid      = &char_uuid;
//    attr_char_value.p_attr_md   = &attr_md;
//
//    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes
//    attr_char_value.max_len     = 4;
//    attr_char_value.init_len    = 4;
//    uint8_t value[4]            = {0x12,0x34,0x56,0x78};
//    attr_char_value.p_value     = value;
//
//    // OUR_JOB: Step 2.E, Add our new characteristic to the service
//    err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
//                                       &char_md,
//                                       &attr_char_value,
//                                       &p_our_service->char_handles);
//    APP_ERROR_CHECK(err_code);
//
//    printf("\r\nService handle: %#x\n\r", p_our_service->service_handle);
//    printf("Char value handle: %#x\r\n", p_our_service->char_handles.value_handle);
//    printf("Char cccd handle: %#x\r\n\r\n", p_our_service->char_handles.cccd_handle);
//
//    return NRF_SUCCESS;
//}

static uint32_t custom_char_add(	ble_custom_service_t * p_our_service,
									uint16_t custom_uuid,
									ble_gatts_char_handles_t * p_our_char_handles,
									uint8_t * p_our_value,	// pointer to where the data value is
									uint16_t p_our_value_size,
									uint8_t read,
									uint8_t write,
									uint8_t notify
									)
{
    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions

    // OUR_JOB: Step 2.A, Add a custom characteristic UUID
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_SENSEN_BASE_UUID;
    char_uuid.uuid      = custom_uuid;
    sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);

    // OUR_JOB: Step 2.F Add read/write properties to our characteristic
    // NOTE: R/W permission is set here, but is not necessarily enforced by SoftDevice
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = read;
    char_md.char_props.write = write;


    // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    if (notify == 1) {
		ble_gatts_attr_md_t cccd_md;
		memset(&cccd_md, 0, sizeof(cccd_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
		cccd_md.vloc                = BLE_GATTS_VLOC_STACK;
		char_md.p_cccd_md           = &cccd_md;
		char_md.char_props.notify   = 1;
    }

    // OUR_JOB: Step 2.B, Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc        = BLE_GATTS_VLOC_USER;
    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
    // NOTE: R/W permission is set here
    if (read == 1) {
    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    } else {
    	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    }
    if (write == 1) {
    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    } else {
    	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    }


    // OUR_JOB: Step 2.C, Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;

    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes
    attr_char_value.max_len     = p_our_value_size;
    attr_char_value.init_len    = p_our_value_size;
//    uint8_t value[p_our_value_size]            = {0};
    attr_char_value.p_value     = p_our_value;

//	NRF_LOG_DEBUG("p_our_value = %d", p_our_value);
//	NRF_LOG_DEBUG("*p_our_value = %d", *p_our_value);


    // OUR_JOB: Step 2.E, Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
                                       &char_md,
                                       &attr_char_value,
									   p_our_char_handles);
//    NRF_LOG_DEBUG("custom_uuid: 0x%x", custom_uuid);
    APP_ERROR_CHECK(err_code);

    printf("\r\nService handle: %#x\n\r", p_our_service->service_handle);
    printf("Char value handle: %#x\r\n", p_our_service->char_handles.value_handle);
    printf("Char cccd handle: %#x\r\n\r\n", p_our_service->char_handles.cccd_handle);

    return NRF_SUCCESS;
}


void custom_service_init(ble_custom_service_t * p_custom_service, uint16_t custom_service_uuid)
{
	// Reset everything to zero (especially needed for boolean default values)
    memset(&p_custom_service, 0, sizeof(p_custom_service));

    // STEP 3: Declare 16 bit service and 128 bit base UUIDs and add them to BLE stack table
	ble_uuid_t        service_uuid;
	ble_uuid128_t     base_uuid = BLE_UUID_SENSEN_BASE_UUID;
//	service_uuid.uuid = BLE_UUID_SENSEN_SERVICE;
	service_uuid.uuid = custom_service_uuid;
	err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
	APP_ERROR_CHECK(err_code);

    // OUR_JOB: Step 3.B, Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
	p_custom_service->conn_handle = BLE_CONN_HANDLE_INVALID;

    // STEP 4: Add our service
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
	                                    &service_uuid,
	                                    &p_custom_service->service_handle);
	APP_ERROR_CHECK(err_code);

    // OUR_JOB: Call the function our_char_add() to add our new characteristic to the service.
//    our_char_add(p_custom_service);

}

//void sensen_service_init(ble_custom_service_t * p_custom_service)
//{
//    // STEP 3: Declare 16 bit service and 128 bit base UUIDs and add them to BLE stack table
//	ble_uuid_t        service_uuid;
//	ble_uuid128_t     base_uuid = BLE_UUID_SENSEN_BASE_UUID;
//	service_uuid.uuid = BLE_UUID_SENSEN_SERVICE;
//	err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
//	APP_ERROR_CHECK(err_code);
//
//    // OUR_JOB: Step 3.B, Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
//	p_custom_service->conn_handle = BLE_CONN_HANDLE_INVALID;
//
//    // STEP 4: Add our service
//	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
//	                                    &service_uuid,
//	                                    &p_custom_service->service_handle);
//	APP_ERROR_CHECK(err_code);
//
//
//    // Print messages to Segger Real Time Terminal
//    // UNCOMMENT THE FOUR LINES BELOW AFTER INITIALIZING THE SERVICE OR THE EXAMPLE WILL NOT COMPILE.
//    NRF_LOG_DEBUG("Executing sensen_service_init()."); // Print message to RTT to the application flow
//    NRF_LOG_DEBUG("Service UUID: 0x%04x", service_uuid.uuid); // Print service UUID should match definition BLE_UUID_SENSEN_SERVICE
//    NRF_LOG_DEBUG("Service UUID type: 0x%02x", service_uuid.type); // Print UUID type. Should match BLE_UUID_TYPE_VENDOR_BEGIN. Search for BLE_UUID_TYPES in ble_types.h for more info
//    NRF_LOG_DEBUG("Service handle: 0x%04x", p_custom_service->service_handle); // Print out the service handle. Should match service handle shown in MCP under Attribute values
//
//    // OUR_JOB: Call the function our_char_add() to add our new characteristic to the service.
//    our_char_add(p_custom_service);
//
//}
//
//void SUM_service_init(ble_custom_service_t * p_custom_service)
//{
//    // Declare 16 bit service and 128 bit base UUIDs and add them to BLE stack table
//	ble_uuid_t        service_uuid;
//	ble_uuid128_t     base_uuid = BLE_UUID_SENSEN_BASE_UUID;
//	service_uuid.uuid = BLE_UUID_SUM_SERVICE;
//	err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
//	APP_ERROR_CHECK(err_code);
//
//    // OUR_JOB: Step 3.B, Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
//	p_custom_service->conn_handle = BLE_CONN_HANDLE_INVALID;
//
//    // Add our service
//	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
//	                                    &service_uuid,
//	                                    &p_custom_service->service_handle);
//	APP_ERROR_CHECK(err_code);
//}
//
//void HAP_service_init(ble_custom_service_t * p_custom_service)
//{
//    // Declare 16 bit service and 128 bit base UUIDs and add them to BLE stack table
//	ble_uuid_t        service_uuid;
//	ble_uuid128_t     base_uuid = BLE_UUID_SENSEN_BASE_UUID;
//	service_uuid.uuid = BLE_UUID_HAP_SERVICE;
//	err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
//	APP_ERROR_CHECK(err_code);
//
//    // OUR_JOB: Step 3.B, Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
//	p_custom_service->conn_handle = BLE_CONN_HANDLE_INVALID;
//
//    // Add our service
//	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
//	                                    &service_uuid,
//	                                    &p_custom_service->service_handle);
//	APP_ERROR_CHECK(err_code);
//}

// Sending out the temperature value BLE update
void our_temperature_characteristic_update(ble_custom_service_t *p_our_service, int32_t *temperature_value)
{
    // OUR_JOB: Step 3.E, Update characteristic value
    if (p_our_service->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint16_t               len = 4;
        ble_gatts_hvx_params_t hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_our_service->char_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &len;
        hvx_params.p_data = (uint8_t*)temperature_value;

        sd_ble_gatts_hvx(p_our_service->conn_handle, &hvx_params);
    }
}

// Sending out the temperature value BLE update
ret_code_t custom_characteristic_update(ble_custom_service_t *p_our_service, ble_gatts_char_handles_t * p_char_handles, void * p_value, uint16_t value_len)
{
//	NRF_LOG_DEBUG("In custom_characteristic_update(): p_char_handles: %d", p_char_handles);

    // OUR_JOB: Step 3.E, Update characteristic value
    if (p_our_service->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint16_t               len = value_len;
        ble_gatts_hvx_params_t hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_char_handles->value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &len;
        hvx_params.p_data = (uint8_t*)p_value;

        return sd_ble_gatts_hvx(p_our_service->conn_handle, &hvx_params);
    } else {
    	return NRF_ERROR_INVALID_STATE;
    }
}

// ALREADY_DONE_FOR_YOU: This is a timer event handler
//static void push_live_stream_values(void * p_context)
ret_code_t push_live_stream_values()
{
	NRF_LOG_DEBUG("In push_live_stream_values()");

//    // OUR_JOB: Step 3.F, Update temperature and characteristic value.
//    int32_t temperature = 0;    // Declare variable holding temperature value
//    static int32_t previous_temperature = 0; // Declare a variable to store current temperature until next measurement.
//
//    sd_temp_get(&temperature); // Get temperature
//
//    // Check if current temperature is different from last temperature
//    if(temperature != previous_temperature)
//    {
//        // If new temperature then send notification
//        our_temperature_characteristic_update(&m_SS_service, &temperature);
//    }
//
//    // Save current temperature until next measurement
//    previous_temperature = temperature;
//    nrf_gpio_pin_toggle(LED_4);


    // Plantower PM2_5
    if (product_service.pm2_5_handles.is_enabled) {
//		static int16_t previous_plantower_2_5_value = 0; // Declare a variable to store current temperature until next measurement.
		// Check if current value is different from last value
	    if(plantower_2_5_value != previous_plantower_2_5_value) {
			// If new value then send notification
			err_code = custom_characteristic_update(&product_service, &product_service.pm2_5_handles, &plantower_2_5_value, sizeof(plantower_2_5_value));
			if (err_code != NRF_SUCCESS) {
				NRF_LOG_WARNING("err_code: %d", err_code);
				return err_code;
			} else {
				previous_plantower_2_5_value = plantower_2_5_value;	// update previous value
			}
	    }
    }

    // Plantower PM10
    if (product_service.pm10_handles.is_enabled) {
//		static int16_t previous_plantower_10_value = 0; // Declare a variable to store current temperature until next measurement.
		// Check if current value is different from last value
	    if(plantower_10_value != previous_plantower_10_value) {
			// If new value then send notification
			err_code = custom_characteristic_update(&product_service, &product_service.pm10_handles, &plantower_10_value, sizeof(plantower_10_value));
			if (err_code != NRF_SUCCESS) {
				NRF_LOG_WARNING("err_code: %d", err_code);
				APP_ERROR_CHECK(err_code);
				return err_code;
			} else {
				previous_plantower_10_value = plantower_10_value;	// update previous value
			}
	    }
    }

    // Spec CO
    if (product_service.co_handles.is_enabled) {
//		static int16_t previous_specCO_value = 0; // Declare a variable to store current temperature until next measurement.
		// Check if current value is different from last value
	    if(specCO_value != previous_specCO_value) {
			// If new value then send notification
			err_code = custom_characteristic_update(&product_service, &product_service.co_handles, &specCO_value, sizeof(specCO_value));
			if (err_code != NRF_SUCCESS) {
				NRF_LOG_WARNING("err_code: %d", err_code);
				APP_ERROR_CHECK(err_code);
				return err_code;
			} else {
				previous_specCO_value = specCO_value;	// update previous value
			}
	    }
    }

    // Figaro CO2
    if (product_service.co2_handles.is_enabled) {
//		static int16_t previous_figCO2_value = 0; // Declare a variable to store current temperature until next measurement.
		// Check if current value is different from last value
	    if(figCO2_value != previous_figCO2_value) {
			// If new value then send notification
			err_code = custom_characteristic_update(&product_service, &product_service.co2_handles, &figCO2_value, sizeof(figCO2_value));
			if (err_code != NRF_SUCCESS) {
				NRF_LOG_WARNING("err_code: %d", err_code);
				APP_ERROR_CHECK(err_code);
				return err_code;
			} else {
				previous_figCO2_value = figCO2_value;	// update previous value
			}
	    }
    }

    // BME Humidity
    if (product_service.rh_handles.is_enabled) {
//		static int16_t previous_bme_humidity = 0; // Declare a variable to store current temperature until next measurement.
		// Check if current value is different from last value
	    if(bme_humidity != previous_bme_humidity) {
			// If new value then send notification
			err_code = custom_characteristic_update(&product_service, &product_service.rh_handles, &bme_humidity, sizeof(bme_humidity));
			if (err_code != NRF_SUCCESS) {
				NRF_LOG_WARNING("err_code: %d", err_code);
				APP_ERROR_CHECK(err_code);
				return err_code;
			} else {
				previous_bme_humidity = bme_humidity;	// update previous value
			}
	    }
    }

    // BME Temp
    if (product_service.temp_bme_handles.is_enabled) {
//		static int16_t previous_bme_temp_C = 0; // Declare a variable to store current temperature until next measurement.
		// Check if current value is different from last value
	    if(bme_temp_C != previous_bme_temp_C) {
			// If new value then send notification
			err_code = custom_characteristic_update(&product_service, &product_service.temp_bme_handles, &bme_temp_C, sizeof(bme_temp_C));
			if (err_code != NRF_SUCCESS) {
				NRF_LOG_WARNING("err_code: %d", err_code);
				APP_ERROR_CHECK(err_code);
				return err_code;
			} else {
				previous_bme_temp_C = bme_temp_C;	// update previous value
			}
	    }
    }

    // RTC Temp
    if (product_service.temp_rtc_handles.is_enabled) {
//		static int16_t previous_rtc_temp = 0; // Declare a variable to store current temperature until next measurement.

//		NRF_LOG_DEBUG("Before sending rtc_temp: %d", rtc_temp);
//		NRF_LOG_DEBUG("Before sending previous_rtc_temp: %d", previous_rtc_temp);

		// Check if current value is different from last value
	    if(rtc_temp != previous_rtc_temp) {

//			NRF_LOG_DEBUG("After comparing previous_rtc_temp: %d", previous_rtc_temp);

			// If new value then send notification
			err_code = custom_characteristic_update(&product_service, &product_service.temp_rtc_handles, &rtc_temp, sizeof(rtc_temp));
			if (err_code != NRF_SUCCESS) {
				NRF_LOG_WARNING("err_code: %d", err_code);
				APP_ERROR_CHECK(err_code);
				return err_code;
			} else {
				previous_rtc_temp = rtc_temp;	// update previous value
			}
	    }
    }

    // nRF Temp
    if (product_service.temp_nrf_handles.is_enabled) {
//		static int16_t previous_temp_nrf = 0; // Declare a variable to store current temperature until next measurement.
		// Check if current value is different from last value
	    if(temp_nrf != previous_temp_nrf) {
			// If new value then send notification
			err_code = custom_characteristic_update(&product_service, &product_service.temp_nrf_handles, &temp_nrf, sizeof(temp_nrf));
			if (err_code != NRF_SUCCESS) {
				NRF_LOG_WARNING("err_code: %d", err_code);
				APP_ERROR_CHECK(err_code);
				return err_code;
			} else {
				previous_temp_nrf = temp_nrf;	// update previous value
			}
	    }
    }

	// Ambient Light CH0
    if (product_service.ambient_CH0_handles.is_enabled) {
//		static int16_t previous_ambient_CH0 = 0; // Declare a variable to store current temperature until next measurement.
		// Check if current value is different from last value
	    if(ambient_CH0 != previous_ambient_CH0) {
			// If new value then send notification
			err_code = custom_characteristic_update(&product_service, &product_service.ambient_CH0_handles, &ambient_CH0, sizeof(ambient_CH0));
			if (err_code != NRF_SUCCESS) {
				NRF_LOG_WARNING("err_code: %d", err_code);
				APP_ERROR_CHECK(err_code);
				return err_code;
			} else {
				previous_ambient_CH0 = ambient_CH0;	// update previous value
			}
	    }
    }

	// Ambient Light CH1
    if (product_service.ambient_CH1_handles.is_enabled) {
//		static int16_t previous_ambient_CH1 = 0; // Declare a variable to store current temperature until next measurement.

//		NRF_LOG_DEBUG("Before sending ambient_CH1: %d", ambient_CH1);

		// Check if current value is different from last value
	    if(ambient_CH1 != previous_ambient_CH1) {

//			NRF_LOG_DEBUG("After comparing previous_ambient_CH1: %d", previous_ambient_CH1);

	    	// If new value then send notification
			err_code = custom_characteristic_update(&product_service, &product_service.ambient_CH1_handles, &ambient_CH1, sizeof(ambient_CH1));
			if (err_code != NRF_SUCCESS) {
				NRF_LOG_WARNING("err_code: %d", err_code);
				APP_ERROR_CHECK(err_code);
				return err_code;
			} else {
				previous_ambient_CH1 = ambient_CH1;	// update previous value
			}
	    }
    }

	// UVA
    if (product_service.uva_handles.is_enabled) {
//		static int16_t previous_UVA_value = 0; // Declare a variable to store current temperature until next measurement.

//		NRF_LOG_DEBUG("Before sending uva_value: %d", uva_value);

		// Check if current value is different from last value
	    if(uva_value != previous_UVA_value) {

//			NRF_LOG_DEBUG("After comparing previous_UVA_value: %d", previous_UVA_value);

			// If new value then send notification
			err_code = custom_characteristic_update(&product_service, &product_service.uva_handles, &uva_value, sizeof(uva_value));
			if (err_code != NRF_SUCCESS) {
				NRF_LOG_WARNING("err_code: %d", err_code);
				APP_ERROR_CHECK(err_code);
				return err_code;
			} else {
				previous_UVA_value = uva_value;	// update previous value
			}
	    }
    }



	NRF_LOG_DEBUG("OUT push_live_stream_values()");
	return NRF_SUCCESS;

}





//-------------------------------------------------------




/**
 * BLE functions from ble_app_uart
 */

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


void HardFault_Handler(void)
{
    uint32_t *sp = (uint32_t *) __get_MSP(); // Get stack pointer
    uint32_t ia = sp[12]; // Get instruction address from stack

//    printf("Hard Fault at address: 0x%08x\r\n", (unsigned int)ia);
    NRF_LOG_DEBUG("Hard Fault at address: 0x%08x\r\n", (unsigned int)ia);
    NRF_LOG_FLUSH();
    while(1)
        ;
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

// Uninitialize SD (closes file, unmounts, uninit.. does NOT change ADP1)
void sd_close() {
	ff_result = f_close(&file);
	if (ff_result) {
		NRF_LOG_WARNING("** WARNING: f_close() Failed, ff_result: %d", ff_result);
	}
}


// Uninitialize SD (closes file, unmounts, uninit.. does NOT change ADP1)
void sd_uninit() {

	NRF_LOG_DEBUG("sd_uninit()...");

//	ff_result = f_close(&file);
//	if (ff_result) {
//		NRF_LOG_WARNING("** WARNING: f_close() Failed, ff_result: %d", ff_result);
//	}
	ff_result = f_mount(0, "", 1);
	if (ff_result) {
		NRF_LOG_WARNING("** WARNING: UNmount Failed, ff_result: %d", ff_result);
	}
	DSTATUS disk_state = disk_uninitialize(0);
	if (disk_state != 1) {
		NRF_LOG_WARNING("** WARNING: Disk NOT properly uninitialized, disk_state: %d", disk_state);
	}

}


// Power on SD
void sd_power_on() {
	nrf_gpio_cfg_output(ADP1_PIN);
	nrf_gpio_pin_set(ADP1_PIN);		// Enable HIGH
}

// Power off SD
void sd_power_off() {
	nrf_gpio_cfg_output(ADP1_PIN);
	nrf_gpio_pin_clear(ADP1_PIN);		// Enable HIGH
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

	NRF_LOG_DEBUG("Initializing disk 0 (SDC)...");
	for (uint32_t retries = 3; retries && disk_state; --retries)
	{
		NRF_LOG_DEBUG("--BI");
		disk_state = disk_initialize(0);
		NRF_LOG_DEBUG("--AI");
	}
	if (disk_state)
	{
		NRF_LOG_ERROR("Disk initialization failed. disk_state: %d", disk_state);
	}

}


// Mounting SD
void sd_mount() {

    NRF_LOG_DEBUG("Mounting volume...");
    ff_result = f_mount(&fs, "", 1);
    if (ff_result) {
    	NRF_LOG_ERROR("Mount failed.");
    }
}

// Open SD
void sd_open(char fname[], BYTE mode) {

    // Open SD
//    NRF_LOG_DEBUG("Opening file " LOG_FILE_NAME "...");
    NRF_LOG_DEBUG("Opening file: %s", fname);
    ff_result = f_open(&file, fname, mode);
    if (ff_result != FR_OK) {
    	NRF_LOG_ERROR("Unable to open or create file: %s, %d", fname, ff_result);
    }

}


void sd_write_str(const void* buff) {

	uint32_t bytes_written;

	ff_result = f_write(&file, buff, strlen(buff), (UINT *) &bytes_written);
	if (ff_result != FR_OK)	{
		NRF_LOG_ERROR("** ERROR: Write failed, ff_result: %d **.", ff_result);
		// Reset the system if it's not working properly
		sd_write_failed = true;
		err_cnt++;
//		NVIC_SystemReset();

	}
	else {
		NRF_LOG_DEBUG("%d bytes written.", bytes_written);
//		// Update the SD card size
//		end_byte = f_size(&file);
//		NRF_LOG_DEBUG("end_byte: %d", end_byte);

	}
}


// Reads a large buffer block from the SDC
uint32_t read_SDC() {

	uint32_t bytes_read;

	ff_result = f_read(&file, sdc_buff, SDC_BUFF_SIZE, (UINT *) &bytes_read);
//	ff_result = f_read(&file, sdc_buff, 20, (UINT *) &bytes_read);
	if (ff_result != FR_OK)	{
		NRF_LOG_ERROR("** ERROR read_SDC(): read failed, ff_result: %d **", ff_result);
		return ff_result;
	}

	return bytes_read;
}


// Write a new Values file
FRESULT sd_values_update() {
	NRF_LOG_ERROR("In sd_values_update()");
	uint32_t bytes_written;

    sd_open(VALUES_FILE_NAME, FA_WRITE | FA_OPEN_ALWAYS);

//	f_lseek(&file, 0);
	ff_result = f_write(&file, &start_byte, sizeof(start_byte), (UINT *) &bytes_written);
//	ff_result = f_write(&file, &is_logging, sizeof(is_logging), (UINT *) &bytes_written);
	ff_result = f_write(&file, &on_logging, sizeof(on_logging), (UINT *) &bytes_written);
	ff_result = f_write(&file, &log_interval, sizeof(log_interval), (UINT *) &bytes_written);

#if PRODUCT_TYPE == HAP
	ff_result = f_write(&file, &plantower_startup_wait_ms,	sizeof(plantower_startup_wait_ms), (UINT *) &bytes_written);
	ff_result = f_write(&file, &specCO_startup_wait_ms, 	sizeof(specCO_startup_wait_ms), (UINT *) &bytes_written);
	ff_result = f_write(&file, &figaroCO2_startup_wait_ms, 	sizeof(figaroCO2_startup_wait_ms), (UINT *) &bytes_written);
#endif

    sd_close();

	return ff_result;
}

// Read a new Config file
FRESULT sd_values_read() {
	uint32_t bytes_read;

    sd_open(VALUES_FILE_NAME, FA_READ);

//	f_lseek(&file, 0);
	ff_result = f_read(&file, &start_byte, sizeof(start_byte), (UINT *) &bytes_read);
//	ff_result = f_read(&file, &is_logging, sizeof(is_logging), (UINT *) &bytes_read);
	ff_result = f_read(&file, &on_logging, sizeof(on_logging), (UINT *) &bytes_read);
	ff_result = f_read(&file, &log_interval, sizeof(log_interval), (UINT *) &bytes_read);

#if PRODUCT_TYPE == HAP
	ff_result = f_read(&file, &plantower_startup_wait_ms, 	sizeof(plantower_startup_wait_ms), (UINT *) &bytes_read);
	ff_result = f_read(&file, &specCO_startup_wait_ms, 		sizeof(specCO_startup_wait_ms), (UINT *) &bytes_read);
	ff_result = f_read(&file, &figaroCO2_startup_wait_ms, 	sizeof(figaroCO2_startup_wait_ms), (UINT *) &bytes_read);
#endif

    sd_close();

	return ff_result;
}

// Write a new Config file
FRESULT sd_info_create() {

//	// First write the data we have so far
//	sd_values_update();

    sd_open(INFO_FILE_NAME, FA_WRITE | FA_OPEN_ALWAYS);

	// Write extra stuff
    char out_str[MAX_OUT_STR_SIZE];
	int out_str_size = sprintf(out_str,
			"Product: %s\r\n"
//			"ble_gap_address = %X:%X:%X:%X:%X:%X\r\n"
			"ble_gap_address = %02X:%02X:%02X:%02X:%02X:%02X\r\n"
			"sensen_FW_version: %s\r\n"
			"product_FW_version: %s\r\n",
			PRODUCT_STR,
			ble_gap_address.addr[5], ble_gap_address.addr[4], ble_gap_address.addr[3], ble_gap_address.addr[2], ble_gap_address.addr[1], ble_gap_address.addr[0],
			sensen_FW_version,
			product_FW_version
			);
    if (out_str_size > MAX_OUT_STR_SIZE) {
    	NRF_LOG_ERROR("** ERROR: out_str too big!, out_str_size=%d", out_str_size);
    	err_cnt++;
    }
    sd_write_str(out_str);

    sd_close();

	return ff_result;
}



/**
 * Save all of the data
 */
void save_data(void) {

    // SD card TODO: add error code checking and err_cnt++
	NRF_LOG_DEBUG("");
    NRF_LOG_DEBUG("Testing SD Card...");
	NRF_LOG_DEBUG("------------------");
    sd_init();		// TODO: check that this doesn't need to be init with the other init's
    sd_mount();
//    sd_open();

//    // FOR TESTING
//    // Read parts of the file
//    sd_open(FA_READ | FA_WRITE);
//    read_SDC();
//    NRF_LOG_DEBUG("sdc_buff: %s", sdc_buff);
//    f_close(&file);


    sd_open(LOG_FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);

    // Write header to SD
    if (!header_is_written) {
    	sd_write_str(FILE_HEADER);
//    	header_is_written = 1;
    }

    char out_str[MAX_OUT_STR_SIZE];
    char extra_str[MAX_OUT_STR_SIZE];
//	NRF_LOG_DEBUG("sharpPM_value*adc_to_V/MBED_VREF: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(sharpPM_value*adc_to_V/MBED_VREF));
	NRF_LOG_FLUSH();
//    int out_str_size = sprintf(out_str, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%lu,%d,%ld,%d,%d,%lu,%lu,%lu,%lu\r\n",time_now,hpm_2_5_value,hpm_10_value,(int) (sharpPM_value*adc_to_V/MBED_VREF*1000),dht_temp_C,dht_humidity,(int) (specCO_value*adc_to_V/MBED_VREF*1000),(int) (figCO_value*adc_to_V/MBED_VREF*1000),figCO2_value, plantower_2_5_value, plantower_10_value, bme_temp_C, bme_humidity, bme_pressure, rtc_temp, temp_nrf, (int) (battery_value*adc_to_V*1000), fuel_v_cell, fuel_percent, err_cnt, dht_error_cnt_total, hpm_error_cnt_total);
//    int out_str_size = sprintf(out_str, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%lu,%d,%ld,%d,%d,%lu,%lu,%lu,%lu,%lu\r\n",time_now,hpm_2_5_value,hpm_10_value,(int) (sharpPM_value*1000*1000/V_to_adc_1000),dht_temp_C,dht_humidity,(int) (specCO_value*1000*1000/V_to_adc_1000),(int) (figCO_value*1000*1000/V_to_adc_1000),figCO2_value, plantower_2_5_value, plantower_10_value, bme_temp_C, bme_humidity, bme_pressure, rtc_temp, temp_nrf, (int) (battery_value*1000*1000/V_to_adc_1000), fuel_v_cell, fuel_percent, fuel_percent_raw, err_cnt, dht_error_cnt_total, hpm_error_cnt_total);

	// Save different strings depending on the device
	#if PRODUCT_TYPE == SUM
		int out_str_size = sprintf(out_str, "%ld,%d,%d,%d,%d,%d,%lu\r\n",time_now, rtc_temp, ambient_CH0, ambient_CH1, uva_value, fuel_v_cell, fuel_percent);
		int extra_str_size = sprintf(extra_str, "%ld,%ld,%d,%lu,%lu,%lu\r\n", time_now, temp_nrf, (int) (battery_value*1000*1000/V_to_adc_1000), fuel_percent, fuel_percent_raw, err_cnt);
	#elif PRODUCT_TYPE == HAP
		int out_str_size = sprintf(out_str, "%ld,%d,%d,%d,%d,%ld,%ld,%d,%lu\r\n",time_now, (int) (specCO_value*1000*1000/V_to_adc_1000), figCO2_value, plantower_2_5_value, plantower_10_value, bme_temp_C, bme_humidity, fuel_v_cell, fuel_percent);
		int extra_str_size = sprintf(extra_str, "%ld,%lu,%d,%ld,%d,%lu,%lu,%lu\r\n",time_now, bme_pressure, rtc_temp, temp_nrf, (int) (battery_value*1000*1000/V_to_adc_1000), fuel_percent, fuel_percent_raw, err_cnt);
	#else
//		int out_str_size = sprintf(out_str, "%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%lu,%d,%ld,%d,%d,%d,%d,%d,%lu,%lu,%lu,%lu,%lu,%ld,%lu,%lu,%lu\r\n",time_now,hpm_2_5_value,hpm_10_value,(int) (sharpPM_value*1000*1000/V_to_adc_1000),dht_temp_C,dht_humidity,(int) (specCO_value*1000*1000/V_to_adc_1000),(int) (figCO_value*1000*1000/V_to_adc_1000),figCO2_value, plantower_2_5_value, plantower_10_value, bme_temp_C, bme_humidity, bme_pressure, rtc_temp, temp_nrf, ambient_CH0, ambient_CH1, uva_value, (int) (battery_value*1000*1000/V_to_adc_1000), fuel_v_cell, fuel_percent, fuel_percent_raw, runtime_estimate, fuel_t0, fuel_p0, t0, err_cnt, dht_error_cnt_total, hpm_error_cnt_total);
//		int extra_str_size = sprintf(extra_str, "%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%lu,%d,%ld,%d,%d,%d,%d,%d,%lu,%lu,%lu,%lu,%lu,%ld,%lu,%lu,%lu\r\n",time_now,hpm_2_5_value,hpm_10_value,(int) (sharpPM_value*1000*1000/V_to_adc_1000),dht_temp_C,dht_humidity,(int) (specCO_value*1000*1000/V_to_adc_1000),(int) (figCO_value*1000*1000/V_to_adc_1000),figCO2_value, plantower_2_5_value, plantower_10_value, bme_temp_C, bme_humidity, bme_pressure, rtc_temp, temp_nrf, ambient_CH0, ambient_CH1, uva_value, (int) (battery_value*1000*1000/V_to_adc_1000), fuel_v_cell, fuel_percent, fuel_percent_raw, runtime_estimate, fuel_t0, fuel_p0, t0, err_cnt, dht_error_cnt_total, hpm_error_cnt_total);
		int out_str_size = sprintf(out_str, "%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%lu,%d,%ld,%d,%d,%d,%d,%d,%lu,%lu,%ld,%lu,%lu,%lu\r\n",time_now,hpm_2_5_value,hpm_10_value,(int) (sharpPM_value*1000*1000/V_to_adc_1000),dht_temp_C,dht_humidity,(int) (specCO_value*1000*1000/V_to_adc_1000),(int) (figCO_value*1000*1000/V_to_adc_1000),figCO2_value, plantower_2_5_value, plantower_10_value, bme_temp_C, bme_humidity, bme_pressure, rtc_temp, temp_nrf, ambient_CH0, ambient_CH1, uva_value, (int) (battery_value*1000*1000/V_to_adc_1000), fuel_v_cell, fuel_percent, fuel_percent_raw, t0, err_cnt, dht_error_cnt_total, hpm_error_cnt_total);
		int extra_str_size = sprintf(extra_str, "%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%lu,%d,%ld,%d,%d,%d,%d,%d,%lu,%lu,%ld,%lu,%lu,%lu\r\n",time_now,hpm_2_5_value,hpm_10_value,(int) (sharpPM_value*1000*1000/V_to_adc_1000),dht_temp_C,dht_humidity,(int) (specCO_value*1000*1000/V_to_adc_1000),(int) (figCO_value*1000*1000/V_to_adc_1000),figCO2_value, plantower_2_5_value, plantower_10_value, bme_temp_C, bme_humidity, bme_pressure, rtc_temp, temp_nrf, ambient_CH0, ambient_CH1, uva_value, (int) (battery_value*1000*1000/V_to_adc_1000), fuel_v_cell, fuel_percent, fuel_percent_raw, t0, err_cnt, dht_error_cnt_total, hpm_error_cnt_total);
	#endif

	NRF_LOG_DEBUG("out_str: %s", out_str);
    // Make sure buffer was big enough and didn't spill over
    if (out_str_size > MAX_OUT_STR_SIZE || extra_str_size > MAX_OUT_STR_SIZE) {
    	NRF_LOG_ERROR("** ERROR: output string too big!, out_str_size=%d, extra_str_size: %d", out_str_size, extra_str_size);
    	err_cnt++;
    }

    // Write the new data to file, update end_byte, close file
    sd_write_str(out_str);
	end_byte = f_size(&file);
    sd_close();


    // Now do the Extra log
    sd_open(EXTRA_LOG_FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    // Write header to SD
    if (!header_is_written) {
    	sd_write_str(FILE_HEADER_EXTRA);
    	header_is_written = 1;
    }
    sd_write_str(extra_str);






//    // FOR TESTING, REMOVE LATER
//	uint32_t bytes_written;
//	uint32_t bytes_read;
//	f_lseek(&file, 0);
////	sd_write_str("Testing write at beginning...");
//	start_byte = 2;
//	ff_result = f_write(&file, &start_byte, sizeof(start_byte), (UINT *) &bytes_written);
//	ff_result = f_write(&file, &is_logging, sizeof(is_logging), (UINT *) &bytes_written);
//	ff_result = f_write(&file, &log_interval, sizeof(log_interval), (UINT *) &bytes_written);
//
//	start_byte = 0;
//	is_logging = 0;
//	log_interval = 0;
//
//	f_lseek(&file, 0);
//	ff_result = f_read(&file, &start_byte, sizeof(start_byte), (UINT *) &bytes_read);
//	ff_result = f_read(&file, &is_logging, sizeof(is_logging), (UINT *) &bytes_read);
//	ff_result = f_read(&file, &log_interval, sizeof(log_interval), (UINT *) &bytes_read);
//
//	NRF_LOG_DEBUG("--AWRT");
//	NRF_LOG_DEBUG("start_byte: %d", start_byte);
//	NRF_LOG_DEBUG("is_logging: %d", is_logging);
//	NRF_LOG_DEBUG("log_interval: %d", log_interval);







    // Need to Uninit stuff.  O/w will not write on subsequent loops if ADP shuts off SDC; also drains power
//    (void) f_close(&file);
//    ff_result = f_mount(0, "", 1);
//    if (ff_result) {
//    	NRF_LOG_WARNING("** WARNING: UNmount Failed, ff_result: %d", ff_result);
//    }
//    DSTATUS disk_state = disk_uninitialize(0);
//    if (disk_state != 1) {
//    	NRF_LOG_WARNING("** WARNING: Disk NOT properly uninitialized, disk_state: %d", disk_state);
//    }
    sd_close();
	sd_uninit();

}


// Split up SDC into packets and keep sending
static void send_sdc_packets() {

//	NRF_LOG_DEBUG("Initial sdc_buff_current_pos: %d", sdc_buff_current_pos);
	int cnt = 0;
//	keep_sending_sdc_packets = true;

//	uint32_t bytes_remaining = 0;
	uint16_t send_length = packet_length;
//	resending_packets = false;
//	uint32_t sdc_bytes_read = 0;

	// Feed the Watchdog Timer before entering the loop
//    nrf_drv_wdt_channel_feed(wdt_meas_channel_id);
	nrf_drv_wdt_feed();	// use this instead for multiple wdt


    // Keep sending packets until buffers are full, then wait for completion
    while (true)
    {
    	// Read from SDC if starting from the beginning
//    	if (sdc_buff_current_pos == 0 && !done_reading_sdc) {
//		if (sdc_buff_current_pos == 0) {
		if (bytes_remaining == 0) {


//			sd_init();
//			sd_mount();
//	//		sd_open(ble_file_name, FA_READ | FA_WRITE);
//			sd_open(ble_file_name, FA_READ);

			sdc_read_num++;
			NRF_LOG_INFO("--READ SDC: sdc_read_num: %d", sdc_read_num);
//        	NRF_LOG_DEBUG("f_tell(&file): %d", f_tell(&file));
			bytes_remaining = read_SDC();
			sdc_buff_current_pos = 0;	// Reset to beginning
//			NRF_LOG_DEBUG("--AR");

//			app_sdc_info_t const * sdc_info;
//			sdc_info = app_sdc_info_get();
//        	NRF_LOG_DEBUG("sdc_info->num_blocks: %d", sdc_info->num_blocks);
//        	NRF_LOG_DEBUG("sdc_info->block_len: %d", sdc_info->block_len);

//			sdc_bytes_read = bytes_remaining;
        	NRF_LOG_DEBUG("bytes_remaining: %d", bytes_remaining);
    		if (bytes_remaining < SDC_BUFF_SIZE) {
            	NRF_LOG_INFO("done_reading_sdc: %d", done_reading_sdc);
    			done_reading_sdc = true;
    		}
    	}

		// If near end of file, and we have less than a packet
		if (bytes_remaining < packet_length) {
			send_length = bytes_remaining;
		} else {
			send_length = packet_length;
		}
//    	NRF_LOG_DEBUG("-sdc_read_num: %d", sdc_read_num);
//    	NRF_LOG_DEBUG("bytes_remaining: %d, packet_length: %d, send_length: %d", bytes_remaining, packet_length, send_length);
////    	nrf_delay_ms(500);

		// Sending the data over BLE
//		err_code = ble_nus_string_send(&m_nus, &sdc_buff[sdc_buff_current_pos], &send_length);
		err_code = custom_characteristic_update(&m_SS_service, &m_SS_service.data_offload_handles, &sdc_buff[sdc_buff_current_pos], send_length);


        if (err_code == NRF_ERROR_RESOURCES ||
            err_code == NRF_ERROR_INVALID_STATE ||
            err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        {
//        	NRF_LOG_DEBUG("send_sdc_packets(), err_code: %d", err_code);
//        	NRF_LOG_DEBUG("sdc_buff_current_pos: %d", sdc_buff_current_pos);
//        	NRF_LOG_DEBUG("cnt: %d", cnt);
////        	NRF_LOG_DEBUG("send_sdc_packets(), %d, err_code: %d", NRF_ERROR_RESOURCES, err_code);
            break;
        }
        else if (err_code != NRF_SUCCESS)
        {
        	NRF_LOG_ERROR("** ERROR send_sdc_packets(), err_code: %d", err_code);
            APP_ERROR_HANDLER(err_code);
        } else {
        	// If successfully sent, update position for next one
			sdc_buff_current_pos += send_length;
			bytes_remaining -= send_length;
			start_byte += send_length;
//			end_byte += send_length;
			cnt++;

			// Check if we read all the SDC and sent the remaining data
			if (done_reading_sdc && (bytes_remaining == 0) ) {
				done_sending_sdc = true;
				updating_values_file = true;
				sdc_read_num = 0;
				is_offloading = false;

				// Need to set the value with special function so App gets notified
//				uint32_t data_buffer = 0;
//				ble_gatts_value_t rx_data;
//				rx_data.len = sizeof(is_offloading);
//				rx_data.offset = 0;
//				rx_data.p_value = (uint8_t*)&is_offloading;
//				err_code = sd_ble_gatts_value_set(m_SS_service.conn_handle, m_SS_service.is_offloading_handles.value_handle, &rx_data);
//				APP_ERROR_CHECK(err_code);

//				for (int i=0; i < APP_PUSH_RETRY_NUM; i++) {	// Keep trying
//					err_code = custom_characteristic_update(&m_SS_service, &m_SS_service.is_offloading_handles, &is_offloading, sizeof(is_offloading));
//					if (err_code == NRF_SUCCESS) break;
//					nrf_delay_ms(APP_PUSH_RETRY_WAIT);
//				}
//				if (err_code) {
//					NRF_LOG_ERROR("After offloading: err_code: %d", err_code);
//					APP_ERROR_CHECK(err_code);
//					err_cnt++;
//				}
//
//				err_code = custom_characteristic_update(&m_SS_service, &m_SS_service.is_offloading_handles, &is_offloading, sizeof(is_offloading));
//				if (err_code != NRF_SUCCESS) {
//					NRF_LOG_ERROR("After offloading: err_code: %d", err_code);
//					APP_ERROR_CHECK(err_code);
//					err_cnt++;
//				}



//				start_sending_sdc_data = false;
				NRF_LOG_INFO("Final cnt: %d", cnt);
				NRF_LOG_INFO("Final send_length: %d", send_length);
				NRF_LOG_INFO("start_byte: %d", start_byte);
				NRF_LOG_INFO("end_byte: %d", end_byte);

	        	// Uninitialize here, since done with SDC
			    sd_close();
	        	sd_uninit();
	        	sd_power_off();
//	    		nrf_gpio_cfg_output(ADP1_PIN);
//	    		nrf_gpio_pin_clear(ADP1_PIN);		// Enable HIGH

	    		// Restart measurements when it's done
//	        	if (is_live_streaming) {
//	        		restart_measurements(LIVE_STREAM_LOG_INTERVAL);
//	        	} else {
//	        		restart_measurements(log_interval);
//	        	}
	        	restart_measurements();

				break;
			}

			// FOR TESTING: REMOVE LATER
//        	sd_uninit();
//        	nrf_delay_ms(1000);
//			break;

        }
    }


}


// Set up SD card and start sending data over BLE
static void send_sdc_data() {

	NRF_LOG_DEBUG("--ENTERED send_sdc_data()");

	// Stop measurements while sending data, and wait for current measurement to finish
	stop_measurements();
	NRF_LOG_INFO("Waiting for in_measuring_loop..");
	while (in_measuring_loop) {
//			nrf_delay_ms(1000);
	}

	// reset flag so that it doesn't try to send again every time it wakes up
	start_sending_sdc_data = false;
    done_reading_sdc = false;
	done_sending_sdc = false;


    // Read the SD card and send the data
    packet_length = BLE_TX_PACKET_SIZE;
//    done_reading_sdc = false;
//	uint8_t sdc_packet[packet_length];
//    uint8_t ble_sdc_buff[packet_length];


//	nrf_gpio_cfg_output(ADP1_PIN);
//	nrf_gpio_pin_set(ADP1_PIN);		// Enable HIGH
////	nrf_delay_ms(1000);
    sd_power_on();
	sd_init();
	sd_mount();
//		sd_open(ble_file_name, FA_READ | FA_WRITE);
	sd_open(ble_file_name, FA_READ);
	// Start where we left off last time
	f_lseek(&file, start_byte);

////	// Get the SD card size
//	end_byte = f_size(&file);
//	NRF_LOG_DEBUG("end_byte: %d", end_byte);




//	send_sdc_packets();
//	keep_sending_sdc_packets = true;

}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
//static void nus_data_handler(ble_nus_evt_t * p_evt)
//{
//
//    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
//    {
////        uint32_t err_code;
////        uint32_t err_code = NRF_SUCCESS;
//
//    	NRF_LOG_INFO("Received data from BLE NUS. Writing data on UART.");
//        NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
//
//
//        // FOR TESTING: trying to read a special symbol (ends in '*'), then send to App
//        char flag_send = '*';
//        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length-1] == flag_send) {
//        	NRF_LOG_INFO("Detected flag_send: %c, sending data", flag_send);
//
//            start_sending_sdc_data = true;
//        }
//
//
//    }
//
//    if (p_evt->type == BLE_NUS_EVT_TX_RDY) {
//////    	NRF_LOG_DEBUG("nus_data_handler(): Detected BLE_NUS_EVT_TX_RDY");
//////		if (resending_packets) {
////		if (!done_sending_sdc) {
////	    	NRF_LOG_DEBUG("nus_data_handler(): Detected BLE_NUS_EVT_TX_RDY");
//////			send_sdc_packets();
//////			keep_sending_sdc_packets = true;
//////				send_sdc_data();
////		}
//
//    }
//
//
//}
///**@snippet [Handling the data received over BLE] */




/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
//    uint32_t       err_code;
//    ble_nus_init_t nus_init;
//
//    memset(&nus_init, 0, sizeof(nus_init));
//
//    nus_init.data_handler = nus_data_handler;
//
//    err_code = ble_nus_init(&m_nus, &nus_init);
//    APP_ERROR_CHECK(err_code);

    // Init our custom services
//    sensen_service_init(&m_SS_service);
//    SUM_service_init(&m_SUM_service);
//    HAP_service_init(&m_HAP_service);

    // Sensen Service
    custom_service_init(&m_SS_service, BLE_UUID_SENSEN_SERVICE);
    custom_char_add(&m_SS_service, BLE_UUID_FW_VER_CHARACTERISTIC,			&m_SS_service.fw_ver_handles, 		(uint8_t *) &sensen_FW_version, 	sizeof(sensen_FW_version),		1,0,	0);
    custom_char_add(&m_SS_service, BLE_UUID_ON_LOGGING_CHARACTERISTIC,		&m_SS_service.on_logging_handles, 	(uint8_t *) &on_logging, 			sizeof(on_logging),				0,1,	0);
    custom_char_add(&m_SS_service, BLE_UUID_LOG_INTERVAL_CHARACTERISTIC,	&m_SS_service.log_rate_handles, 	(uint8_t *) &log_interval, 			sizeof(log_interval),			0,1,	0);
    custom_char_add(&m_SS_service, BLE_UUID_BATT_PERCENT_CHARACTERISTIC,	&m_SS_service.batt_percent_handles, (uint8_t *) &fuel_percent, 			sizeof(fuel_percent),			1,0,	0);
    custom_char_add(&m_SS_service, BLE_UUID_MIN_BATT_CHARACTERISTIC,		&m_SS_service.min_batt_handles, 	(uint8_t *) &min_battery_level, 	sizeof(min_battery_level),		0,1,	0);
    custom_char_add(&m_SS_service, BLE_UUID_RTC_CHARACTERISTIC,				&m_SS_service.rtc_handles, 			(uint8_t *) &time_to_be_set, 		sizeof(time_to_be_set),			0,1,	0);

    custom_char_add(&m_SS_service, BLE_UUID_DATA_OFFLOAD_CHARACTERISTIC,	&m_SS_service.data_offload_handles, (uint8_t *) sdc_buff, 				DATA_OFFLOAD_PACKET_SIZE,		0,0,	1);
    custom_char_add(&m_SS_service, BLE_UUID_START_BYTE_CHARACTERISTIC,		&m_SS_service.start_byte_handles, 	(uint8_t *) &start_byte, 			sizeof(start_byte),				1,0,	0);
    custom_char_add(&m_SS_service, BLE_UUID_END_BYTE_CHARACTERISTIC,		&m_SS_service.end_byte_handles, 	(uint8_t *) &end_byte, 				sizeof(end_byte),				1,0,	0);
    custom_char_add(&m_SS_service, BLE_UUID_IS_OFFLOADING_CHARACTERISTIC,	&m_SS_service.is_offloading_handles, (uint8_t *) &is_offloading, 		sizeof(is_offloading),			1,1,	1);

    custom_char_add(&m_SS_service, BLE_UUID_BATT_VOLTAGE_CHARACTERISTIC,	&m_SS_service.batt_voltage_handles, (uint8_t *) &fuel_v_cell, 			sizeof(fuel_v_cell),			1,0,	0);

//    our_char_add(&m_SS_service);

    // SUM Service
#if PRODUCT_TYPE == SUM	// Can't add all products because we run out of memory for Charx's
    custom_service_init(&product_service, BLE_UUID_SUM_SERVICE);
    custom_char_add(&product_service, BLE_UUID_FW_VER_CHARACTERISTIC,		&product_service.fw_ver_handles, 	(uint8_t *) &product_FW_version, 	sizeof(product_FW_version),		1,0,	0);
    custom_char_add(&product_service, BLE_UUID_TEMP_RTC_CHARACTERISTIC, 	&product_service.temp_rtc_handles, 	(uint8_t *) &rtc_temp, 				sizeof(rtc_temp),				1,0,	1);
    custom_char_add(&product_service, BLE_UUID_AMBIENT_CH0_CHARACTERISTIC, 	&product_service.ambient_CH0_handles, (uint8_t *) &ambient_CH0, 		sizeof(ambient_CH0),			1,0,	1);
    custom_char_add(&product_service, BLE_UUID_AMBIENT_CH1_CHARACTERISTIC, 	&product_service.ambient_CH1_handles, (uint8_t *) &ambient_CH1, 		sizeof(ambient_CH1),			1,0,	1);
    custom_char_add(&product_service, BLE_UUID_UVA_CHARACTERISTIC, 			&product_service.uva_handles, 		(uint8_t *) &uva_value, 			sizeof(uva_value),				1,0,	1);

    // HAP Service
#elif PRODUCT_TYPE == HAP
    custom_service_init(&product_service, BLE_UUID_HAP_SERVICE);
    custom_char_add(&product_service, BLE_UUID_FW_VER_CHARACTERISTIC,		&product_service.fw_ver_handles, 	(uint8_t *) &product_FW_version, 	sizeof(product_FW_version),		1,0,	0);
    custom_char_add(&product_service, BLE_UUID_PM2_5_CHARACTERISTIC, 		&product_service.pm2_5_handles, 	(uint8_t *) &plantower_2_5_value, 	sizeof(plantower_2_5_value),	1,0,	1);
    custom_char_add(&product_service, BLE_UUID_PM10_CHARACTERISTIC, 		&product_service.pm10_handles, 		(uint8_t *) &plantower_10_value, 	sizeof(plantower_10_value),		1,0,	1);
    custom_char_add(&product_service, BLE_UUID_CO_CHARACTERISTIC, 			&product_service.co_handles, 		(uint8_t *) &specCO_value, 			sizeof(specCO_value),			1,0,	1);
    custom_char_add(&product_service, BLE_UUID_CO2_CHARACTERISTIC, 			&product_service.co2_handles, 		(uint8_t *) &figCO2_value, 			sizeof(figCO2_value),			1,0,	1);
    custom_char_add(&product_service, BLE_UUID_RH_CHARACTERISTIC, 			&product_service.rh_handles, 		(uint8_t *) &bme_humidity, 			sizeof(bme_humidity),			1,0,	1);
    custom_char_add(&product_service, BLE_UUID_TEMP_BME_CHARACTERISTIC, 	&product_service.temp_bme_handles, 	(uint8_t *) &bme_temp_C, 			sizeof(bme_temp_C),				1,0,	1);
    custom_char_add(&product_service, BLE_UUID_PLANTOWER_WAIT_CHARACTERISTIC, 	&product_service.plantower_wait_handles, 	(uint8_t *) &plantower_startup_wait_ms, sizeof(plantower_startup_wait_ms),			1,1,	0);
    custom_char_add(&product_service, BLE_UUID_SPEC_CO_WAIT_CHARACTERISTIC, 	&product_service.specCO_wait_handles, 		(uint8_t *) &specCO_startup_wait_ms, 	sizeof(specCO_startup_wait_ms),				1,1,	0);
    custom_char_add(&product_service, BLE_UUID_FIGARO_CO2_WAIT_CHARACTERISTIC, 	&product_service.figaroCO2_wait_handles, 	(uint8_t *) &figaroCO2_startup_wait_ms, sizeof(figaroCO2_startup_wait_ms),			1,1,	0);

    // All Services if unspecified
#else
    custom_service_init(&product_service, BLE_UUID_CUSTOM_SERVICE);
    custom_char_add(&product_service, BLE_UUID_FW_VER_CHARACTERISTIC,		&product_service.fw_ver_handles, 	(uint8_t *) &product_FW_version, 	sizeof(product_FW_version),		1,0,	0);
    custom_char_add(&product_service, BLE_UUID_PM2_5_CHARACTERISTIC, 		&product_service.pm2_5_handles, 	(uint8_t *) &plantower_2_5_value, 	sizeof(plantower_2_5_value),	1,0,	1);
    custom_char_add(&product_service, BLE_UUID_PM10_CHARACTERISTIC, 		&product_service.pm10_handles, 		(uint8_t *) &plantower_10_value, 	sizeof(plantower_10_value),		1,0,	1);
    custom_char_add(&product_service, BLE_UUID_CO_CHARACTERISTIC, 			&product_service.co_handles, 		(uint8_t *) &specCO_value, 			sizeof(specCO_value),			1,0,	1);
    custom_char_add(&product_service, BLE_UUID_CO2_CHARACTERISTIC, 			&product_service.co2_handles, 		(uint8_t *) &figCO2_value, 			sizeof(figCO2_value),			1,0,	1);
    custom_char_add(&product_service, BLE_UUID_RH_CHARACTERISTIC, 			&product_service.rh_handles, 		(uint8_t *) &bme_humidity, 			sizeof(bme_humidity),			1,0,	1);
    custom_char_add(&product_service, BLE_UUID_TEMP_BME_CHARACTERISTIC, 	&product_service.temp_bme_handles, 	(uint8_t *) &bme_temp_C, 			sizeof(bme_temp_C),				1,0,	1);

    custom_char_add(&product_service, BLE_UUID_TEMP_RTC_CHARACTERISTIC, 	&product_service.temp_rtc_handles, 	(uint8_t *) &rtc_temp, 				sizeof(rtc_temp),				1,0,	1);
    custom_char_add(&product_service, BLE_UUID_AMBIENT_CH0_CHARACTERISTIC, 	&product_service.ambient_CH0_handles, (uint8_t *) &ambient_CH0, 		sizeof(ambient_CH0),			1,0,	1);
    custom_char_add(&product_service, BLE_UUID_AMBIENT_CH1_CHARACTERISTIC, 	&product_service.ambient_CH1_handles, (uint8_t *) &ambient_CH1, 		sizeof(ambient_CH1),			1,0,	1);
    custom_char_add(&product_service, BLE_UUID_UVA_CHARACTERISTIC, 			&product_service.uva_handles, 		(uint8_t *) &uva_value, 			sizeof(uva_value),				1,0,	1);

#endif

//    custom_char_add(&m_SS_service, BLE_UUID_FW_VER_CHARACTERISTIC,		&m_SS_service.fw_ver_handles, 	(uint8_t *) &sensen_FW_version, 	sizeof(sensen_FW_version),		1,0,0);
//    custom_char_add(&product_service, BLE_UUID_FW_VER_CHARACTERISTIC,		&product_service.fw_ver_handles, 	(uint8_t *) &product_FW_version, 		sizeof(product_FW_version),			1,0,0);
//    custom_char_add(&product_service, BLE_UUID_TEMP_RTC_CHARACTERISTIC, 	&product_service.temp_rtc_handles, 	(uint8_t *) &rtc_temp, 				sizeof(rtc_temp),				1,0,1);

}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
	// SHOULD ONLY BE HERE IF ADVERTISING TURNED OFF
	NRF_LOG_WARNING("** WARNING: In sleep_mode_enter(). SHOULD NOT BE HERE");

    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
		case BLE_ADV_EVT_FAST:
			err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
			APP_ERROR_CHECK(err_code);
			break;
		case BLE_ADV_EVT_SLOW:
			err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
			APP_ERROR_CHECK(err_code);
			break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


// Stop all of the sensor measurements
static void stop_measurements() {
	// Start the loop timer to trigger measurements
	NRF_LOG_DEBUG("In stop_measurements()");
	err_code = app_timer_stop(meas_loop_timer);
	if (err_code) {
		NRF_LOG_WARNING("** WARNING: %d, app_timer_stop(meas_loop_timer)", err_code);
	}
//	APP_ERROR_CHECK(err_code);
	is_logging = false;	// flag that we have stopped running
//	on_logging = false;	// flag that we have stopped running
//	using_live_stream_interval = false;
}


// Starts all of the sensor measurements
//static void restart_measurements(uint32_t temp_log_interval) {
static void restart_measurements() {
	// Stop the current timer
	stop_measurements();
	NRF_LOG_DEBUG("In restart_measurements()");

	if (on_logging) {	// only start again if turned on
		// Start the loop timer to trigger measurements
		if (is_live_streaming) {
			err_code = app_timer_start(meas_loop_timer, APP_TIMER_TICKS(LIVE_STREAM_LOG_INTERVAL), NULL);
			using_live_stream_interval = true;
			adjusting_timer_start = false;
		} else {
	//		// Wait before starting timer so that it will start at predictable times (e.g. 4:05, 4:10, 4:15, etc)
	//		err_code = read_rtc();
	//		if (err_code) {
	//			NRF_LOG_INFO("* RETRY: RTC ERROR, err_code=%d *", err_code);
	//			if (err_code) {
	//				NRF_LOG_ERROR("** ERROR: RTC read, err_code=%d **", err_code);
	//				time_now = 0;
	//				err_cnt++;
	//				rtc_error_cnt_total++;
	//			}
	//		}



	//		err_code = app_timer_start(meas_loop_timer, APP_TIMER_TICKS(temp_log_interval), NULL);
			err_code = app_timer_start(meas_loop_timer, APP_TIMER_TICKS(log_interval), NULL);
			using_live_stream_interval = false;
			adjusting_timer_start = true;
		}
		APP_ERROR_CHECK(err_code);
		is_logging = true;	// flag that we are running
	//	on_logging = true;	// flag that we are running
	//	test_all();	// Do a first measurement (otherwise we have to wait a full log_interval)
//		adjusting_timer_start = true;
		meas_loop_wait_done = true;
	} else {
		NRF_LOG_DEBUG("on_logging: %d, NOT restarted()", on_logging);
	}
}





// Event Handler for when GATT is written
static void on_ble_write(ble_custom_service_t * p_our_service, ble_evt_t const * p_ble_evt)
{
	// Declare buffer variable to hold received data. The data can only be 32 bit long.
	uint32_t data_buffer = 0;
	// Populate ble_gatts_value_t structure to hold received data and metadata.
	ble_gatts_value_t rx_data;
	rx_data.len = sizeof(uint32_t);
	rx_data.offset = 0;
	rx_data.p_value = (uint8_t*)&data_buffer;

//	static bool using_live_stream_interval = false;

//	NRF_LOG_DEBUG("p_our_service: %d", p_our_service);
//	NRF_LOG_DEBUG("&m_SS_service: %d", &m_SS_service);
//	NRF_LOG_DEBUG("&product_service: %d", &product_service);
//	NRF_LOG_DEBUG("&product_service: %d", &product_service);


	/**
	 * Sensen Services
	 */
	// For turning on Logging
	if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->on_logging_handles.value_handle) {

		NRF_LOG_INFO("on_logging: %d", on_logging);
		NRF_LOG_DEBUG("is_logging: %d", is_logging);
//		if (on_logging > 1) NRF_LOG_WARNING("on_logging > 1: %d", on_logging);

//		if (on_logging && !is_logging) {	// wants state change: turn on
////			restart_measurements(log_interval);
//			restart_measurements();
//			updating_values_file = true;
//		} else if (!on_logging && is_logging) {	// wants state change: turn off
//			stop_measurements();
//			updating_values_file = true;
//		}

		restart_measurements();
		updating_values_file = true;

		NRF_LOG_DEBUG("AS on_logging: %d", on_logging);
		NRF_LOG_DEBUG("AS is_logging: %d", is_logging);

	// Writing a new Log Interval
	} else if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->log_rate_handles.value_handle) {

		NRF_LOG_INFO("log_interval: %d", log_interval);

//		// Need to stop and restart the timer
//		if (is_logging) {
//			stop_measurements();
//		}
//		if (on_logging) {
////			restart_measurements(log_interval);	// Restarted with the updated log_interval value
//			restart_measurements();	// Restarted with the updated log_interval value
//			updating_values_file = true;
//		}

		restart_measurements();	// Restarted with the updated log_interval value
		updating_values_file = true;


	// Setting the RTC to a user defined time
	} else if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->rtc_handles.value_handle) {

		NRF_LOG_INFO("time_to_be_set: %d", time_to_be_set);
		setting_new_time = true;	// set flag, handle it later whe reading RTC


//		// FOR TESTING if this will work if called NOT from main context
//		NRF_LOG_INFO("setting_new_time: %d", time_to_be_set);
//
//	    struct tm * p_tm;
//	    p_tm = gmtime(&time_to_be_set);
//	    set_rtc((uint8_t) p_tm->tm_sec, (uint8_t) p_tm->tm_min, (uint8_t) p_tm->tm_hour, (uint8_t) p_tm->tm_wday + 1, (uint8_t) p_tm->tm_mday, (uint8_t) p_tm->tm_mon + 1, (uint8_t) p_tm->tm_year - 100);
//		NRF_LOG_INFO("DONE setting_new_time: %d", time_to_be_set);
//
//	    setting_new_time = false;	// reset flag


	// Data offload
	} else if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->data_offload_handles.cccd_handle) {
		// Get data
		err_code = sd_ble_gatts_value_get(p_our_service->conn_handle, p_our_service->data_offload_handles.cccd_handle, &rx_data);
		// Print handle and value
		if(data_buffer == 0x0001) {
			p_our_service->data_offload_handles.is_enabled = true;
			NRF_LOG_INFO("p_our_service->data_offload_handles.is_enabled: %d", p_our_service->data_offload_handles.is_enabled);

			// Do some checking before we start the offload process
			if (start_byte > end_byte) {
				start_byte = end_byte;
			}

//			// Make sure we aren't in the measuring loop, since SDC might already init/uninit
//			NRF_LOG_INFO("Waiting for in_measuring_loop..");
//			while (in_measuring_loop) {
//		//			nrf_delay_ms(1000);
//			}


//			updating_end_byte = true;


//			// Update the End Byte (file size) that will be sent
//		    sd_power_on();
//			sd_init();
//			sd_mount();
//			sd_open(ble_file_name, FA_READ);
////			// Start where we left off last time
////			f_lseek(&file, start_byte);
//			// Get the SD card size
//			end_byte = f_size(&file);
//			NRF_LOG_DEBUG("end_byte: %d", end_byte);
//
//        	// Uninitialize here, since don't want to interfere with another SDC operation
//        	sd_uninit();
//        	sd_power_off();


//			start_sending_sdc_data = true;

		} else if(data_buffer == 0x0000) {
			p_our_service->data_offload_handles.is_enabled = false;
			NRF_LOG_INFO("p_our_service->data_offload_handles.is_enabled: %d", p_our_service->data_offload_handles.is_enabled);

			done_sending_sdc = true;
		}

	// Is_Offloading, whether it is Transmitting data
	} else if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->is_offloading_handles.cccd_handle) {
		// Get data
		err_code = sd_ble_gatts_value_get(p_our_service->conn_handle, p_our_service->is_offloading_handles.cccd_handle, &rx_data);
		// Print handle and value
		if(data_buffer == 0x0001) {
			p_our_service->is_offloading_handles.is_enabled = true;
			NRF_LOG_INFO("p_our_service->is_offloading_handles.is_enabled: %d", p_our_service->is_offloading_handles.is_enabled);

			start_sending_sdc_data = true;

		} else if(data_buffer == 0x0000) {
			p_our_service->is_offloading_handles.is_enabled = false;
			NRF_LOG_INFO("p_our_service->is_offloading_handles.is_enabled: %d", p_our_service->is_offloading_handles.is_enabled);

			done_sending_sdc = true;
		}



	/**
	 * HAP Services for changing sensor wait times
	 */

		// Changing the Plantower startup wait time
		} else if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->plantower_wait_handles.value_handle) {
			NRF_LOG_INFO("plantower_startup_wait_ms: %d", plantower_startup_wait_ms);
	//		restart_measurements();	// Restarted with the updated log_interval value
			updating_values_file = true;

		// Changing the Spec CO startup wait time
		} else if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->specCO_wait_handles.value_handle) {
			NRF_LOG_INFO("specCO_startup_wait_ms: %d", specCO_startup_wait_ms);
	//		restart_measurements();	// Restarted with the updated log_interval value
			updating_values_file = true;

		// Changing the Figaro CO2 startup wait time
		} else if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->figaroCO2_wait_handles.value_handle) {
			NRF_LOG_INFO("figaroCO2_startup_wait_ms: %d", figaroCO2_startup_wait_ms);
	//		restart_measurements();	// Restarted with the updated log_interval value
			updating_values_file = true;



	/**
	 * Sensor Live Stream values using CCCD enables
	 */

	// Plantower PM2_5
	} else if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->pm2_5_handles.cccd_handle) {
		// Get data
		err_code = sd_ble_gatts_value_get(p_our_service->conn_handle, p_our_service->pm2_5_handles.cccd_handle, &rx_data);
		// Print handle and value
		if(data_buffer == 0x0001) {
			p_our_service->pm2_5_handles.is_enabled = true;
			NRF_LOG_INFO("p_our_service->pm2_5_handles.is_enabled: %d", p_our_service->pm2_5_handles.is_enabled);
		} else if(data_buffer == 0x0000) {
			p_our_service->pm2_5_handles.is_enabled = false;
			NRF_LOG_INFO("p_our_service->pm2_5_handles.is_enabled: %d", p_our_service->pm2_5_handles.is_enabled);
			}
	// Plantower PM10
	} else if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->pm10_handles.cccd_handle) {
		// Get data
		err_code = sd_ble_gatts_value_get(p_our_service->conn_handle, p_our_service->pm10_handles.cccd_handle, &rx_data);
		// Print handle and value
		if(data_buffer == 0x0001) {
			p_our_service->pm10_handles.is_enabled = true;
			NRF_LOG_INFO("p_our_service->pm10_handles.is_enabled: %d", p_our_service->pm10_handles.is_enabled);
		} else if(data_buffer == 0x0000) {
			p_our_service->pm10_handles.is_enabled = false;
			NRF_LOG_INFO("p_our_service->pm10_handles.is_enabled: %d", p_our_service->pm10_handles.is_enabled);
		}
	// Spec CO
	} else if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->co_handles.cccd_handle) {
		// Get data
		err_code = sd_ble_gatts_value_get(p_our_service->conn_handle, p_our_service->co_handles.cccd_handle, &rx_data);
		// Print handle and value
		if(data_buffer == 0x0001) {
			p_our_service->co_handles.is_enabled = true;
			NRF_LOG_INFO("p_our_service->co_handles.is_enabled: %d", p_our_service->co_handles.is_enabled);
		} else if(data_buffer == 0x0000) {
			p_our_service->co_handles.is_enabled = false;
			NRF_LOG_INFO("p_our_service->co_handles.is_enabled: %d", p_our_service->co_handles.is_enabled);
		}
	// Figaro CO2
	} else if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->co2_handles.cccd_handle) {
		// Get data
		err_code = sd_ble_gatts_value_get(p_our_service->conn_handle, p_our_service->co2_handles.cccd_handle, &rx_data);
		// Print handle and value
		if(data_buffer == 0x0001) {
			p_our_service->co2_handles.is_enabled = true;
			NRF_LOG_INFO("p_our_service->co2_handles.is_enabled: %d", p_our_service->co2_handles.is_enabled);
		} else if(data_buffer == 0x0000) {
			p_our_service->co2_handles.is_enabled = false;
			NRF_LOG_INFO("p_our_service->co2_handles.is_enabled: %d", p_our_service->co2_handles.is_enabled);
		}
	// BME Humidity
	} else if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->rh_handles.cccd_handle) {
		// Get data
		err_code = sd_ble_gatts_value_get(p_our_service->conn_handle, p_our_service->rh_handles.cccd_handle, &rx_data);
		// Print handle and value
		if(data_buffer == 0x0001) {
			p_our_service->rh_handles.is_enabled = true;
			NRF_LOG_INFO("p_our_service->rh_handles.is_enabled: %d", p_our_service->rh_handles.is_enabled);
		} else if(data_buffer == 0x0000) {
			p_our_service->rh_handles.is_enabled = false;
			NRF_LOG_INFO("p_our_service->rh_handles.is_enabled: %d", p_our_service->rh_handles.is_enabled);
		}
	// BME Temp
	} else if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->temp_bme_handles.cccd_handle) {
		// Get data
		err_code = sd_ble_gatts_value_get(p_our_service->conn_handle, p_our_service->temp_bme_handles.cccd_handle, &rx_data);
		// Print handle and value
		if(data_buffer == 0x0001) {
			p_our_service->temp_bme_handles.is_enabled = true;
			NRF_LOG_INFO("p_our_service->temp_bme_handles.is_enabled: %d", p_our_service->temp_bme_handles.is_enabled);
		} else if(data_buffer == 0x0000) {
			p_our_service->temp_bme_handles.is_enabled = false;
			NRF_LOG_INFO("p_our_service->temp_bme_handles.is_enabled: %d", p_our_service->temp_bme_handles.is_enabled);
		}
	// RTC Temp
	} else if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->temp_rtc_handles.cccd_handle) {
		// Get data
		err_code = sd_ble_gatts_value_get(p_our_service->conn_handle, p_our_service->temp_rtc_handles.cccd_handle, &rx_data);
		// Print handle and value
		if(data_buffer == 0x0001) {
			p_our_service->temp_rtc_handles.is_enabled = true;
			NRF_LOG_INFO("p_our_service->temp_rtc_handles.is_enabled: %d", p_our_service->temp_rtc_handles.is_enabled);
		} else if(data_buffer == 0x0000) {
			p_our_service->temp_rtc_handles.is_enabled = false;
			NRF_LOG_INFO("p_our_service->temp_rtc_handles.is_enabled: %d", p_our_service->temp_rtc_handles.is_enabled);
		}
	// nRF Temp
	} else if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->temp_nrf_handles.cccd_handle) {
		// Get data
		err_code = sd_ble_gatts_value_get(p_our_service->conn_handle, p_our_service->temp_nrf_handles.cccd_handle, &rx_data);
		// Print handle and value
		if(data_buffer == 0x0001) {
			p_our_service->temp_nrf_handles.is_enabled = true;
			NRF_LOG_INFO("p_our_service->temp_nrf_handles.is_enabled: %d", p_our_service->temp_nrf_handles.is_enabled);
		} else if(data_buffer == 0x0000) {
			p_our_service->temp_nrf_handles.is_enabled = false;
			NRF_LOG_INFO("p_our_service->temp_nrf_handles.is_enabled: %d", p_our_service->temp_nrf_handles.is_enabled);
		}
	// Ambient Light CH0
	} else if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->ambient_CH0_handles.cccd_handle) {
		// Get data
		err_code = sd_ble_gatts_value_get(p_our_service->conn_handle, p_our_service->ambient_CH0_handles.cccd_handle, &rx_data);
		// Print handle and value
		if(data_buffer == 0x0001) {
			p_our_service->ambient_CH0_handles.is_enabled = true;
			NRF_LOG_INFO("p_our_service->ambient_CH0_handles.is_enabled: %d", p_our_service->ambient_CH0_handles.is_enabled);
		} else if(data_buffer == 0x0000) {
			p_our_service->ambient_CH0_handles.is_enabled = false;
			NRF_LOG_INFO("p_our_service->ambient_CH0_handles.is_enabled: %d", p_our_service->ambient_CH0_handles.is_enabled);
		}
	// Ambient Light CH1
	} else if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->ambient_CH1_handles.cccd_handle) {
		// Get data
		err_code = sd_ble_gatts_value_get(p_our_service->conn_handle, p_our_service->ambient_CH1_handles.cccd_handle, &rx_data);
		// Print handle and value
		if(data_buffer == 0x0001) {
			p_our_service->ambient_CH1_handles.is_enabled = true;
			NRF_LOG_INFO("p_our_service->ambient_CH1_handles.is_enabled: %d", p_our_service->ambient_CH1_handles.is_enabled);
		} else if(data_buffer == 0x0000) {
			p_our_service->ambient_CH1_handles.is_enabled = false;
			NRF_LOG_INFO("p_our_service->ambient_CH1_handles.is_enabled: %d", p_our_service->ambient_CH1_handles.is_enabled);
		}
	// UVA
	} else if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->uva_handles.cccd_handle) {
		// Get data
		err_code = sd_ble_gatts_value_get(p_our_service->conn_handle, p_our_service->uva_handles.cccd_handle, &rx_data);
		// Print handle and value
		if(data_buffer == 0x0001) {
			p_our_service->uva_handles.is_enabled = true;
			NRF_LOG_INFO("p_our_service->uva_handles.is_enabled: %d", p_our_service->uva_handles.is_enabled);
		} else if(data_buffer == 0x0000) {
			p_our_service->uva_handles.is_enabled = false;
			NRF_LOG_INFO("p_our_service->uva_handles.is_enabled: %d", p_our_service->uva_handles.is_enabled);
		}

	}


	// Check if anything is enabled, don't do for the m_SS_service
	if (p_our_service == &product_service) {
	//	bool something_is_enabled = (
		is_live_streaming = (
				p_our_service->pm2_5_handles.is_enabled ||
				p_our_service->pm10_handles.is_enabled ||
				p_our_service->co_handles.is_enabled ||
				p_our_service->co2_handles.is_enabled ||
				p_our_service->rh_handles.is_enabled ||
				p_our_service->temp_bme_handles.is_enabled ||
				p_our_service->temp_rtc_handles.is_enabled ||
				p_our_service->temp_nrf_handles.is_enabled ||
				p_our_service->ambient_CH0_handles.is_enabled ||
				p_our_service->ambient_CH1_handles.is_enabled ||
				p_our_service->uva_handles.is_enabled //||
				);

		NRF_LOG_DEBUG("using_live_stream_interval: %d", using_live_stream_interval);
		NRF_LOG_DEBUG("is_live_streaming: %d", is_live_streaming);

		// Temporarily change the logging interval faster
		if (!using_live_stream_interval && is_live_streaming) {
	//		app_timer_start(m_our_char_timer_id, APP_TIMER_TICKS(LIVE_STREAM_LOG_INTERVAL), NULL);
	//		using_live_stream_interval = true;

			// Reset the temp variables before restarting livestream
			NRF_LOG_DEBUG("Resetting LiveStream values..");
			previous_plantower_2_5_value = 0; // Declare a variable to store current temperature until next measurement.
			previous_plantower_10_value = 0;
			previous_specCO_value = 0;
			previous_figCO2_value = 0;
			previous_bme_humidity = 0;
			previous_bme_temp_C = 0;
			previous_rtc_temp = 0;
			previous_temp_nrf = 0;
			previous_ambient_CH0 = 0;
			previous_ambient_CH1 = 0;
			previous_UVA_value = 0;

//			stop_measurements();
	//		restart_measurements(LIVE_STREAM_LOG_INTERVAL);	// NOTE: will auto-choose LIVE_STREAM_LOG_INTERVAL in restart_measurements()
			restart_measurements();	// NOTE: will auto-choose LIVE_STREAM_LOG_INTERVAL in restart_measurements()
	//		using_live_stream_interval = true;

		// Restore to original logging interval
		} else if (using_live_stream_interval && !is_live_streaming) {
	//		app_timer_stop(m_our_char_timer_id);
	//		using_live_stream_interval = false;

////			stop_measurements();
//	//		using_live_stream_interval = false;
//			// Only restart it if logging is turned on
//			if (on_logging) {
//	//			restart_measurements(log_interval);
//				restart_measurements();
//			}

			restart_measurements();

		}
	}


}

// Event Handler for custom sensen service
void ble_custom_service_ble_evt_handler(ble_custom_service_t * p_our_service, ble_evt_t const * p_ble_evt)
{
    // OUR_JOB: Step 3.D Implement switch case handling BLE events related to our service.
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            on_ble_write(p_our_service, p_ble_evt);
            break;
        case BLE_GAP_EVT_CONNECTED:
            p_our_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            // When connected; start our timer to start regular temperature measurements
//            app_timer_start(m_our_char_timer_id, APP_TIMER_TICKS(LIVE_STREAM_UPDATE_INTERVAL), NULL);

            break;
        case BLE_GAP_EVT_DISCONNECTED:
            p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;

            // When disconnected; stop our timer to stop temperature measurements
//            app_timer_stop(m_our_char_timer_id);

            break;
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
//		case BLE_GATTS_EVT_HVN_TX_COMPLETE :
////			NRF_LOG_DEBUG("ble_evt_handler(): BLE_GATTS_EVT_HVN_TX_COMPLETE");
////			NRF_LOG_DEBUG("resending_packets: %d, done_reading_sdc: %d", resending_packets, done_reading_sdc);
////			if (resending_packets) {
//			if (!done_sending_sdc) {
//				NRF_LOG_DEBUG("ble_evt_handler(): BLE_GATTS_EVT_HVN_TX_COMPLETE");
//				send_sdc_packets();
////				send_sdc_data();
//			}
//			break;

        case BLE_GAP_EVT_CONNECTED:
        	NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
        	NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            is_live_streaming = false;
            if (using_live_stream_interval) {
    			restart_measurements();
            }
            break;

#ifndef S140
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
#endif

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;
#if !defined (S112)
         case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
        {
            ble_gap_data_length_params_t dl_params;

            // Clearing the struct will effectivly set members to @ref BLE_GAP_DATA_LENGTH_AUTO
            memset(&dl_params, 0, sizeof(ble_gap_data_length_params_t));
            err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &dl_params, NULL);
            APP_ERROR_CHECK(err_code);
        } break;
#endif //!defined (S112)
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
        	NRF_LOG_DEBUG("** TIMEOUT: BLE_GATTC_EVT_TIMEOUT in ble_evt_handler()");
        	NRF_LOG_FLUSH();
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
        	NRF_LOG_DEBUG("** TIMEOUT: BLE_GATTS_EVT_TIMEOUT in ble_evt_handler()");
        	NRF_LOG_FLUSH();
        	err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        default:
            // No implementation needed.
            break;
    }


    // Also handle events for our custom services
    ble_custom_service_ble_evt_handler(&m_SS_service, p_ble_evt);
    ble_custom_service_ble_evt_handler(&product_service, p_ble_evt);
//    ble_custom_service_ble_evt_handler(&product_service, p_ble_evt);
//    ble_custom_service_ble_evt_handler(&product_service, p_ble_evt);

//    ble_SUM_service_ble_evt_handler(&product_service, p_ble_evt);
//    ble_HAP_service_ble_evt_handler(&product_service, p_ble_evt);
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_DEBUG("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, 64);
//    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, 23);	// other values (16, 19, etc) didn't work
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 * NOTE: max of 31 bytes
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

//    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.name_type          = BLE_ADVDATA_SHORT_NAME;
    init.advdata.short_name_len = 4;
    init.advdata.include_appearance = false;
//    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;	// needed to disable timeout
//    init.advdata.p_tx_power_level	= BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;	// needed to disable timeout

    // Scan response: send the UUID
//    ble_uuid_t m_adv_uuids[] =  {
//    		{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE},			// ble_nus
////    		{BLE_UUID_SENSEN_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN},	// sensen_service
////    		{BLE_UUID_SUM_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN},		// SUM_service
////    		{BLE_UUID_HAP_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN},		// HAP_service
//    };
//    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
//    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

//    broadcast_data.err_cnt_total = 	0x11111119;
//    broadcast_data.is_logging = 	is_logging;
//	broadcast_data.fuel_percent = 	0xFFFFFFF1;

    ble_advdata_service_data_t sr_service_data;
    memset(&sr_service_data, 0, sizeof(sr_service_data));
    sr_service_data.service_uuid = BLE_UUID_SENSEN_SERVICE;
//    sr_service_data.data.p_data = broadcast_data;
//    sr_service_data.data.size = sizeof(err_cnt_total) + sizeof(is_logging) + sizeof(fuel_percent);
    sr_service_data.data.p_data = (uint8_t *) &broadcast_data;
    sr_service_data.data.size = sizeof(broadcast_data);
    init.srdata.p_service_data_array = &sr_service_data;
    init.srdata.service_data_count++;


    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
//    NRF_LOG_DEBUG("APP_ADV_INTERVAL: %d", APP_ADV_INTERVAL);
    init.config.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

//    init.config.ble_adv_slow_enabled  = true;
//    init.config.ble_adv_slow_interval = APP_ADV_INTERVAL;
//    init.config.ble_adv_slow_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    init.evt_handler = on_adv_evt;

//    broadcast_data.err_cnt_total = 	0x11111119;
//    broadcast_data.is_logging = 	0xFFFFFFFA;
//    broadcast_data.err_cnt_total = 	0x22222228;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

//    broadcast_data.err_cnt_total = 	0x33333337;


    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

//    broadcast_data.err_cnt_total = 	0x44444446;

}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

//    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
//    uint32_t err_code = bsp_init(BSP_INIT_BUTTONS, bsp_event_handler);
    uint32_t err_code = bsp_init(0, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
	// Needed to prevent FPU from keeping MPU awake
	#if (__FPU_USED == 1)
	 __set_FPSCR(__get_FPSCR() & ~(0x0000009F));
	 (void) __get_FPSCR();
	 NVIC_ClearPendingIRQ(FPU_IRQn);
	#endif

//     NRF_LOG_DEBUG("Entering sd_app_evt_wait()");
    err_code = sd_app_evt_wait();
    if (err_code) {
        NRF_LOG_DEBUG("power_manage(), err_code: %d", err_code);
    }
    APP_ERROR_CHECK(err_code);
}

//------------------------------------------------------------------------------------------------------


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


// Watchdog timer setup
void wdt_init(void) {

    // Setup Watchdog Timer
	NRF_LOG_DEBUG("Start Watchdog Timer...");
	NRF_LOG_DEBUG("WDT_TIMEOUT_MEAS: %d", WDT_TIMEOUT_MEAS);
//	NRF_LOG_DEBUG("WDT_TIMEOUT_SLEEP: %d", WDT_TIMEOUT_SLEEP);
//	if (!nrf_drv_clock_init_check() ) {
//		err_code = nrf_drv_clock_init();
//		NRF_LOG_DEBUG("nrf_drv_clock_init() err_code: %d", err_code);
//		APP_ERROR_CHECK(err_code);
//	}
//    nrf_drv_clock_lfclk_request(NULL);
//    err_code = app_timer_init();	// CAREFULL, MAYBE DON'T NEED THIS SINCE INIT ALREADY
//	NRF_LOG_DEBUG("app_timer_init() err_code: %d", err_code);
//    APP_ERROR_CHECK(err_code);

    // WDT for within a measurement loop
    // Default values: Pause in SLEEP, Pause in HALT; 2000 ms; IRQ 7
    nrf_drv_wdt_config_t wdt_meas_config = NRF_DRV_WDT_DEAFULT_CONFIG;
    wdt_meas_config.reload_value = WDT_TIMEOUT_MEAS;
    err_code = nrf_drv_wdt_init(&wdt_meas_config, wdt_event_handler);
//	NRF_LOG_DEBUG("wdt_init() err_code: %d", err_code);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&wdt_meas_channel_id);
//	NRF_LOG_DEBUG("wdt_init() err_code: %d", err_code);
    APP_ERROR_CHECK(err_code);

//    // WDT for long sleep cycles
//    // Default values: Pause in SLEEP, Pause in HALT; 2000 ms; IRQ 7
//    nrf_drv_wdt_config_t wdt_sleep_config = NRF_DRV_WDT_DEAFULT_CONFIG;
//    wdt_sleep_config.reload_value = WDT_TIMEOUT_SLEEP;
//    wdt_sleep_config.behaviour = NRF_WDT_BEHAVIOUR_RUN_SLEEP;	// keep it running while waiting
//    err_code = nrf_drv_wdt_init(&wdt_sleep_config, wdt_event_handler);
//	NRF_LOG_DEBUG("wdt_init() err_code: %d", err_code);
//    APP_ERROR_CHECK(err_code);
//    err_code = nrf_drv_wdt_channel_alloc(&wdt_sleep_channel_id);
//	NRF_LOG_DEBUG("wdt_init() err_code: %d", err_code);
//    APP_ERROR_CHECK(err_code);

    nrf_drv_wdt_enable();

}



/////** MACROS needed for app_uart **/
////// Taken from uart example
//void uart_error_handle(app_uart_evt_t * p_event)
//{
//    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
//    {
//        APP_ERROR_HANDLER(p_event->data.error_communication);
//    }
//    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
//    {
//        APP_ERROR_HANDLER(p_event->data.error_code);
//    }
//}


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
	NRF_LOG_DEBUG("--1A");

    // Sending the cmd
    err_code = nrf_serial_write(&serial_uart, cmd, HPM_CMD_LEN, &serial_bytes_written, HPM_SERIAL_TIMEOUT);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEBUG("serial_bytes_written: %d", serial_bytes_written);


//    // Check if the HPM acknowledged
    char hpm_ack_rx[2] = {0x0, 0x0 };
	size_t serial_bytes_read;
    err_code = nrf_serial_read(&serial_uart, hpm_ack_rx, sizeof(hpm_ack_rx), &serial_bytes_read, HPM_SERIAL_TIMEOUT);
    NRF_LOG_DEBUG("hpm_ack_rx: 0x%x, 0x%x", hpm_ack_rx[0], hpm_ack_rx[1]);
    NRF_LOG_DEBUG("err_code: %d", err_code);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEBUG("hpm_ack_rx: 0x%x, 0x%x", hpm_ack_rx[0], hpm_ack_rx[1]);

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
		NRF_LOG_DEBUG("err_code: %d", hpm_read_meas_cmd);
	    APP_ERROR_CHECK(err_code);

		// Sending the read_meas command
		NRF_LOG_DEBUG("hpm_read_meas_cmd: 0x%x", hpm_read_meas_cmd);
		err_code = nrf_serial_write(&serial_uart, hpm_read_meas_cmd, HPM_CMD_LEN, &serial_bytes_written, HPM_SERIAL_TIMEOUT);
		APP_ERROR_CHECK(err_code);

	    for(int i = 0; i < HPM_BUFF_SIZE; i++) {

	        NRF_LOG_DEBUG("i: %d", i);
			err_code = nrf_serial_read(&serial_uart, &hpm_buff[i], sizeof(char), &serial_bytes_read, HPM_SERIAL_TIMEOUT);
	        NRF_LOG_DEBUG("err_code: %d", err_code);
			APP_ERROR_CHECK(err_code);
	        NRF_LOG_DEBUG("0x%x", hpm_buff[i]);
	    }
		APP_ERROR_CHECK(err_code);

		// calc values
		hpm_2_5_value = 256*hpm_buff[3] + hpm_buff[4];
		hpm_10_value = 256*hpm_buff[5] + hpm_buff[6];
		NRF_LOG_DEBUG("hpm_2_5_value: %d", hpm_2_5_value);
		NRF_LOG_DEBUG("hpm_10_value: %d", hpm_10_value);

		// calc checksum
		int checksum_calc = 0;
		for(int i = 0; i < HPM_BUFF_SIZE - 1; i++) {	// everything except last one (checksum value)
			checksum_calc += hpm_buff[i];
		}
		checksum_calc = (65536 - checksum_calc) % 256;
	    NRF_LOG_DEBUG("checksum_calc: 0x%x", checksum_calc);

		// TODO: make this return a proper success code
		if (checksum_calc == hpm_buff[HPM_BUFF_SIZE - 1]) {
			return 0;
		} else {
			NRF_LOG_DEBUG("HPM checksum ERROR!");
			return 1;
		}
    }

    // Reading using HPM's Autosend
    else {
		NRF_LOG_DEBUG("USING AUTOSEND");

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
				NRF_LOG_DEBUG("HAVEN'T FOUND HEADER YET..");

			}

		}

		// If we found the header, read the rest
		if (found_header) {
			NRF_LOG_DEBUG("FOUND HEADER..");

			err_code = nrf_serial_read(&serial_uart, &hpm_buff[2], HPM_BUFF_SIZE - 2, &serial_bytes_read, HPM_SERIAL_TIMEOUT);
			// If it timed out, return and handle it there TODO: make this more robust
			NRF_LOG_DEBUG("err_code: %d", err_code);
			if (err_code == NRF_ERROR_TIMEOUT) return err_code;
			APP_ERROR_CHECK(err_code);

			// calc values
			hpm_2_5_value = 256*hpm_buff[6] + hpm_buff[7];
			hpm_10_value = 256*hpm_buff[8] + hpm_buff[9];
			NRF_LOG_DEBUG("hpm_2_5_value: %d", hpm_2_5_value);
			NRF_LOG_DEBUG("hpm_10_value: %d", hpm_10_value);

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
				NRF_LOG_ERROR("** ERROR: HPM checksum ERROR! **");
				return 2;
			}
		} else {
			NRF_LOG_ERROR("** ERROR: HPM never found Header! **");
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
            NRF_LOG_DEBUG("saadc_callback (should NOT be here): %d", p_event->data.done.p_buffer[i]);
        }

    }
}

// Taken from example_code/saadc_simpler
// TODO:  init and uninit each channel as you use them.
void saadc_init(void)
{
	// Calculate adc_to_V (couldn't do this up top since C is dumb)
	adc_to_V = 1.0f / ((ADC_GAIN_VALUE / ADC_REFERENCE_VOLTAGE) * (pow(2, ADC_RESOLUTION_BITS)-1) );
	adc_to_mV = 1000 * 1.0f / ((ADC_GAIN_VALUE / ADC_REFERENCE_VOLTAGE) * (pow(2, ADC_RESOLUTION_BITS)-1) );
	V_to_adc_1000 = 1000*(ADC_GAIN_VALUE / ADC_REFERENCE_VOLTAGE) * (pow(2, ADC_RESOLUTION_BITS)-1);
	NRF_LOG_DEBUG("V_to_adc_1000: %d", V_to_adc_1000);
//	NRF_LOG_DEBUG("adc_to_V: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(adc_to_V));

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
//       .clear_bus_init     = true,
//	   .hold_bus_uninit		= true
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

//    nrf_drv_twi_disable(&m_twi);	// for saving power


}

// 2's Complement
int16_t twosComp( uint8_t bit_msb, uint8_t bit_lsb) {
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

	// Check that we are not over 23:59
	if (hour > 23) {
		NRF_LOG_WARNING("** WARNING: hour too big, adjusting to a lower value: %d", hour);
		hour = 23;
	}

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
    NRF_LOG_DEBUG("is_running: %d", is_running );

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
    time_now = mktime(&t);

//    NRF_LOG_DEBUG("sizeof(time_now): %d", sizeof(time_now));

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


//// Quick Start Fuel Gauge MAX17043 with TWI (I2C)
//// This lets you pick when the "initial guess" of the internal SOC algorithm takes place
//// Make sure this happens when there aren't initial turn-on fluctuations.
//static ret_code_t fuel_gauge_quick_start() {
//
//	uint8_t regt = 0x06;	// MODE register
//	uint8_t cmd[] = {regt, 0x40, 0x00};	// write 0x4000 to quick start
//
//    nrf_drv_twi_enable(&m_twi);		// for saving power
//
//    // Get battery voltage
//	NRF_LOG_DEBUG("--Q1");
//    err_code = nrf_drv_twi_tx(&m_twi, fuel_addr, cmd, 3, false);
//    if (err_code) {	// handle error outside
//    	NRF_LOG_DEBUG("** WARNING in fuel_gauge_quick_start(), err_code: %d **", err_code);
//    	return err_code;
//    }
////    APP_ERROR_CHECK(err_code);
//
//    nrf_drv_twi_disable(&m_twi);		// for saving power
//
//    return err_code;
//
//}


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
    	NRF_LOG_WARNING("** WARNING in fuel_gauge_sleep(), err_code: %d **", err_code);
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
    	NRF_LOG_WARNING("** WARNING in fuel_gauge_wake(), err_code: %d **", err_code);
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
    	NRF_LOG_WARNING("** WARNING in read_fuel_gauge(), err_code: %d **", err_code);
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
    fuel_percent_raw = (cmd[0] << 8) | cmd[1];	// combine 2 bytes
//	NRF_LOG_DEBUG("cmd[0] = 0x%x", cmd[0]);
//	NRF_LOG_DEBUG("cmd[1] = 0x%x", cmd[1]);
//	NRF_LOG_DEBUG("fuel_percent_raw = %d", fuel_percent_raw);
	fuel_percent_raw = fuel_percent_raw/256.0 * 1000;	// convert to float, but leave 3 decimal places
////    fuel_percent_raw = fuel_percent_raw * (1000.0/256.0);	// convert to float, but leave 3 decimal places
//	NRF_LOG_DEBUG("fuel_percent_raw = %d", fuel_percent_raw);
//	NRF_LOG_DEBUG("cmd[0]*1000 + cmd[1]*(1000.0/256.0) = %d", cmd[0]*1000 + cmd[1]*(1000.0/256.0) );
//	fuel_percent_raw = cmd[0]*1000 + cmd[1]*(1000.0/256.0);	// Need *1000 for 3 decimal places, /256 to get fractional part


    nrf_drv_twi_disable(&m_twi);		// for saving power

    return err_code;

}


// Adjust or Extrapolate battery fuel percentage for our custom batteries
static void calc_fuel_percent() {



	// Only need to correct for non-standard battery
	if (battery_type_used != BAT_LIPO_2000mAh) {

		//	float FUEL_SCALE_FACTOR = 2.589;
//		float battery_scale_factor = battery_scale_factors[battery_type_used];
		float m = m_batt[battery_type_used];
		float b = b_batt[battery_type_used];
//		NRF_LOG_DEBUG("battery_scale_factor*1000: %d", battery_scale_factor*1000);

		// Use empirical linear fit
		fuel_percent = 100*1000 * (m*fuel_v_cell + b);



//		// Store initial values for later
//		if (t0 == 0) {
//			t0 = time_now;
//		}
//		if (fuel_p0 == 0) {
////			fuel_t0 = time_now;
//			fuel_p0 = fuel_percent_raw;
//
//////			t0 = time_now - 3600*10;
////			t0 = time_now - 3600*5;
//////			fuel_t0 = (time_now - 3600*10);
//////			fuel_p0 = 100*1000;
////			fuel_p0 = 60*1000;
//		}
//
//
////		// FOR TESTING
////		if (loop_num > 0) fuel_percent_raw = 80*1000;
////		if (loop_num > 1) fuel_percent_raw = 60*1000;
////		if (loop_num > 2) fuel_percent_raw = 40*1000;
////		if (loop_num > 3) fuel_percent_raw = 10*1000;
////		NRF_LOG_DEBUG("fuel_percent_raw: %d", fuel_percent_raw);
//
//
//
//		// If above the threshold, adjust the measured value differently
//		NRF_LOG_DEBUG("fuel_percent_raw: %d", fuel_percent_raw);
//		NRF_LOG_DEBUG("FUEL_PERCENT_THRESHOLD*1000: %d", FUEL_PERCENT_THRESHOLD*1000);
//		if (fuel_percent_raw > FUEL_PERCENT_THRESHOLD*1000) {
//			// Simple attempt: weighted averages of current and initial
//			fuel_percent = fuel_percent_raw/battery_scale_factor + (battery_scale_factor-1)*fuel_p0/battery_scale_factor;
//	//	} else {	// More complicated: extrapolate along a line: (y-y0)=m(x-x0), m=(yL-y0)/(xL-x0), xL=ath*xth
//		} else {	// More complicated: Estimate total runtime and take percentage of that
//			if (runtime_estimate == 0) {	// estimate runtime only the first time it falls below threshold
//				runtime_estimate = (time_now - t0) * battery_scale_factor * ((1.0f*(100-FUEL_PERCENT_THRESHOLD)*1000) / (1.0f*fuel_p0-FUEL_PERCENT_THRESHOLD*1000));	// Later part corrects for not starting at 100% battery
////				fuel_t0 = t0 - runtime_estimate * ((100*1000 - fuel_percent_raw) / (1.0f*100*1000));
//				fuel_t0 = t0 - runtime_estimate * ((100*1000 - fuel_p0) / (1.0f*100*1000));
//				NRF_LOG_DEBUG("previous fuel_t0: %d", fuel_t0);
//
//				uint32_t max_fuel_percent;
//				if (fuel_p0 > 100*1000) max_fuel_percent = fuel_p0;
//				else max_fuel_percent = 100*1000;
//				fuel_t0 = t0 - runtime_estimate * ((max_fuel_percent - fuel_p0) / (1.0f*max_fuel_percent));
//				NRF_LOG_DEBUG("modified fuel_t0: %d", fuel_t0);
//				NRF_LOG_DEBUG("fuel_p0: %d", fuel_p0);
//
//				fuel_t0 = t0;	// NOTE: This assumes we start with battery ~100%
//			}
//
//			NRF_LOG_DEBUG("runtime_estimate: %d", runtime_estimate);
//			NRF_LOG_DEBUG("time_now: %d", time_now);
//			NRF_LOG_DEBUG("t0: %d", t0);
//			NRF_LOG_DEBUG("fuel_t0: %d", fuel_t0);
//			NRF_LOG_DEBUG("(runtime_estimate - (time_now - fuel_t0)): %d", (runtime_estimate - (time_now - fuel_t0)));
//			NRF_LOG_DEBUG("100*(runtime_estimate - (time_now - fuel_t0)) / runtime_estimate: %d", 100*(runtime_estimate - (time_now - fuel_t0)) / runtime_estimate);
//			NRF_LOG_DEBUG("(runtime_estimate - (time_now - fuel_t0)) / (1.0f*runtime_estimate): " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT((runtime_estimate - (time_now - fuel_t0)) / (1.0f*runtime_estimate)));
//
//			if ((time_now - fuel_t0) > runtime_estimate) {	// set to 0 if already past runtime_estimate
//				fuel_percent = 0;
//				NRF_LOG_DEBUG("set fuel_percent to 0");
//			} else {
////				NRF_LOG_DEBUG("runtime_estimate: %d", runtime_estimate);
////				NRF_LOG_DEBUG("time_now: %d", time_now);
////				NRF_LOG_DEBUG("t0: %d", t0);
////				NRF_LOG_DEBUG("fuel_t0: %d", fuel_t0);
////				NRF_LOG_DEBUG("(runtime_estimate - (time_now - fuel_t0)): %d", (runtime_estimate - (time_now - fuel_t0)));
////				NRF_LOG_DEBUG("100*(runtime_estimate - (time_now - fuel_t0)) / runtime_estimate: %d", 100*(runtime_estimate - (time_now - fuel_t0)) / runtime_estimate);
////				NRF_LOG_DEBUG("(runtime_estimate - (time_now - fuel_t0)) / (1.0f*runtime_estimate): " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT((runtime_estimate - (time_now - fuel_t0)) / (1.0f*runtime_estimate)));
//
//				// Need to be careful.. don't lose precision (use float), and don't overflow int (break into 2 calcs)
//				float temp = (runtime_estimate - (time_now - fuel_t0)) / (1.0f*runtime_estimate);	// % of runtime estimate.  Calc separately to avoid uint32 overflow
//				NRF_LOG_DEBUG("temp*1000: %d", temp*1000);
//				NRF_LOG_DEBUG("temp: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(temp));
//				fuel_percent = 1000 * 100*temp;	// 100x b/c %, 1000x for 3 decimal places
//			}
//		}
	// No need to modify calculation for standard LiPo battery
	} else {
		fuel_percent = fuel_percent_raw;
	}


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
    	NRF_LOG_WARNING("** WARNING in UV_SI1145_init(), err_code: %d **", err_code);
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
    	NRF_LOG_WARNING("** WARNING in UV_SI1145_read(), err_code: %d **", err_code);
    	return err_code;
    }
//    APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(&m_twi, UV_SI1145_addr, cmd, 4);
    APP_ERROR_CHECK(err_code);
    // Convert and save
    UV_SI1145_VIS_value = 	cmd[0] | (cmd[1] << 8);	// combine 2 bytes (MSB is 2nd)
    UV_SI1145_IR_value = 	cmd[2] | (cmd[3] << 8);	// combine 2 bytes (MSB is 2nd)
	NRF_LOG_DEBUG("cmd[0] = 0x%x", cmd[0]);
	NRF_LOG_DEBUG("cmd[1] = 0x%x", cmd[1]);
	NRF_LOG_DEBUG("cmd[2] = 0x%x", cmd[2]);
	NRF_LOG_DEBUG("cmd[3] = 0x%x", cmd[3]);

    // Read the UV data
    regt = 0x2C;	// start of UV data
    err_code = nrf_drv_twi_tx(&m_twi, UV_SI1145_addr, &regt, 1, true);
    APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_rx(&m_twi, UV_SI1145_addr, cmd, 2);
    APP_ERROR_CHECK(err_code);
    // Convert and save
    UV_SI1145_UV_value = 	cmd[0] | (cmd[1] << 8);	// combine 2 bytes (MSB is 2nd)
	NRF_LOG_DEBUG("cmd[0] = 0x%x", cmd[0]);
	NRF_LOG_DEBUG("cmd[1] = 0x%x", cmd[1]);


    nrf_drv_twi_disable(&m_twi);		// for saving power

    return err_code;


}


// Ambient Light Sensor LTR-329ALS-01 Settings
static ret_code_t ambient_light_set_control(uint8_t gain, bool reset, bool active) {

	// Sets the gain, SW reset and mode of LTR303
	// Default value is 0x00
	// If gain = 0, device is set to 1X gain (default)
	// If gain = 1, device is set to 2X gain
	// If gain = 2, device is set to 4X gain
	// If gain = 3, device is set to 8X gain
	// If gain = 4, invalid
	// If gain = 5, invalid
	// If gain = 6, device is set to 48X gain
	// If gain = 7, device is set to 96X gain
	//----------------------------------------
	// If reset = false(0), initial start-up procedure not started (default)
	// If reset = true(1), initial start-up procedure started
	//----------------------------------------
	// If mode = false(0), stand-by mode (default)
	// If mode = true(1), active mode

	uint8_t control = 0x00;

	// sanity check for gain
	if (gain > 3 && gain < 6) {
		gain = 0x00;
	}
	else if(gain >= 7) {
		gain = 0x00;
	}

	// control byte logic
	control |= gain << 2;
	if(reset) {
		control |= 0x02;
	}
	if(active) {
		control |= 0x01;
	}


    nrf_drv_twi_enable(&m_twi);		// for saving power

    // Write default coefficients, maybe later read and overwrite with product-calibrated ones
    uint8_t cmd[] = {LTR303_CONTR, control	};
    err_code = nrf_drv_twi_tx(&m_twi, LTR303_ADDR, cmd, sizeof(cmd), false);
    if (err_code) {	// handle error outside
    	NRF_LOG_WARNING("** WARNING in UV_SI1145_init(), err_code: %d **", err_code);
    	return err_code;
    }

    nrf_drv_twi_disable(&m_twi);		// for saving power

    return err_code;

}


// Ambient Light Sensor LTR-329ALS-01 Settings
static ret_code_t ambient_light_set_meas_rate(uint8_t integrationTime, uint8_t measurementRate) {

	// Sets the integration time and measurement rate of the sensor
	// integrationTime is the measurement time for each ALs cycle
	// measurementRate is the interval between DATA_REGISTERS update
	// measurementRate must be set to be equal or greater than integrationTime
	// Default value is 0x03
	// If integrationTime = 0, integrationTime will be 100ms (default)
	// If integrationTime = 1, integrationTime will be 50ms
	// If integrationTime = 2, integrationTime will be 200ms
	// If integrationTime = 3, integrationTime will be 400ms
	// If integrationTime = 4, integrationTime will be 150ms
	// If integrationTime = 5, integrationTime will be 250ms
	// If integrationTime = 6, integrationTime will be 300ms
	// If integrationTime = 7, integrationTime will be 350ms
	//------------------------------------------------------
	// If measurementRate = 0, measurementRate will be 50ms
	// If measurementRate = 1, measurementRate will be 100ms
	// If measurementRate = 2, measurementRate will be 200ms
	// If measurementRate = 3, measurementRate will be 500ms (default)
	// If measurementRate = 4, measurementRate will be 1000ms
	// If measurementRate = 5, measurementRate will be 2000ms
	// If measurementRate = 6, measurementRate will be 2000ms
	// If measurementRate = 7, measurementRate will be 2000ms

	uint8_t measurement = 0x00;

	// Perform sanity checks
	if(integrationTime >= 0x07) {
		integrationTime = 0x00;
	}
	if(measurementRate >= 0x07) {
		measurementRate = 0x00;
	}

	measurement |= integrationTime << 3;
	measurement |= measurementRate;


    nrf_drv_twi_enable(&m_twi);		// for saving power

    // Write default coefficients, maybe later read and overwrite with product-calibrated ones
    uint8_t cmd[] = {LTR303_MEAS_RATE, measurement	};
    err_code = nrf_drv_twi_tx(&m_twi, LTR303_ADDR, cmd, sizeof(cmd), false);
    if (err_code) {	// handle error outside
    	NRF_LOG_WARNING("** WARNING in UV_SI1145_init(), err_code: %d **", err_code);
    	return err_code;
    }

    nrf_drv_twi_disable(&m_twi);		// for saving power

    return err_code;

}


// Ambient Light Sensor LTR-329ALS-01 read
static ret_code_t ambient_light_read() {

	uint8_t cmd[LTR303_DATA_SIZE];

    nrf_drv_twi_enable(&m_twi);		// for saving power


    // Tell it which register we want to start reading from
    uint8_t reg0 = LTR303_DATA_CH1_0;
    err_code = nrf_drv_twi_tx(&m_twi, LTR303_ADDR, &reg0, sizeof(reg0), true);
    if (err_code) {	// handle error outside
    	return err_code;
    }
//    APP_ERROR_CHECK(err_code);
    // Read the value from that register
	err_code = nrf_drv_twi_rx(&m_twi, LTR303_ADDR, cmd, LTR303_DATA_SIZE);
    if (err_code) {	// handle error outside
    	return err_code;
    }
//	APP_ERROR_CHECK(err_code);


    nrf_drv_twi_disable(&m_twi);		// for saving power


    // Convert and store the values
    ambient_CH1 = (cmd[1] << 8) | cmd[0];
    ambient_CH0 = (cmd[3] << 8) | cmd[2];

    return err_code;

}


// UVA Settings
static ret_code_t UVA_set_control(uint8_t integrationTime, bool sleep) {

	// If integrationTime = 0, integrationTime will be 1/2x
	// If integrationTime = 1, integrationTime will be 1x (default)
	// If integrationTime = 2, integrationTime will be 2x
	// If integrationTime = 3, integrationTime will be 4x

	// If active = 0, sleep
	// If active = 1, wake

	// sanity check for integrationTime
	if (integrationTime > 3) {
		integrationTime = 0x01;
	}

	uint8_t control = 0x02;
	control |= integrationTime << 2;
	if(sleep) {
		control |= 0x01;
	}
//	NRF_LOG_DEBUG("control: %d", control);


    nrf_drv_twi_enable(&m_twi);		// for saving power

    // Write default coefficients, maybe later read and overwrite with product-calibrated ones
    uint8_t cmd[] = {control	};
    err_code = nrf_drv_twi_tx(&m_twi, VEML6070_ADDR_L, cmd, sizeof(cmd), false);
    if (err_code) {	// handle error outside
    	NRF_LOG_WARNING("** WARNING in UVA_set_control(), err_code: %d **", err_code);
    	return err_code;
    }

    nrf_drv_twi_disable(&m_twi);		// for saving power

    return err_code;

}

// UVA Read
static ret_code_t UVA_read() {

	uint8_t msb;
	uint8_t lsb;
//	uint8_t reg0;

    nrf_drv_twi_enable(&m_twi);		// for saving power


    // Get the MSB from one address and register
//    reg0 = 0x00;
//    err_code = nrf_drv_twi_tx(&m_twi, VEML6070_ADDR_H, &reg0, sizeof(reg0), true);
//    if (err_code) {	// handle error outside
//    	return err_code;
//    }
////    APP_ERROR_CHECK(err_code);
//    // Read the value from that register
	err_code = nrf_drv_twi_rx(&m_twi, VEML6070_ADDR_H, &msb, 1);
    if (err_code) {	// handle error outside
    	return err_code;
    }
//	NRF_LOG_DEBUG("msb: %d", msb);
//	APP_ERROR_CHECK(err_code);

    // Now get the LSB from another address and register
//    reg0 = 0x01;
//    err_code = nrf_drv_twi_tx(&m_twi, VEML6070_ADDR_L, &reg0, sizeof(reg0), true);
//    if (err_code) {	// handle error outside
//    	return err_code;
//    }
////    APP_ERROR_CHECK(err_code);
//    // Read the value from that register
	err_code = nrf_drv_twi_rx(&m_twi, VEML6070_ADDR_L, &lsb, 1);
    if (err_code) {	// handle error outside
    	return err_code;
    }
//	NRF_LOG_DEBUG("lsb: %d", lsb);
//	APP_ERROR_CHECK(err_code);


    nrf_drv_twi_disable(&m_twi);		// for saving power


    // Convert and store the values
    uva_value = (msb << 8) | lsb;

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
		NRF_LOG_ERROR("** ERROR: HPM checksum ERROR! **");
		return 2;
	}

    return err_code;
}


/**
 * For getting the Temp internally through the nRF
 */
int32_t get_temp_nrf(void) {

    // Can't do above with SoftDevice
    int32_t temp;
    err_code = sd_temp_get(&temp);
    temp = temp*100 / 4;

	return temp;
}


/**
 * Read all of the sensors
 */
void get_data() {


//	// Spec CO, Analog read.  TODO: implement averaging over 128 samples
//	if (using_component(SPEC_CO, components_used)) {
//		NRF_LOG_INFO("");
//		NRF_LOG_INFO("Testing Spec CO...");
//		NRF_LOG_INFO("------------------");
//
//		// Wait for sensor to settle from when ADP turned on
//		NRF_LOG_INFO("Waiting for specCO_startup_wait_done..");
//		while (!specCO_startup_wait_done) {
////			nrf_delay_ms(1000);
//		}
//		NRF_LOG_DEBUG("--SPEC_CO WAIT DONE");
//
//		// Pre-read wait, prevents garbage reading
//		nrf_saadc_value_t specCO_temp;
//		nrf_drv_saadc_sample_convert(SPEC_CO_CHANNEL_NUM, &specCO_temp);
//		nrf_delay_ms(PRE_READ_WAIT);
//
//		NRF_LOG_DEBUG("Sampling...");
//		int specCO_total = 0;
//		// read it a bunch of times and then average
//		for (int i = 0; i < 128; i++) {
//			nrf_drv_saadc_sample_convert(SPEC_CO_CHANNEL_NUM, &specCO_temp);
//			specCO_total += specCO_temp;
//
//			nrf_delay_ms(SPEC_CO_DELAY);
//		}
//
//		NRF_LOG_DEBUG("specCO_temp: %d", specCO_temp);
//
////		specCO_value = specCO_total/128.0f;
////		NRF_LOG_DEBUG("specCO_value: %d", specCO_value);
//		specCO_value = specCO_total/128;
//		NRF_LOG_DEBUG("specCO_value: %d", specCO_value);
////		NRF_LOG_DEBUG("specCO_value (mV): %d", specCO_value*1000*1000/V_to_adc_1000);
//		NRF_LOG_INFO("specCO_value (mV): %d", specCO_value*adc_to_mV);
//
//
//
//
//		// Read #2
//		nrf_delay_ms(SPEC_CO_WAIT_BETWEEN_SAMPLES);
//		nrf_drv_saadc_sample_convert(SPEC_CO_CHANNEL_NUM, &specCO_temp);
//		nrf_delay_ms(PRE_READ_WAIT);
//
//		NRF_LOG_DEBUG("Sampling...");
//		specCO_total = 0;
//		// read it a bunch of times and then average
//		for (int i = 0; i < 128; i++) {
//			nrf_drv_saadc_sample_convert(SPEC_CO_CHANNEL_NUM, &specCO_temp);
//			specCO_total += specCO_temp;
//
//			nrf_delay_ms(SPEC_CO_DELAY);
//		}
//
//		NRF_LOG_DEBUG("specCO_temp: %d", specCO_temp);
//		specCO_value_10 = specCO_total/128;
//		NRF_LOG_DEBUG("specCO_value_10: %d", specCO_value_10);
//		NRF_LOG_INFO("specCO_value_10 (mV): %d", specCO_value_10*adc_to_mV);
//
//
//
//
//		// Read #3
//		nrf_delay_ms(SPEC_CO_WAIT_BETWEEN_SAMPLES);
//		nrf_drv_saadc_sample_convert(SPEC_CO_CHANNEL_NUM, &specCO_temp);
//		nrf_delay_ms(PRE_READ_WAIT);
//
//		NRF_LOG_DEBUG("Sampling...");
//		specCO_total = 0;
//		// read it a bunch of times and then average
//		for (int i = 0; i < 128; i++) {
//			nrf_drv_saadc_sample_convert(SPEC_CO_CHANNEL_NUM, &specCO_temp);
//			specCO_total += specCO_temp;
//
//			nrf_delay_ms(SPEC_CO_DELAY);
//		}
//
//		NRF_LOG_DEBUG("specCO_temp: %d", specCO_temp);
//		specCO_value_20 = specCO_total/128;
//		NRF_LOG_DEBUG("specCO_value_20: %d", specCO_value_20);
//		NRF_LOG_INFO("specCO_value_20 (mV): %d", specCO_value_20*adc_to_mV);
//
//
//
//
//		// Read #4
//		nrf_delay_ms(SPEC_CO_WAIT_BETWEEN_SAMPLES);
//		nrf_drv_saadc_sample_convert(SPEC_CO_CHANNEL_NUM, &specCO_temp);
//		nrf_delay_ms(PRE_READ_WAIT);
//
//		NRF_LOG_DEBUG("Sampling...");
//		specCO_total = 0;
//		// read it a bunch of times and then average
//		for (int i = 0; i < 128; i++) {
//			nrf_drv_saadc_sample_convert(SPEC_CO_CHANNEL_NUM, &specCO_temp);
//			specCO_total += specCO_temp;
//
//			nrf_delay_ms(SPEC_CO_DELAY);
//		}
//
//		NRF_LOG_DEBUG("specCO_temp: %d", specCO_temp);
//		specCO_value_30 = specCO_total/128;
//		NRF_LOG_DEBUG("specCO_value_30: %d", specCO_value_30);
//		NRF_LOG_INFO("specCO_value_30 (mV): %d", specCO_value_30*adc_to_mV);
//
//
//
//
//		// Read #5
//		nrf_delay_ms(SPEC_CO_WAIT_BETWEEN_SAMPLES);
//		nrf_drv_saadc_sample_convert(SPEC_CO_CHANNEL_NUM, &specCO_temp);
//		nrf_delay_ms(PRE_READ_WAIT);
//
//		NRF_LOG_DEBUG("Sampling...");
//		specCO_total = 0;
//		// read it a bunch of times and then average
//		for (int i = 0; i < 128; i++) {
//			nrf_drv_saadc_sample_convert(SPEC_CO_CHANNEL_NUM, &specCO_temp);
//			specCO_total += specCO_temp;
//
//			nrf_delay_ms(SPEC_CO_DELAY);
//		}
//
//		NRF_LOG_DEBUG("specCO_temp: %d", specCO_temp);
//		specCO_value_40 = specCO_total/128;
//		NRF_LOG_DEBUG("specCO_value_40: %d", specCO_value_40);
//		NRF_LOG_INFO("specCO_value_40 (mV): %d", specCO_value_40*adc_to_mV);
//
//
//
//
//		// Read #6
//		nrf_delay_ms(SPEC_CO_WAIT_BETWEEN_SAMPLES);
//		nrf_drv_saadc_sample_convert(SPEC_CO_CHANNEL_NUM, &specCO_temp);
//		nrf_delay_ms(PRE_READ_WAIT);
//
//		NRF_LOG_DEBUG("Sampling...");
//		specCO_total = 0;
//		// read it a bunch of times and then average
//		for (int i = 0; i < 128; i++) {
//			nrf_drv_saadc_sample_convert(SPEC_CO_CHANNEL_NUM, &specCO_temp);
//			specCO_total += specCO_temp;
//
//			nrf_delay_ms(SPEC_CO_DELAY);
//		}
//
//		NRF_LOG_DEBUG("specCO_temp: %d", specCO_temp);
//		specCO_value_50 = specCO_total/128;
//		NRF_LOG_DEBUG("specCO_value_50: %d", specCO_value_50);
//		NRF_LOG_INFO("specCO_value_50 (mV): %d", specCO_value_50*adc_to_mV);
//
//
//
//
//		// Read #7
//		nrf_delay_ms(SPEC_CO_WAIT_BETWEEN_SAMPLES);
//		nrf_drv_saadc_sample_convert(SPEC_CO_CHANNEL_NUM, &specCO_temp);
//		nrf_delay_ms(PRE_READ_WAIT);
//
//		NRF_LOG_DEBUG("Sampling...");
//		specCO_total = 0;
//		// read it a bunch of times and then average
//		for (int i = 0; i < 128; i++) {
//			nrf_drv_saadc_sample_convert(SPEC_CO_CHANNEL_NUM, &specCO_temp);
//			specCO_total += specCO_temp;
//
//			nrf_delay_ms(SPEC_CO_DELAY);
//		}
//
//		NRF_LOG_DEBUG("specCO_temp: %d", specCO_temp);
//		specCO_value_60 = specCO_total/128;
//		NRF_LOG_DEBUG("specCO_value_60: %d", specCO_value_60);
//		NRF_LOG_INFO("specCO_value_60 (mV): %d", specCO_value_60*adc_to_mV);
//
//
//	}



//NRF_LOG_DEBUG("--ST");
//if (0) {

	/** Initialize some stuff **/
    // ADP High Power wake, turn ON power to high power sensors
//	if (using_component(ADP_HIGH, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("WAKE, ADP_HIGH: Turning ON ADP Power...");
		NRF_LOG_INFO("-------------------------------");

		nrf_gpio_cfg_output(ADP2_PIN);
		NRF_LOG_INFO("ADP_HIGH: HIGH.");
		nrf_gpio_pin_set(ADP2_PIN);	// Enable HIGH, Turn OFF ADP
//	}
	// Turn on Sample LED
    nrf_gpio_cfg_output(SAMPLE_LED);
	nrf_gpio_pin_set(SAMPLE_LED);	// Enable HIGH, Turn ON LED
	// Sharp PM, Turn LED OFF (high)
	if (using_component(SHARP, components_used)) {
	    nrf_gpio_cfg_output(SHARP_PM_LED);
		nrf_gpio_pin_set(SHARP_PM_LED);
	}
//	//    // TWI (I2C) init
//	    twi_init();


	// RTC, TWI (I2C)
	if (using_component(RTC, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing RTC with I2C/TWI...");
		NRF_LOG_INFO("---------------------------");

		// Set time manually
		if (SETTING_TIME_MANUALLY && !time_was_set) {
			NRF_LOG_WARNING("** WARNING: SETTING TIME MANUALLY **");
//			set_rtc(00, 44, 21, 	3, 6, 3, 18);	// 2018-03-06 Tues, 9:44:00 pm, NOTE: GMT!!!
//			set_rtc(00, 9, 13, 5, 10, 5, 18);	// about 11 seconds of delay
			set_rtc(00, 34, 17 +4, 	6, 27, 7, 18);	// about 11 seconds of delay
			time_was_set = 1;
			// NOTE: turn OFF SETTING_TIME_MANUALLY after
		}

		// Set time with new BLE time that was sent
		if (setting_new_time) {
			NRF_LOG_INFO("setting_new_time: %d", time_to_be_set);

		    struct tm * p_tm;
		    p_tm = gmtime(&time_to_be_set);	// need to adjust for INITIAL_SETTLING_WAIT as well
		    set_rtc((uint8_t) p_tm->tm_sec + INITIAL_SETTLING_WAIT/1000, (uint8_t) p_tm->tm_min, (uint8_t) p_tm->tm_hour, (uint8_t) p_tm->tm_wday + 1, (uint8_t) p_tm->tm_mday, (uint8_t) p_tm->tm_mon + 1, (uint8_t) p_tm->tm_year - 100);

		    setting_new_time = false;	// reset flag
		}

		// Try reading a few times in case there is error
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
			NRF_LOG_ERROR("** ERROR: RTC read, err_code=%d **", err_code);
			time_now = 0;
			err_cnt++;
			rtc_error_cnt_total++;
		}



		if (t0 == 0) {	// Record initial time
			t0 = time_now;
		}


		NRF_LOG_DEBUG("adjusting_timer_start: %d", adjusting_timer_start);

//////		// FOR TESTING, REMOVE LATER
//		adjusting_timer_start = 0;
//		time_now = 1533076231;	//1533076200

		// If nRF clock and RTC are unsynced, correct it
		if (!adjusting_timer_start && !is_live_streaming) {
			uint32_t timer_offset_s = time_now % (log_interval/1000);
			if (timer_offset_s != 0) {
				int32_t ms_to_next_meas = log_interval - timer_offset_s*1000 - INITIAL_SETTLING_WAIT;

				// Correct if the wait is too short (maybe RTC was slower)
				if (ms_to_next_meas < log_interval/2) {
					ms_to_next_meas += log_interval;
					skipping_next_meas_loop = true;
					NRF_LOG_INFO("skipping_next_meas_loop: %d", skipping_next_meas_loop);
				} else {

				}

			    start_adjustment_wait_done = false;
				err_code = app_timer_start(start_adjustment_timer, APP_TIMER_TICKS(ms_to_next_meas), NULL);
				if (err_code) {
					NRF_LOG_WARNING("** WARNING: %d, app_timer_start(ms_to_next_meas)", err_code);
				}

				NRF_LOG_DEBUG("time_now: %d", time_now);
				NRF_LOG_DEBUG("ms_to_next_meas: %d", ms_to_next_meas);
				NRF_LOG_DEBUG("log_interval: %d", log_interval);
				NRF_LOG_DEBUG("INITIAL_SETTLING_WAIT: %d", INITIAL_SETTLING_WAIT);
				NRF_LOG_DEBUG("ms_to_next_meas - log_interval: %d", ms_to_next_meas - log_interval);
				NRF_LOG_DEBUG("(ms_to_next_meas - log_interval)/1000: %d", (ms_to_next_meas - log_interval)/1000);
//				NRF_LOG_DEBUG("test math: %d", (146*1000 - 300*1000)/1000);
				int32_t amount_to_adjust = (ms_to_next_meas - log_interval + INITIAL_SETTLING_WAIT)/1000;
				NRF_LOG_DEBUG("amount_to_adjust: %d", amount_to_adjust);


				// Adjust current time so it lines up with the others
//				time_now += (ms_to_next_meas - log_interval + INITIAL_SETTLING_WAIT)/1000;
				time_now += amount_to_adjust;

			}
		}


		// Try to make all of the measurement timers sync'ed to start at predictable times (e.g. 4:05, 4:10, 4:15, etc)
		if (adjusting_timer_start) {
			int32_t time_to_wait_s = log_interval/1000 - (time_now % (log_interval/1000)) - INITIAL_SETTLING_WAIT/1000;
//			uint32_t max_sensor_wait_ms = (PLANTOWER_STARTUP_WAIT_TIME > SPEC_CO_STARTUP_WAIT_TIME) ? PLANTOWER_STARTUP_WAIT_TIME : SPEC_CO_STARTUP_WAIT_TIME;
			if (time_to_wait_s < 2*max_sensor_wait_ms/1000) {	// Make sure we wait more than the sensor wait time; don't want to adjust time while still measuring
				time_to_wait_s += log_interval/1000;
				skipping_next_meas_loop = true;
				NRF_LOG_INFO("skipping_next_meas_loop: %d", skipping_next_meas_loop);
			}
			NRF_LOG_INFO("time_to_wait_s: %d", time_to_wait_s);
		    start_adjustment_wait_done = false;
			err_code = app_timer_start(start_adjustment_timer, APP_TIMER_TICKS(time_to_wait_s*1000), NULL);
			if (err_code) {
				NRF_LOG_WARNING("** WARNING: %d, app_timer_start(time_to_wait_s)", err_code);
			}
//			APP_ERROR_CHECK(err_code);
		}

		// Print the reading
		NRF_LOG_INFO("time_now: %d", time_now);
		NRF_LOG_INFO("rtc_temp: %d", rtc_temp);
	}


    // Init Fuel Gauge
	if (using_component(FUEL_GAUGE, components_used)) {
		fuel_gauge_wake();	// Turn on after things have settled, but give it time to estimate
	}
	// Init Ambient Light Sensor
	if (using_component(AMBIENT_LTR, components_used)) {
//	if (0 && using_component(AMBIENT_LTR, components_used)) {
		// Init sensor
		nrf_delay_ms(100);	// Required startup wait

//		ambient_light_set_control(LTR329_gain, true, true);
//		nrf_delay_ms(1000);	// Wakeup time

		ambient_light_set_meas_rate(LTR329_integration_time, LTR329_meas_rate);
		ambient_light_set_control(LTR329_gain, false, true);
//		ambient_light_set_control(LTR329_gain, true, true);
//		nrf_delay_ms(10);	// Wakeup time (Datasheet, but doesn't work if too small)
		nrf_delay_ms(100);	// Wakeup time
	}
	// Init UVA Sensor
	if (using_component(UVA_VEML, components_used)) {
//	if (0 && using_component(UVA_VEML, components_used)) {
		// Init sensor
//		nrf_delay_ms(1000);	// wait for some measurements
		UVA_set_control(UVA_integration_time, false);
//		nrf_delay_ms(1000);	// wait for some measurements
	}

//	// ADC setup
//    saadc_init();





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
			NRF_LOG_DEBUG("Start DHT read..");
			err_code = dht_read();
			if (err_code == DHTLIB_ERROR_TIMEOUT) {
				NRF_LOG_INFO("* RETRY: DHT TIMEOUT ERROR, err_code=%d *", err_code);
				avoided_error_cnt++;
			}
			nrf_delay_ms(500);
		}
		if (err_code) {
			NRF_LOG_ERROR("** ERROR: DHT read, err_code=%d **", err_code);
			dht_temp_C = 0;
			dht_humidity = 0;
			err_cnt++;
			dht_error_cnt_total++;
		} else {
			NRF_LOG_DEBUG("SUCCESS: DHT READ");
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

		// Wait for sensor to settle from when ADP turned on
		NRF_LOG_INFO("Waiting for figCO2_startup_wait_done: %d", figaroCO2_startup_wait_ms);
		while (!figCO2_startup_wait_done) {
//			nrf_delay_ms(1000);
		}
		NRF_LOG_DEBUG("--FIGARO_CO2 WAIT DONE");

//		// Wait 2 seconds
//		nrf_delay_ms(FIGARO_CO2_STARTUP_WAIT_TIME);

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
			NRF_LOG_ERROR("** ERROR: Figaro CO2 read, err_code=%d **", err_code);
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
			NRF_LOG_ERROR("** ERROR: BME280 TRH read, err_code=%d **", err_code);
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


	// Trying to read sample from ADC
	if (using_component(ADC, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing ADC...");
		NRF_LOG_INFO("--------------");

		nrf_drv_saadc_sample_convert(ADC_CHANNEL_NUM, &adc_value);
		NRF_LOG_DEBUG("Sample 1: %d", adc_value);

		// convert to V
//		NRF_LOG_DEBUG("adc_to_V*1000: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(adc_to_V*1000));
//		NRF_LOG_DEBUG("adc_value (mV): " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(adc_value*adc_to_V*1000));
		NRF_LOG_INFO("adc_value (mV): " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(adc_value*1000*1000/V_to_adc_1000));
//		NRF_LOG_DEBUG("adc_value (\%): " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(adc_value*adc_to_V/MBED_VREF));
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
		NRF_LOG_DEBUG("Sampling... (between LED stuff)");
		NRF_LOG_DEBUG("Turning LED ON (low)");
	//    nrf_gpio_cfg_output(SHARP_PM_LED);
		nrf_gpio_pin_clear(SHARP_PM_LED);
		nrf_delay_us(280);

		// Taking sample (do it twice, just for testing)
		nrf_drv_saadc_sample_convert(SHARP_PM_CHANNEL_NUM, &sharpPM_value);

		// Turn LED off (high)
		nrf_delay_us(40);
		nrf_gpio_pin_set(SHARP_PM_LED);
		nrf_delay_us(9680);
		NRF_LOG_DEBUG("LED turned OFF (high)");

		NRF_LOG_DEBUG("Sample 1: %d", sharpPM_value);
		NRF_LOG_INFO("sharpPM_value (mV): %d", sharpPM_value*1000*1000/V_to_adc_1000);
	}


	// Figaro CO, Analog read.
	if (using_component(FIGARO_CO, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing Figaro CO...");
		NRF_LOG_INFO("--------------------");

		// Pre-read wait, prevents garbage reading
		nrf_drv_saadc_sample_convert(FIG_CO_CHANNEL_NUM, &figCO_value);
		nrf_delay_ms(PRE_READ_WAIT);

		NRF_LOG_DEBUG("Sampling...");
		nrf_drv_saadc_sample_convert(FIG_CO_CHANNEL_NUM, &figCO_value);
		NRF_LOG_INFO("figCO_value (mV): %d", figCO_value*1000*1000/V_to_adc_1000);
	}


	// Check nRF Battery Level
	if (using_component(BATTERY, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Checking Battery Level...");
		NRF_LOG_INFO("-------------------------");

		// Pre-read wait, prevents garbage reading
		nrf_drv_saadc_sample_convert(BATTERY_CHANNEL_NUM, &battery_value);
		nrf_delay_ms(PRE_READ_WAIT);

//		NRF_LOG_DEBUG("Sampling...");
		nrf_drv_saadc_sample_convert(BATTERY_CHANNEL_NUM, &battery_value);
//		NRF_LOG_DEBUG("V_to_adc_1000: %d", V_to_adc_1000);
//		NRF_LOG_DEBUG("battery_value (mV): %d", battery_value*adc_to_V*1000);
		NRF_LOG_INFO("battery_value (mV): %d", battery_value*1000*1000/V_to_adc_1000);
//		NRF_LOG_DEBUG("adc_to_V*1000: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(adc_to_V*1000));
//		adc_to_V = 1.0f / ((ADC_GAIN_VALUE / ADC_REFERENCE_VOLTAGE) * (pow(2, ADC_RESOLUTION_BITS)-1) );
//		NRF_LOG_DEBUG("battery_value (mV): %d", battery_value*adc_to_V*1000);
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
			NRF_LOG_ERROR("** ERROR: Fuel Gauge read, err_code=%d **", err_code);
			fuel_v_cell = 0;
			fuel_percent = 0;
			fuel_percent_raw = 0;
			err_cnt++;
			fuel_gauge_error_cnt_total++;
		} else {	// calculate % from raw value (maybe extrapolate)
			calc_fuel_percent();
		}

		// Read Fuel
//		NRF_LOG_DEBUG("battery_scale_factor*1000: %d", battery_scale_factors[battery_type_used]*1000);
		NRF_LOG_INFO("fuel_v_cell: %d", fuel_v_cell);
		NRF_LOG_INFO("fuel_percent_raw: %d", fuel_percent_raw);
		NRF_LOG_INFO("fuel_percent: %d", fuel_percent);
//		NRF_LOG_INFO("runtime_estimate: %d", runtime_estimate);
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

	// Ambient Light Sensor LTR-329ALS-01, TWI (I2C)
	if (using_component(AMBIENT_LTR, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing Ambient Light Sensor with I2C/TWI...");
		NRF_LOG_INFO("--------------------------------------------");

//		// Init sensor
//		ambient_light_set_meas_rate(LTR329_integration_time, LTR329_meas_rate);
//		ambient_light_set_control(LTR329_gain, false, true);
//		nrf_delay_ms(200);	// wait for some measurements

		// Read the data
		err_code = ambient_light_read();
		if (err_code) {
			NRF_LOG_ERROR("** ERROR: ambient_light_read(), err_code=%d **", err_code);
			ambient_CH0 = 0;
			ambient_CH1 = 0;
			err_cnt++;
		}
		NRF_LOG_INFO("ambient_CH0: %d", ambient_CH0);
		NRF_LOG_INFO("ambient_CH1: %d", ambient_CH1);
	}

	// UVA Sensor, TWI (I2C)
	if (using_component(UVA_VEML, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("UVA with I2C/TWI...");
		NRF_LOG_INFO("-------------------");

//		// Init sensor
//		nrf_delay_ms(1000);	// wait for some measurements
//		UVA_set_control(UVA_integration_time, false);
//		nrf_delay_ms(1000);	// wait for some measurements

		// NOTE: need to wait a bit to integrate measurement
		nrf_delay_ms(UVA_VEML_MEAS_DELAY);


		// Read the data
		err_code = UVA_read();
		if (err_code) {
			NRF_LOG_ERROR("** ERROR: UVA_read(), err_code=%d **", err_code);
			uva_value = 0;
			err_cnt++;
		}
		NRF_LOG_INFO("uva_value: %d", uva_value);


	}


//NRF_LOG_DEBUG("--ST");
//if (0) {

	// Small Plantower, TWI (I2C)
	if (using_component(SMALL_PLANTOWER, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing Small Plantower with I2C/TWI...");
		NRF_LOG_INFO("---------------------------------------");
		// Wait for sensor to settle from when ADP turned on
		NRF_LOG_INFO("Waiting for plantower_startup_wait_done: %d", plantower_startup_wait_ms);
		while (!plantower_startup_wait_done) {
//			nrf_delay_ms(1000);
		}
		NRF_LOG_DEBUG("--PLANTOWER WAIT DONE");

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
			NRF_LOG_ERROR("** ERROR: PLANTOWER read, err_code=%d **", err_code);
			plantower_2_5_value = 0;
			plantower_10_value = 0;
//			// FOR TESTING, REMOVE LATER
//			plantower_2_5_value = 3;
//			plantower_10_value = 7;
			err_cnt++;
			plantower_error_cnt_total++;
		}

		NRF_LOG_INFO("plantower_2_5_value = %d", plantower_2_5_value);
		NRF_LOG_INFO("plantower_10_value = %d", plantower_10_value);
//		NRF_LOG_DEBUG("&plantower_2_5_value = %d", &plantower_2_5_value);
//		NRF_LOG_DEBUG("&plantower_10_value = %d", &plantower_10_value);
	}
//}	// REMOVE


	// Uninitialize TWI/I2C (need to do this before
	NRF_LOG_DEBUG("--BFGS");
	if (using_component(FUEL_GAUGE, components_used)) {
		fuel_gauge_sleep();	// Turn off to save power
	}
	NRF_LOG_DEBUG("--AFGS");
	// Sleep Ambient Light Sensor
	if (using_component(AMBIENT_LTR, components_used)) {
		ambient_light_set_control(LTR329_gain, false, false);
	}
	// Sleep UVA Sensor
	if (using_component(UVA_VEML, components_used)) {
		UVA_set_control(UVA_integration_time, true);
	}
//	// TWI (I2C) UNinit
//    nrf_drv_twi_uninit(&m_twi);



    // ADP High Power sleep, turn OFF power to high power sensors
	if (!is_live_streaming && using_component(ADP_HIGH, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("SLEEP, ADP_HIGH: Turning OFF ADP Power...");
		NRF_LOG_INFO("-----------------------------------------");

		nrf_gpio_cfg_output(ADP2_PIN);
		NRF_LOG_INFO("ADP_HIGH: LOW.");
		nrf_gpio_pin_clear(ADP2_PIN);	// Enable HIGH, Turn OFF ADP
	}


	// Spec CO, Analog read.  TODO: implement averaging over 128 samples
	if (using_component(SPEC_CO, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("Testing Spec CO...");
		NRF_LOG_INFO("------------------");

		// Wait for sensor to settle from when ADP turned on
		NRF_LOG_INFO("Waiting for specCO_startup_wait_done: %d", specCO_startup_wait_ms);
		while (!specCO_startup_wait_done) {
//			nrf_delay_ms(1000);
		}
		NRF_LOG_DEBUG("--SPEC_CO WAIT DONE");

		// Pre-read wait, prevents garbage reading
		nrf_saadc_value_t specCO_temp;
		nrf_drv_saadc_sample_convert(SPEC_CO_CHANNEL_NUM, &specCO_temp);
		nrf_delay_ms(PRE_READ_WAIT);

//		NRF_LOG_DEBUG("Sampling...");
		int specCO_total = 0;
		// read it a bunch of times and then average
		for (int i = 0; i < 128; i++) {
			nrf_drv_saadc_sample_convert(SPEC_CO_CHANNEL_NUM, &specCO_temp);
			specCO_total += specCO_temp;

			nrf_delay_ms(SPEC_CO_DELAY);
		}

//		NRF_LOG_DEBUG("specCO_temp: %d", specCO_temp);

//		specCO_value = specCO_total/128.0f;
//		NRF_LOG_DEBUG("specCO_value: %d", specCO_value);
		specCO_value = specCO_total/128;
//		NRF_LOG_DEBUG("specCO_value: %d", specCO_value);
//		NRF_LOG_DEBUG("specCO_value (mV): %d", specCO_value*1000*1000/V_to_adc_1000);
		NRF_LOG_INFO("specCO_value (mV): %d", specCO_value*adc_to_mV);
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
				NRF_LOG_DEBUG("Need to initialize the clock..");
				err_code = nrf_drv_clock_init();	// TODO: Maybe remove, unnecessary?
				APP_ERROR_CHECK(err_code);
			}
			if (!nrf_drv_power_init_check() ) {
				NRF_LOG_DEBUG("Need to initialize the power driver..");
				err_code = nrf_drv_power_init(NULL);	// TODO: Maybe remove, unnecessary?
				APP_ERROR_CHECK(err_code);
			}
		nrf_drv_clock_lfclk_request(NULL);
		err_code = app_timer_init();	// needed for serial timeout checking
		NRF_LOG_DEBUG("err_code = %d", err_code);
		APP_ERROR_CHECK(err_code);
		err_code = nrf_serial_init(&serial_uart, &m_uart0_drv_config, &serial_config);
		NRF_LOG_DEBUG("err_code = %d", err_code);
		APP_ERROR_CHECK(err_code);

		// Need to send commands if not using autosend
		if (!USING_AUTOSEND) {

			NRF_LOG_DEBUG("Sending Command.. Disable Autosend");
			err_code = hpm_cmd_and_ack(hpm_stop_autosend_cmd);	//stop HPM autosend

			NRF_LOG_DEBUG("Sending Command.. Disable Autosend");
			err_code = hpm_cmd_and_ack(hpm_stop_autosend_cmd);	//Do again, since HPM may still be autosending, which may confuse cmd_ack
			NRF_LOG_DEBUG("Sending Command.. Start Measurement");
			err_code = hpm_cmd_and_ack(hpm_start_meas_cmd);	//start HPM
		}

		// Try to read to read it a few times
		err_code = NRF_ERROR_TIMEOUT;
		for (int i=0; (err_code == NRF_ERROR_TIMEOUT) && (i < HPM_NUM_RETRIES); i++) {
			// Try reading, and retry if timeout error.  HPM sends every 1000 ms, so make sure we try enough times
			NRF_LOG_DEBUG("HPM Read, Try #%d", i);
			err_code = NRF_ERROR_TIMEOUT;
			for (int j=0; (err_code == NRF_ERROR_TIMEOUT) && (j < (1000/HPM_SERIAL_TIMEOUT + 1)); j++) {
				err_code = hpm_read_meas();		//read the measurement
				if (err_code == NRF_ERROR_TIMEOUT) {
					NRF_LOG_DEBUG("* RETRY: HPM TIMEOUT ERROR, err_code=%d *", err_code);
					avoided_error_cnt++;
				} else {
					NRF_LOG_DEBUG("SUCCESS: HPM READ");
				}
			}

			// Try the whole reading process again if it didn't work
			if ((HPM_NUM_RETRIES > 1) && (err_code != NRF_SUCCESS)) {
				nrf_delay_ms(HPM_RETRY_WAIT);
				NRF_LOG_DEBUG("* RETRY: HPM wasn't read, err_code=%d *", err_code);
			}
		}

		if (err_code) {
			NRF_LOG_ERROR("** ERROR: HPM read UNKNOWN ERROR, err_code=%d **", err_code);
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
int test_main() {

	// Start basic stuff
	NRF_LOG_INFO("");
	NRF_LOG_INFO("");
	NRF_LOG_INFO("-----------------------");
	NRF_LOG_INFO("| Testing test_main() |");
	NRF_LOG_INFO("-----------------------");
	NRF_LOG_INFO("loop_num = %d", loop_num);
	NRF_LOG_DEBUG("wdt_triggered = %d", wdt_triggered);


    // ADP, turn ON all power
	NRF_LOG_INFO("");
	NRF_LOG_INFO("WAKE: Turning ON ADP Power...");
	NRF_LOG_INFO("-----------------------------");


    nrf_gpio_cfg_output(ADP1_PIN);
	nrf_gpio_pin_set(ADP1_PIN);		// Enable HIGH
	NRF_LOG_INFO("ADP1_PIN: HIGH.");

//    nrf_gpio_cfg_output(ADP2_PIN);
//	nrf_gpio_pin_set(ADP2_PIN);		// Enable HIGH
//	NRF_LOG_INFO("ADP2_PIN: HIGH.");

    nrf_gpio_cfg_output(STATUS_LED);
	nrf_gpio_pin_set(STATUS_LED);	// Enable HIGH, Turn ON LED
	NRF_LOG_DEBUG("STATUS_LED: HIGH.");
//	nrf_gpio_pin_clear(STATUS_LED);	// Enable LOW, Turn ON LED

//	nrf_delay_ms(1000);


	// Start timer for DHT startup wait (settling time is ~1.5s)
//	if (!nrf_drv_clock_init_check() ) {
//		err_code = nrf_drv_clock_init();
//		NRF_LOG_DEBUG("nrf_drv_clock_init() err_code: %d", err_code);
//		APP_ERROR_CHECK(err_code);
//	}
//	nrf_drv_clock_lfclk_request(NULL);
	dht_startup_wait_done = 0;
	if (using_component(DHT, components_used)) {
		err_code = app_timer_start(dht_startup_timer, APP_TIMER_TICKS(DHT_STARTUP_WAIT_TIME), NULL);
		APP_ERROR_CHECK(err_code);
	}
    plantower_startup_wait_done = 0;
	if (using_component(SMALL_PLANTOWER, components_used)) {
		err_code = app_timer_start(plantower_startup_timer, APP_TIMER_TICKS(plantower_startup_wait_ms), NULL);
		APP_ERROR_CHECK(err_code);
	}
    specCO_startup_wait_done = 0;
	if (using_component(SPEC_CO, components_used)) {
		err_code = app_timer_start(specCO_startup_timer, APP_TIMER_TICKS(specCO_startup_wait_ms), NULL);
		APP_ERROR_CHECK(err_code);
	}
    figCO2_startup_wait_done = 0;
	if (using_component(FIGARO_CO2, components_used)) {
		err_code = app_timer_start(figCO2_startup_timer, APP_TIMER_TICKS(figaroCO2_startup_wait_ms), NULL);
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
	NRF_LOG_DEBUG("--BTWI");
    twi_init();
	NRF_LOG_DEBUG("--ATWI");

	// All the measurements happen here
	err_cnt = 0;
	avoided_error_cnt = 0;
	for (int i=0; i < NUM_SAMPLES_PER_ON_CYCLE; i++) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("-- SAMPLE #%d/%d --", i+1, NUM_SAMPLES_PER_ON_CYCLE);

		// Read all of the sensors
		get_data();
		// Save the data to SD card
		if (!skipping_sdc_write && using_component(SDC, components_used)) {
			save_data();
		}

		// Wait between multiple samples as a buffer time, just in case
		if (NUM_SAMPLES_PER_ON_CYCLE > 1) {
			NRF_LOG_INFO("");
			NRF_LOG_INFO("Wait before next sample: %d ms", WAIT_BETWEEN_SAMPLES);
			nrf_delay_ms(WAIT_BETWEEN_SAMPLES);
		}

	}

	// TWI (I2C) UNinit
    nrf_drv_twi_uninit(&m_twi);

//    // FOR TESTING, REMOVE LATER
//    nrf_delay_ms(2000);
//	nrf_gpio_cfg_output(TWI_SCL_PIN);
//	nrf_gpio_cfg_output(TWI_SDA_PIN);
//	NRF_LOG_INFO("SDA/SCL: HIGH.");
//	nrf_gpio_pin_set(TWI_SCL_PIN);	// Enable HIGH, Turn OFF ADP
//	nrf_gpio_pin_set(TWI_SDA_PIN);	// Enable HIGH, Turn OFF ADP



    // ADP sleep, turn OFF all power
	if (!is_live_streaming && using_component(ADP, components_used)) {
		NRF_LOG_INFO("");
		NRF_LOG_INFO("SLEEP: Turning OFF ADP Power...");
		NRF_LOG_INFO("-------------------------------");

		nrf_gpio_cfg_output(ADP1_PIN);
		nrf_gpio_cfg_output(ADP2_PIN);
		NRF_LOG_INFO("ALL ADP: LOW.");
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
	//		NRF_LOG_DEBUG("Woke Temporarily");
			__WFE();
	//		NRF_LOG_DEBUG("Woke Temporarily");
		}
	}


	// Feed the Watchdog Timer
	NRF_LOG_INFO("");
	NRF_LOG_INFO("Feeding the Watchdog..");
	NRF_LOG_INFO("----------------------");
	NRF_LOG_INFO("WDT_TIMEOUT_MEAS: %d", WDT_TIMEOUT_MEAS);
//	NRF_LOG_DEBUG("WDT_TIMEOUT_SLEEP: %d", WDT_TIMEOUT_SLEEP);
	// check if SD card failed
	if (sd_write_failed && SD_FAIL_SHUTDOWN) {
		NRF_LOG_ERROR("** FATAL ERROR: sd_write_failed: %d **", sd_write_failed);
//		NRF_LOG_FLUSH();
		// wait until wdt runs out
		while (1) {}
	}
//    nrf_drv_wdt_channel_feed(wdt_meas_channel_id);
	nrf_drv_wdt_feed();	// use this instead for multiple wdt

    // Ending stuff
    avoided_error_cnt_total += avoided_error_cnt;
//	NRF_LOG_DEBUG("avoided_error_cnt = %d", avoided_error_cnt);
//	NRF_LOG_DEBUG("avoided_error_cnt_total = %d", avoided_error_cnt_total);
	if (avoided_error_cnt > 0) {
		NRF_LOG_WARNING("** ERRORS AVOIDED! **");

	}
	NRF_LOG_FLUSH();

    loop_num++;
    return err_cnt;
}

// If the battery level is too low, stop measurements and sleep forever
static void check_min_battery_level() {
//	if (fuel_percent < min_battery_level) {
	if (fuel_v_cell < min_battery_level) {
		NRF_LOG_WARNING("fuel_v_cell < min_battery_level: LOW POWER MODE");
		stop_measurements();

		// Ensure that all ADP's are off
		nrf_gpio_cfg_output(ADP1_PIN);
		nrf_gpio_cfg_output(ADP2_PIN);
		NRF_LOG_DEBUG("ALL ADP: LOW.");
		nrf_gpio_pin_clear(ADP1_PIN);	// Enable HIGH, Turn OFF ADP
		nrf_gpio_pin_clear(ADP2_PIN);	// Enable HIGH, Turn OFF ADP

	}
}


/**
 * @brief Runs everything.  Basically the main program
 */
//int test_all()
static void test_all()
{

	// Show that we're measuring, check it later so it doesn't interfere with BLE
	in_measuring_loop = true;

	// Set some flags now, so we know what state we were in at the start of the loop
	skipping_sdc_write =
			is_live_streaming 		||
			testing_sensors			||
			adjusting_timer_start;

	// MAIN ACTION IS HERE
	err_cnt = test_main();
	err_cnt_total += err_cnt;


	// Update Broadcasted values
	broadcast_data.err_cnt_total = err_cnt_total;
	broadcast_data.is_logging = is_logging;
	broadcast_data.fuel_percent = fuel_percent;
	// NOTE: Appears in reverse direction on Sniffer:
	// 0x0200000001000000D1130100 means
	//	broadcast_data.err_cnt_total = 2;
	//	broadcast_data.is_logging = 1;
	//	broadcast_data.fuel_percent = 70609;
	//	broadcast_data.err_cnt_total = 2;
	//	broadcast_data.is_logging = 1;
	//	broadcast_data.fuel_percent = 5;

//	err_code = sd_ble_gap_adv_data_set(NULL, 0, (uint8_t *) &broadcast_data, sizeof(broadcast_data));
//	NRF_LOG_INFO("sd_ble_gap_adv_data_set() err_code = %d", err_code);
//	APP_ERROR_CHECK(err_code);

////	ble_advdata_t *new_advdata;
//	new_advdata->adv_data.len = sizeof(broadcast_data);
//	err_code = ble_advdata_encode(new_advdata, (uint8_t *) &broadcast_data, );
//	APP_ERROR_CHECK(err_code);
//	err_code = ble_advdata_set(NULL, new_advdata);
//	APP_ERROR_CHECK(err_code);

	ble_advdata_t new_advdata;
    memset(&new_advdata, 0, sizeof(new_advdata));
    ble_advdata_service_data_t sr_service_data;
    memset(&sr_service_data, 0, sizeof(sr_service_data));
    sr_service_data.service_uuid = BLE_UUID_SENSEN_SERVICE;
//    sr_service_data.data.p_data = broadcast_data;
//    sr_service_data.data.size = sizeof(err_cnt_total) + sizeof(is_logging) + sizeof(fuel_percent);
    sr_service_data.data.p_data = (uint8_t *) &broadcast_data;
    sr_service_data.data.size = sizeof(broadcast_data);
    new_advdata.p_service_data_array = &sr_service_data;
    new_advdata.service_data_count++;

	err_code = ble_advdata_set(NULL, &new_advdata);
	APP_ERROR_CHECK(err_code);




	// Push values to App if Live Streaming
	if (is_live_streaming) {
		for (int i=0; i < APP_PUSH_RETRY_NUM; i++) {	// Keep trying
			err_code = push_live_stream_values();
			if (err_code == NRF_SUCCESS) {
				break;
			} else {
				NRF_LOG_WARNING("** WARNING: Trying again: push_live_stream_values(), err_code=%d **", err_code);
				nrf_delay_ms(MAX_CONN_INTERVAL*1.25);
			}
		}
		if (err_code) {
			NRF_LOG_ERROR("** ERROR: push_live_stream_values(), err_code=%d **", err_code);
		}

	}

	// Print out summary
	NRF_LOG_FLUSH();
	NRF_LOG_DEBUG("");
	NRF_LOG_INFO("-- SUMMARY --");

	NRF_LOG_INFO("log_interval = %d", log_interval);
	NRF_LOG_INFO("min_battery_level = %d", min_battery_level);
	NRF_LOG_INFO("is_live_streaming = %d", is_live_streaming);

//	NRF_LOG_DEBUG("ble_gap_address = %x:%x:%x:%x:%x:%x", ble_gap_address.addr[5], ble_gap_address.addr[4], ble_gap_address.addr[3], ble_gap_address.addr[2], ble_gap_address.addr[1], ble_gap_address.addr[0]);
	NRF_LOG_INFO("ble_gap_address = %02X:%02X:%02X:%02X:%02X:%02X", ble_gap_address.addr[5], ble_gap_address.addr[4], ble_gap_address.addr[3], ble_gap_address.addr[2], ble_gap_address.addr[1], ble_gap_address.addr[0]);
//	NRF_LOG_DEBUG("ble_gap_address = %x", ble_gap_address.addr);
//	NRF_LOG_DEBUG("ble_gap_address.addr[0] = %x", ble_gap_address.addr[0]);
//	NRF_LOG_DEBUG("ble_gap_address.addr[1] = %x", ble_gap_address.addr[1]);
//	NRF_LOG_DEBUG("ble_gap_address.addr[2] = %x", ble_gap_address.addr[2]);
//	NRF_LOG_DEBUG("ble_gap_address.addr[3] = %x", ble_gap_address.addr[3]);
//	NRF_LOG_DEBUG("ble_gap_address.addr[4] = %x", ble_gap_address.addr[4]);
//	NRF_LOG_DEBUG("ble_gap_address.addr[5] = %x", ble_gap_address.addr[5]);


	NRF_LOG_INFO("time_now = %d", time_now);
	NRF_LOG_INFO("err_cnt = %d", err_cnt);
	NRF_LOG_INFO("err_cnt_total = %d", err_cnt_total);
//	NRF_LOG_DEBUG("dht_error_cnt_total = %d", dht_error_cnt_total);
//	NRF_LOG_DEBUG("hpm_error_cnt_total = %d", hpm_error_cnt_total);
	NRF_LOG_DEBUG("*** test_main() COMPLETE!, next loop_num: %d ***", loop_num);
	NRF_LOG_FLUSH();

	// Final stuff to do before going back to sleep
	meas_loop_wait_done = false;
	in_measuring_loop = false;
	check_min_battery_level();


}





// Timeout handlers for startup waits with the single shot timers
static void meas_loop_handler(void * p_context) {
	if (skipping_next_meas_loop) {
		NRF_LOG_INFO("Skipped the coming meas_loop: %d", skipping_next_meas_loop);
		meas_loop_wait_done = false;
		skipping_next_meas_loop = false;
	} else {
		meas_loop_wait_done = true;
	}
	NRF_LOG_DEBUG("meas_loop_wait_done: %d", meas_loop_wait_done);
//	test_all();
}
static void plantower_startup_handler(void * p_context) {
	plantower_startup_wait_done = true;
	NRF_LOG_DEBUG("plantower_startup_wait_done: %d", plantower_startup_wait_done);
}
static void specCO_startup_handler(void * p_context) {
	specCO_startup_wait_done = true;
	NRF_LOG_DEBUG("specCO_startup_wait_done: %d", specCO_startup_wait_done);
}
static void figCO2_startup_handler(void * p_context) {
	figCO2_startup_wait_done = true;
	NRF_LOG_DEBUG("figCO2_startup_wait_done: %d", figCO2_startup_wait_done);
}
static void start_adjustment_handler(void * p_context) {
	start_adjustment_wait_done = true;
	NRF_LOG_DEBUG("start_adjustment_wait_done: %d", start_adjustment_wait_done);
}
static void dht_startup_handler(void * p_context) {
	dht_startup_wait_done = 1;
	NRF_LOG_DEBUG("dht_startup_wait_done: %d", dht_startup_wait_done);
}
static void hpm_startup_handler(void * p_context) {
	hpm_startup_wait_done = 1;
	NRF_LOG_DEBUG("hpm_startup_wait_done: %d", hpm_startup_wait_done);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timer for main code
    err_code = app_timer_create(&meas_loop_timer,
								APP_TIMER_MODE_REPEATED,
//								APP_TIMER_MODE_SINGLE_SHOT,
//								test_all);
    //								test_timer);
    								meas_loop_handler);
    APP_ERROR_CHECK(err_code);

	// Create timers for startup wait (depends on each sensor)
    err_code = app_timer_create(&plantower_startup_timer,
    							APP_TIMER_MODE_SINGLE_SHOT,
                                plantower_startup_handler);	// sets a flag when timer expires
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&specCO_startup_timer,
    							APP_TIMER_MODE_SINGLE_SHOT,
								specCO_startup_handler);	// sets a flag when timer expires
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&figCO2_startup_timer,
    							APP_TIMER_MODE_SINGLE_SHOT,
								figCO2_startup_handler);	// sets a flag when timer expires
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&start_adjustment_timer,
    							APP_TIMER_MODE_SINGLE_SHOT,
								start_adjustment_handler);	// sets a flag when timer expires
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&dht_startup_timer,
    							APP_TIMER_MODE_SINGLE_SHOT,
                                dht_startup_handler);	// sets a flag when timer expires
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&hpm_startup_timer,
    							APP_TIMER_MODE_SINGLE_SHOT,
                                hpm_startup_handler);	// sets a flag when timer expires
    APP_ERROR_CHECK(err_code);

//    // For live stream BLE
//    err_code = app_timer_create(&m_our_char_timer_id,
//    							APP_TIMER_MODE_REPEATED,
//								push_live_stream_values);	// sets a flag when timer expires
//    APP_ERROR_CHECK(err_code);


}


/**
 * Main program.  Comment whatever major things you want to run.
 */
int main(void) {


	// Initial delay so Fuel Gauge starts with good initial guess
	// NOTE: MAYBE MOVE THIS SOMEWHERE ELSE, if LEDs or ADPs are drawing power on powerup
	if (using_component(FUEL_GAUGE, components_used)) {
		nrf_delay_ms(INITIAL_FUEL_GAUGE_WAIT);
	}


    // Startup Message
    log_init();
	NRF_LOG_DEBUG("");
	NRF_LOG_DEBUG("-----------------");
	NRF_LOG_DEBUG("| Initial Setup |");
	NRF_LOG_DEBUG("-----------------");
    // Show which product we are using
	if (PRODUCT_TYPE == SUM) {
		NRF_LOG_INFO("PRODUCT_TYPE: SUM");
	} else if (PRODUCT_TYPE == HAP) {
		NRF_LOG_INFO("PRODUCT_TYPE: HAP");
	} else if (PRODUCT_TYPE == BATTERY_TEST) {
		NRF_LOG_INFO("PRODUCT_TYPE: BATTERY_TEST");
	} else if (PRODUCT_TYPE == WAIT_TIME_TEST) {
		NRF_LOG_INFO("PRODUCT_TYPE: WAIT_TIME_TEST");
	} else if (PRODUCT_TYPE == CUSTOM) {
		NRF_LOG_INFO("PRODUCT_TYPE: CUSTOM");
	} else {
		NRF_LOG_WARNING("** WARNING: PRODUCT_TYPE: UNKNOWN");
	}


	// Setup general stuff for the board
    bsp_board_leds_init();
	// Turn OFF LED's
    nrf_gpio_cfg_output(SAMPLE_LED);
	nrf_gpio_pin_clear(SAMPLE_LED);	// Enable HIGH, Turn OFF LED
	nrf_gpio_cfg_output(STATUS_LED);
	nrf_gpio_pin_clear(STATUS_LED);	// Enable HIGH, Turn OFF LED
    // ADP, turn OFF all power: start with power off when entering loop.
	NRF_LOG_DEBUG("Starting with ADP Power OFF... LOW");
    nrf_gpio_cfg_output(ADP1_PIN);
	nrf_gpio_pin_clear(ADP1_PIN);	// Enable HIGH
    nrf_gpio_cfg_output(ADP2_PIN);
	nrf_gpio_pin_clear(ADP2_PIN);	// Enable HIGH
//	// ADC setup
    saadc_init();

	// BLE
    bool erase_bonds;
    timers_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    // Watchdog Timer
    wdt_init();
//    wdt_init();


//    // FOR TESTING, REMOVE LATER
//    nrf_gpio_cfg_output(ADP1_PIN);
//	nrf_gpio_pin_clear(ADP1_PIN);	// Enable HIGH
////	nrf_gpio_pin_set(ADP1_PIN);	// Enable HIGH
//    nrf_gpio_cfg_output(ADP2_PIN);
//	nrf_gpio_pin_clear(ADP2_PIN);	// Enable HIGH
////	nrf_gpio_pin_set(ADP2_PIN);	// Enable HIGH
//	while (1) power_manage();



	// Main section that uses sensors
//	err_code = test_all(components_used);
//	err_code = test_all();

    // Save some values
    sd_ble_gap_addr_get(&ble_gap_address);
	// Other info user cares about
	NRF_LOG_INFO("ble_gap_address = %02X:%02X:%02X:%02X:%02X:%02X", ble_gap_address.addr[5], ble_gap_address.addr[4], ble_gap_address.addr[3], ble_gap_address.addr[2], ble_gap_address.addr[1], ble_gap_address.addr[0]);
	NRF_LOG_INFO("on_logging = %d", on_logging);
	NRF_LOG_INFO("log_interval = %d", log_interval);
	NRF_LOG_INFO("WDT_TIMEOUT_MEAS: %d", WDT_TIMEOUT_MEAS);
	NRF_LOG_DEBUG("plantower_startup_wait_ms = %d", plantower_startup_wait_ms);
	NRF_LOG_DEBUG("SPEC_CO_STARTUP_WAIT_TIME = %d", SPEC_CO_STARTUP_WAIT_TIME);
	NRF_LOG_DEBUG("FUEL_PERCENT_THRESHOLD = %d", FUEL_PERCENT_THRESHOLD);
	NRF_LOG_DEBUG("DEVICE_NAME = %s", DEVICE_NAME);
//	NRF_LOG_DEBUG("NUM_SAMPLES_PER_ON_CYCLE = %d", NUM_SAMPLES_PER_ON_CYCLE);
//	NRF_LOG_DEBUG("WAIT_BETWEEN_SAMPLES = %d", WAIT_BETWEEN_SAMPLES);
	NRF_LOG_DEBUG("SETTING_TIME_MANUALLY = %d", SETTING_TIME_MANUALLY);
	NRF_LOG_DEBUG("SD_FAIL_SHUTDOWN = %d", SD_FAIL_SHUTDOWN);
	NRF_LOG_FLUSH();


	// Make new Config file, or Read in the values
	sd_power_on();
	sd_init();
	sd_mount();

	// Option to delete the initial values, so it doesn't keep overwriting the FW values
	if (RESET_VALUES_FILE) {
		NRF_LOG_WARNING("Deleting %s", VALUES_FILE_NAME);
		ff_result = f_unlink(VALUES_FILE_NAME);
		if (ff_result != FR_OK) NRF_LOG_WARNING("f_unlink(VALUES_FILE_NAME) ff_result: %d", ff_result);
	}

	// Option to delete ALL sd card files, to start fresh
	if (DELETE_ALL_FILES) {
		NRF_LOG_WARNING("Deleting ALL Files..");
		ff_result = f_unlink(VALUES_FILE_NAME);
		if (ff_result != FR_OK) NRF_LOG_WARNING("f_unlink(VALUES_FILE_NAME) ff_result: %d", ff_result);
		ff_result = f_unlink(INFO_FILE_NAME);
		if (ff_result != FR_OK) NRF_LOG_WARNING("f_unlink(INFO_FILE_NAME) ff_result: %d", ff_result);
		ff_result = f_unlink(LOG_FILE_NAME);
		if (ff_result != FR_OK) NRF_LOG_WARNING("f_unlink(LOG_FILE_NAME) ff_result: %d", ff_result);
		ff_result = f_unlink(EXTRA_LOG_FILE_NAME);
		if (ff_result != FR_OK) NRF_LOG_WARNING("f_unlink(EXTRA_LOG_FILE_NAME) ff_result: %d", ff_result);
	}


	// The initial values stored on the SD card
	FRESULT ff_result_stat = f_stat(VALUES_FILE_NAME, NULL);
	if (ff_result_stat == FR_NO_FILE) {
		// File doesn't exist, so make a new one
		NRF_LOG_INFO("Writing new _values file..");
//		ff_result = sd_values_create();
		ff_result = sd_values_update();
		if (ff_result != FR_OK) NRF_LOG_WARNING("sd_values_create() ff_result: %d", ff_result);
//	    sd_close();
	} else if(ff_result_stat == FR_OK) {
		if (READING_VALUES_FILE) {
			NRF_LOG_INFO("Reading in previous config file..");
			ff_result = sd_values_read();
			if (ff_result != FR_OK) NRF_LOG_WARNING("sd_values_read() ff_result: %d", ff_result);
//		    sd_close();
		}
	} else {
		NRF_LOG_WARNING("ff_result_stat: %d", ff_result_stat);
	}

	// Make an info file that saves some settings for user to see later
	NRF_LOG_INFO("Creating info file..");
	ff_result = sd_info_create();
	if (ff_result != FR_OK) NRF_LOG_WARNING("sd_info_create() ff_result: %d", ff_result);

	// Clean up SD card stuff
//	sd_close();
	sd_uninit();
	sd_power_off();



//    printf("\r\nUART Start! (printf)\r\n");
    NRF_LOG_DEBUG("UART Start!");
    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
//    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_SLOW);	// Unknown error
    APP_ERROR_CHECK(err_code);


    // Start timer for main measurement loop
//	err_code = app_timer_start(meas_loop_timer, APP_TIMER_TICKS(log_interval), NULL);
//	APP_ERROR_CHECK(err_code);
//	test_all();

//	if (on_logging) {
////		restart_measurements(log_interval);
//		restart_measurements();
//	}
	if (!on_logging) {
	    NRF_LOG_DEBUG("---TESTING SENSORS..");
	    nrf_delay_ms(500);
		testing_sensors = true;
    	test_all();
		testing_sensors = false;
	}

    // Let the user read the startup messages
	nrf_delay_ms(INITIAL_MSG_WAIT);


	restart_measurements();

    // Enter main loop.
    for (;;)
    {
        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
        power_manage();
        // Restart the measurement timer so that the start times are sync'ed.  NOTE: needs to happen before checking meas_loop_wait_done b/c it can sleep after setting the flag
        if (start_adjustment_wait_done) {
//        	stop_measurements();
        	restart_measurements();
        	start_adjustment_wait_done = false;
        	adjusting_timer_start = false;	// reset flag
			NRF_LOG_DEBUG("adjusting_timer_start: %d", adjusting_timer_start);
        }
        if (meas_loop_wait_done) {
        	test_all();
        }
//        if (updating_end_byte) {
//			// Start the SD card
//		    sd_power_on();
//			sd_init();
//			sd_mount();
//			// Update the End Byte (file size) that will be sent
//			sd_open(ble_file_name, FA_READ);
////			// Start where we left off last time
////			f_lseek(&file, start_byte);
//			// Get the SD card size
//			end_byte = f_size(&file);
//			NRF_LOG_DEBUG("end_byte: %d", end_byte);
//
////			// Update the Config file
////			updating_values_file = true;
//////			f_close(&file);
//////			sd_values_update();
//
//        	// Uninitialize here, since don't want to interfere with another SDC operation
//		    sd_close();
//        	sd_uninit();
//        	sd_power_off();
//
//        	updating_end_byte = false;
//        }
        if (updating_values_file) {

			// Start the SD card
			sd_power_on();
			sd_init();
			sd_mount();

			// Update Config file
			sd_values_update();

			// Start the SD card
//		    sd_close();
			sd_uninit();
			sd_power_off();

			updating_values_file = false;	// reset flag
        }
        if (start_sending_sdc_data) {
			send_sdc_data();
        }
        if (!done_sending_sdc) {
//            done_reading_sdc = false;
//			done_sending_sdc = false;
			send_sdc_packets();
        }

        // Feed WDT before sleeping again, otherwise it will still reset in ~3.5 days
        nrf_drv_wdt_feed();
    }

}


/** @} */
