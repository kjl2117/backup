#include "Dht22.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_timer.h"

/// percentage of humidity
static int dht_humidity;
/// celsius
static int dht_temperature;
/// pin to read the sensor info on
//DigitalInOut _pin;
static uint8_t dht_pin;
/// times startup (must settle for at least a second)
#define DHT_STARTUP_SETTLING_TIME	2000	// ms
/** Timer variables **/
#define TIMER_CHANNEL_NUM	NRF_TIMER_CC_CHANNEL0
#define TIMER_NUM			1	// Which Timer to use: TIMER0 reserved for SoftDevice, maybe change sdk_config.h
static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(TIMER_NUM);

// Dummy function for argument, Taken from peripheral/saadc
void dht_timer_handler(nrf_timer_event_t event_type, void * p_context)
{

}


void dht_init(uint8_t pin_num) {
    // Set creation time so we can make
    // sure we pause at least 1 second for
    // startup.
//    _timer.start();	// FIX THIS LATER

    dht_temperature = 0;
    dht_humidity = 0;
    dht_pin = pin_num;
	NRF_LOG_INFO("--T1");

    // For Timer
    ret_code_t err_code;
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, dht_timer_handler);
	NRF_LOG_INFO("--T2");
    APP_ERROR_CHECK(err_code);
//    nrf_drv_timer_enable(&m_timer);


}

// Needed when done, to get rid of the timer
void dht_uninit() {

	nrf_drv_timer_uninit(&m_timer);
}
//void dht_timeout_handler(void * p_context);	// blank function to use as an argument


int dht_read() {

//    // NRF_LOG setup
//    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
//    NRF_LOG_DEFAULT_BACKENDS_INIT();


//    uint32_t err_code;
//    err_code = app_simple_timer_init();
//    APP_ERROR_CHECK(err_code);

    // BUFFER TO RECEIVE
    uint8_t bits[5];
    uint8_t cnt = 7;
    uint8_t idx = 0;

    // EMPTY BUFFER
    for (int i=0; i< 5; i++) bits[i] = 0;

//    // Verify sensor settled after boot	// FIX THIS LATER
//    while(_timer.read_ms() < 1500) {}
//    _timer.stop();
//    nrf_delay_ms(1500);	// FIX THIS LATER, should use timer instead
//    uint32_t ticks_per_ms = nrf_drv_timer_ms_to_ticks(&m_timer, 1);
    uint32_t ticks_per_us = nrf_drv_timer_us_to_ticks(&m_timer, 1);

//	nrf_drv_timer_enable(&m_timer);	// start timer before the loop
//    while(nrf_drv_timer_capture(&m_timer, TIMER_CHANNEL_NUM)/ticks_per_ms < DHT_STARTUP_SETTLING_TIME) {}	//wait until settled
//	NRF_LOG_INFO("DHT TIMER INITIALLY SETTLED");
//    nrf_drv_timer_disable(&m_timer);	// disable the timer while it's not needed

    uint32_t bit_time_elapsed = 0;



    // Notify it we are ready to read
    nrf_gpio_cfg_output(dht_pin);
	nrf_gpio_pin_clear(dht_pin);
	nrf_delay_ms(18);
	nrf_gpio_pin_set(dht_pin);
	nrf_delay_us(40);
	nrf_gpio_cfg_input(dht_pin, NRF_GPIO_PIN_PULLUP);

    // ACKNOWLEDGE or TIMEOUT
    unsigned int loopCnt = 10000;
    while(nrf_gpio_pin_read(dht_pin) == 0)
        if (loopCnt-- == 0) return DHTLIB_ERROR_TIMEOUT;
//	NRF_LOG_INFO("--1");

    loopCnt = 10000;
    while(nrf_gpio_pin_read(dht_pin) == 1)
        if (loopCnt-- == 0) return DHTLIB_ERROR_TIMEOUT;
//	NRF_LOG_INFO("--2");


    // READ OUTPUT - 40 BITS => 5 BYTES or TIMEOUT
//	nrf_drv_timer_enable(&m_timer);	// start timer before the loop
    for (int i=0; i<40; i++)
    {
        loopCnt = 10000;
        while(nrf_gpio_pin_read(dht_pin) == 0)
            if (loopCnt-- == 0) return DHTLIB_ERROR_TIMEOUT;
//    	NRF_LOG_INFO("--3");

////        Timer t;
////        t. start();
        nrf_drv_timer_enable(&m_timer);	// start timer, needed to measure 1 or 0 //// CAREFUL! NEED TO MAKE SURE TO RESET EACH LOOP
//		nrf_drv_timer_clear(&m_timer);	// reset timer


        loopCnt = 10000;
        while(nrf_gpio_pin_read(dht_pin) == 1)
            if (loopCnt-- == 0) return DHTLIB_ERROR_TIMEOUT;
//    	NRF_LOG_INFO("--4");

//        if (t.read_us() > 40) bits[idx] |= (1 << cnt);
//        if (1) bits[idx] |= (1 << cnt);
//        if (nrf_drv_timer_capture(&m_timer, TIMER_CHANNEL_NUM)/ticks_per_us > 40) bits[idx] |= (1 << cnt);
        bit_time_elapsed = nrf_drv_timer_capture(&m_timer, TIMER_CHANNEL_NUM)/ticks_per_us;
        nrf_drv_timer_disable(&m_timer);	// start timer, needed to measure 1 or 0 //// CAREFUL! NEED TO MAKE SURE TO RESET EACH LOOP
        if (bit_time_elapsed > 40) bits[idx] |= (1 << cnt);
//		if (i == 10) return bit_time_elapsed;	// FOR TESTING
        if (cnt == 0)   // next byte?
        {
            cnt = 7;    // restart at MSB
            idx++;      // next byte!
        }
        else cnt--;
    }

//    // WRITE TO RIGHT VARS - DHT22
//    // as bits[1] and bits[3] are allways zero they are omitted in formulas.
//    dht_humidity    = bits[0];
//    dht_temperature = bits[2];
//
//    uint8_t sum = bits[0] + bits[2];
//
//    if (bits[4] != sum) return DHTLIB_ERROR_CHECKSUM;


//	NRF_LOG_INFO("bits: %d", bits);
//    return bit_time_elapsed;

	nrf_drv_timer_disable(&m_timer);	// done with the Timer
//	nrf_drv_timer_uninit(&m_timer);	// TODO: CHECK THIS: do not uninit, may need it on


    uint8_t dht22_check_sum;
    dht22_check_sum = bits[0] + bits[1] + bits[2] + bits[3];
    //printf("dht22 check sum: %d",dht22_check_sum);
    dht22_check_sum= dht22_check_sum%256;
    //printf("dht22 check sum: %d",dht22_check_sum);

    if (dht22_check_sum == bits[4]) {
        dht_humidity = bits[0]*256 + bits[1];

        dht_temperature = bits[2]*256 + bits[3];
        //printf("all good \n");
        return DHTLIB_OK;
    }
    return DHTLIB_ERROR_CHECKSUM;

//	return 0;
}

float dht_getFahrenheit() {
    return((dht_temperature * 1.8) + 32*10);
}

int dht_getCelsius() {
    return(dht_temperature);
}

int dht_getHumidity() {
    return(dht_humidity);
}
