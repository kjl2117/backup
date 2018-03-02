#ifndef DHT22_H
#define DHT22_H

//#include "mbed.h"
//#include "nrf_gpio.h"
#include "nrf.h"

#define DHTLIB_OK                0
#define DHTLIB_ERROR_CHECKSUM   -1
#define DHTLIB_ERROR_TIMEOUT    -2


/** Construct the sensor object.
 *
 * @param pin PinName for the sensor pin.
 */
void dht_init(uint8_t dht_pin);

/** Release the sensor object (mostly to uninit timer)
 *
 */
void dht_uninit();

/** Update the humidity and temp from the sensor.
 *
 * @returns
 *   0 on success, otherwise error.
 */
int dht_read();

/** Get the temp(f) from the saved object.
 *
 * @returns
 *   Fahrenheit float
 */
float dht_getFahrenheit();

/** Get the temp(c) from the saved object.
 *
 * @returns
 *   Celsius int
 */
int dht_getCelsius();

/** Get the humidity from the saved object.
 *
 * @returns
 *   Humidity percent int
 */
int dht_getHumidity();

///// percentage of humidity
//static int dht_humidity;
///// celsius
//static int dht_temperature;
///// pin to read the sensor info on
////DigitalInOut _pin;
///// times startup (must settle for at least a second)
////Timer _timer;

#endif
