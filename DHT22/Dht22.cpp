#include "Dht22.h"

Dht22::Dht22(PinName const &p) : _pin(p) {
    // Set creation time so we can make 
    // sure we pause at least 1 second for 
    // startup.
    _timer.start();
    
    _temperature = 0;
    _humidity = 0;
}

int Dht22::read()
{
    // BUFFER TO RECEIVE
    uint8_t bits[5];
    uint8_t cnt = 7;
    uint8_t idx = 0;

    // EMPTY BUFFER
    for (int i=0; i< 5; i++) bits[i] = 0;
    
    // Verify sensor settled after boot
    while(_timer.read_ms() < 1500) {}
    _timer.stop();

    // Notify it we are ready to read
    _pin.output();
    _pin = 0;
    wait_ms(18);
    _pin = 1;
    wait_us(40);
    _pin.input();

    // ACKNOWLEDGE or TIMEOUT
    unsigned int loopCnt = 10000;
    while(_pin == 0)
        if (loopCnt-- == 0) return DHTLIB_ERROR_TIMEOUT;

    loopCnt = 10000;
    while(_pin == 1)
        if (loopCnt-- == 0) return DHTLIB_ERROR_TIMEOUT;

    // READ OUTPUT - 40 BITS => 5 BYTES or TIMEOUT
    for (int i=0; i<40; i++)
    {
        loopCnt = 10000;
        while(_pin == 0)
            if (loopCnt-- == 0) return DHTLIB_ERROR_TIMEOUT;

        //unsigned long t = micros();
        Timer t;
        t. start();

        loopCnt = 10000;
        while(_pin == 1)
            if (loopCnt-- == 0) return DHTLIB_ERROR_TIMEOUT;

        if (t.read_us() > 40) bits[idx] |= (1 << cnt);
        if (cnt == 0)   // next byte?
        {
            cnt = 7;    // restart at MSB
            idx++;      // next byte!
        }
        else cnt--;
    }

//    // WRITE TO RIGHT VARS - DHT22
//    // as bits[1] and bits[3] are allways zero they are omitted in formulas.
//    _humidity    = bits[0]; 
//    _temperature = bits[2]; 
//
//    uint8_t sum = bits[0] + bits[2];  
//
//    if (bits[4] != sum) return DHTLIB_ERROR_CHECKSUM;
    
    
    uint8_t dht22_check_sum;
    dht22_check_sum = bits[0] + bits[1] + bits[2] + bits[3];
    //printf("dht22 check sum: %d",dht22_check_sum);
    dht22_check_sum= dht22_check_sum%256;
    //printf("dht22 check sum: %d",dht22_check_sum);
    
    if (dht22_check_sum == bits[4]) {
        _humidity = bits[0]*256 + bits[1];
        
        _temperature = bits[2]*256 + bits[3];
        //printf("all good \n");
        return DHTLIB_OK;
    }
    return DHTLIB_ERROR_CHECKSUM;
}

float Dht22::getFahrenheit() {
    return((_temperature * 1.8) + 32);
}

int Dht22::getCelsius() {
    return(_temperature);
}

int Dht22::getHumidity() {
    return(_humidity);
}
