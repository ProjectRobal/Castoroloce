#pragma once

#include <Arduino.h>

#include <Motor.hpp>

#include <driver/adc.h>


// for same or less the sensor is on black
#define SENSOR_ON_BLACK 280

class Robot
{
    private:

    Motor _m;

    gpio_num_t _servo;

    adc2_channel_t _sensor;

    int adc_out;

    bool on_track()
    {
        adc2_get_raw(_sensor,ADC_WIDTH_BIT_10,&adc_out);

        return SENSOR_ON_BLACK >= adc_out;
    }

    public:

    Robot(gpio_num_t mA,gpio_num_t mB,gpio_num_t servo,adc2_channel_t sensor)
    : _m(mA,mB),
    _servo(servo),
    _sensor(sensor)
    {
        adc_out=0;
        //servo, channel 3 , 50 Hz , 16 bits resolution
        ledcSetup(3,50,16);

    }

    // main loop
    void loop()
    {

    }

};