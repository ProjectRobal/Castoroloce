#pragma once

#include <Arduino.h>

#include <MPU6050.h>

#include <driver/adc.h>

#include <Motor.hpp>

#include <PID.hpp>


#include <Vectors.hpp>


// for same or less the sensor is on black
#define SENSOR_ON_BLACK 280

// for 50 Hz and 16 bit:

// 0.65 ms
#define SERVO_MIN 1700

// 2750 - front 

// 1 ms
#define SERVO_MAX 3200


class Robot
{
    private:

    Motor _m;

    gpio_num_t _servo;

    adc2_channel_t _sensor;

    int adc_out;

    PID<float> regulator;

    Vec3i gyroscope;
    Vec3i acceleration;

    MPU6050 mpu;

    int8_t angel;

    uint32_t width;

    bool on_track()
    {
        adc2_get_raw(_sensor,ADC_WIDTH_BIT_10,&adc_out);

        return SENSOR_ON_BLACK >= adc_out;
    }

    void read_mpu()
    {
        mpu.getMotion6(&acceleration.x,&acceleration.y,&acceleration.z,
        &gyroscope.x,&gyroscope.y,&gyroscope.z);
    }

    // from -90 to 90
    void set_angel(int8_t angel)
    {
        angel+=90;

        ledcWrite(3,SERVO_MIN + ((float)angel/180.0) * (SERVO_MAX-SERVO_MIN));
    }


    public:

    Robot(gpio_num_t mA,gpio_num_t mB,gpio_num_t servo,adc2_channel_t sensor)
    : _m(mA,mB),
    _servo(servo),
    _sensor(sensor),
    regulator(-0.01,0,0)
    {
        adc_out=0;
        width=0;

        regulator.setMax(90.0);
        regulator.setMin(-90.0);
        
        //servo, channel 3 , 50 Hz , 16 bits resolution
        ledcSetup(3,50,16);

        ledcAttachPin(servo,3);

        mpu.initialize();

        mpu.setFullScaleAccelRange(MPU6050_IMU::ACCEL_FS::MPU6050_ACCEL_FS_8);
        mpu.setFullScaleGyroRange(MPU6050_IMU::GYRO_FS::MPU6050_GYRO_FS_2000);

        set_angel(0);

        regulator.setMax(90);
        regulator.setMin(-90);
    }

    // main loop, dt - timestamp
    void loop(double dt)
    {
        if(on_track())
        {
            Serial.println("On track!");
            return;
        }

        if(!mpu.testConnection())
        {
            return;
        }

        int16_t z=mpu.getRotationZ();

        if(abs(z)<45)
        {
            return;
        }

        angel=regulator.step(z,dt);

        set_angel(angel);
        

        Serial.println("Gyroscope:");
        Serial.print("z: ");
        Serial.print(z);
        Serial.println();
        Serial.println("Angel: ");
        Serial.print(angel);
        Serial.println();

    }

};