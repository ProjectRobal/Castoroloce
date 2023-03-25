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

// 0 for testing purposed
#define MAX_SPEED 0 //256 //65535


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

    float angel;

    float target_angel;

    uint32_t width;

    uint32_t z_axis;

    bool SoftStart;

    uint16_t ticks_elapsed;

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
    void set_angel(float angel)
    {
        angel+=90;

        ledcWrite(3,SERVO_MIN + (angel/180.0) * (SERVO_MAX-SERVO_MIN));
    }


    public:

    Robot(gpio_num_t mA,gpio_num_t mB,gpio_num_t servo,adc2_channel_t sensor)
    : _m(mA,mB),
    _servo(servo),
    _sensor(sensor),
    regulator(-0.01,0,0)
    {
        SoftStart=true;
        ticks_elapsed=0;
        adc_out=0;
        width=0;
        z_axis=0;
        target_angel=0;

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

        if(SoftStart)
        {
            if(ticks_elapsed<5)
            {
                _m.Update(Motor::FORWARD,MAX_SPEED/10);
            }
            else if((ticks_elapsed>10)&&(ticks_elapsed<=15))
            {
                _m.Update(Motor::FORWARD,MAX_SPEED/5);
            }
            else if((ticks_elapsed>15)&&(ticks_elapsed<=20))
            {
                _m.Update(Motor::FORWARD,MAX_SPEED/2);
            }
            else if(ticks_elapsed>20)
            {
                _m.Update(Motor::FORWARD,MAX_SPEED);
                SoftStart=false;
            }

            ++ticks_elapsed;
        }
        else
        {
            _m.Update(Motor::FORWARD,MAX_SPEED);
        }

        int32_t z=mpu.getRotationZ();

        // filter some noises
        if(abs(z)<45)
        {
            return;
        }

        z_axis+=z*dt;

        angel=regulator.step(z_axis,dt);

        set_angel(target_angel+angel);
        
        Serial.println("Gyroscope:");
        Serial.print("z: ");
        Serial.print(z);
        Serial.print("z axis: ");
        Serial.println(z_axis);
        Serial.println("Angel: ");
        Serial.print(angel);
        Serial.println();

    }

    void stop()
    {
        _m.stop();
    }

    ~Robot()
    {
        stop();
    }

};