#pragma once

#include <Arduino.h>

#include <MPU6050.h>

#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

#include <driver/adc.h>

#include <Motor.hpp>

#include <PID.hpp>


#include <Vectors.hpp>


#define CONFIG_FILE "/config.json"

// for same or less the sensor is on black
#define SENSOR_ON_BLACK 280

// for 50 Hz and 16 bit:

// 0.65 ms
#define SERVO_MIN 1700

// 2750 - front 

// 1 ms
#define SERVO_MAX 3200

// 0 for testing purposed
#define MAX_SPEED 256 //0 //65535


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

    int32_t z_axis;

    bool SoftStart;

    uint16_t ticks_elapsed;

    uint16_t max_speed;

    uint16_t black_threshold;

    uint16_t servo_min;

    uint16_t servo_max;


    bool on_track()
    {
        adc2_get_raw(_sensor,ADC_WIDTH_BIT_10,&adc_out);

        return black_threshold >= adc_out;
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

        ledcWrite(3,servo_min + (angel/180.0) * (servo_max-servo_min));
    }

    bool load_config_from_flash()
    {
        if(!SPIFFS.begin(true))
        {
            return false;
        }

        StaticJsonDocument<200> doc;

        fs::File file=SPIFFS.open(CONFIG_FILE);

        if(!file)
        {
            return false;    
        }        

        DeserializationError error = deserializeJson(doc, file);

        if(error)
        {
            return false;
        }

        default_config();

        float p=doc["P"];
        float i=doc["I"];
        float d=doc["D"];  

        if(doc.containsKey("max_speed"))
        {
            max_speed=doc["max_speed"];
        }

        if(doc.containsKey("servo_min"))
        {
            servo_min=doc["servo_min"];
        } 

        if(doc.containsKey("servo_max"))
        {
            servo_max=doc["servo_max"];
        }

        if(doc.containsKey("black_threshold"))
        {
            black_threshold=doc["black_threshold"];
        }

        regulator.setParams(p,i,d);

        file.close();

        return true;
    }

    void default_config()
    {
        regulator.setParams(1.0,0.0,0.0);

        max_speed=MAX_SPEED;

        black_threshold=SENSOR_ON_BLACK;

        servo_min=SERVO_MIN;

        servo_max=SERVO_MAX;
    }

    public:

    Robot(gpio_num_t mA,gpio_num_t mB,gpio_num_t servo,adc2_channel_t sensor)
    : _m(mA,mB),
    _servo(servo),
    _sensor(sensor),
    regulator(1.0,0.0,0.0)
    {
        SoftStart=true;
        ticks_elapsed=0;
        adc_out=0;
        width=0;
        z_axis=0;
        target_angel=0;

        if(!load_config_from_flash())
        {
            default_config();
        }

        Serial.println("PID params:");
        Serial.print("P: ");
        Serial.println(regulator.P());
        Serial.print("I: ");
        Serial.println(regulator.I());
        Serial.print("D: ");
        Serial.println(regulator.D());
        
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
            if(ticks_elapsed<=2)
            {
                _m.Update(Motor::FORWARD,max_speed/10);
            }
            else if((ticks_elapsed>2)&&(ticks_elapsed<=4))
            {
                _m.Update(Motor::FORWARD,max_speed/5);
            }
            else if((ticks_elapsed>4)&&(ticks_elapsed<=6))
            {
                _m.Update(Motor::FORWARD,max_speed/2);
            }
            else if(ticks_elapsed>6)
            {
                _m.Update(Motor::FORWARD,max_speed);
                SoftStart=false;
            }

            ++ticks_elapsed;
        }
        else
        {
            _m.Update(Motor::FORWARD,max_speed);
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
        Serial.print(" z axis: ");
        Serial.println(z_axis);
        Serial.println(" Angel: ");
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