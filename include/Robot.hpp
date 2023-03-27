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

    float angel_offset;

    uint32_t width;

    double z_axis;

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

        if(doc.containsKey("offset"))
        {
            angel_offset=doc["offset"];
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

        angel_offset=0.0;
    }

    char buff[32];
    uint8_t cmd_iter;

    char cmd;

    int32_t gyro_error;

    // bits:
    // 0 - command readed
    // 1 - number readed
    uint8_t cmd_control;

    void execute_cmd()
    {
        if(!(cmd_control&(1<<1)))
        {
            return;
        }

        cmd_control=0;
        cmd_iter=0;

        float var=atof(buff);

        switch(cmd)
        {
            case 'p':
            case 'P':
                regulator.setP(var);
            break;

            case 'I':
            case 'i':
                regulator.setI(var);
            break;

            case 'd':
            case 'D':
                regulator.setD(var);
            break;
        }

        memset(buff,0,32);
    }


    public:

    Robot(gpio_num_t mA,gpio_num_t mB,gpio_num_t servo,adc2_channel_t sensor)
    : _m(mA,mB),
    _servo(servo),
    _sensor(sensor),
    regulator(1.0,0.0,0.0)
    {
        cmd_control=0;
        cmd_iter=0;
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

        #ifdef DEBUG

        Serial.println("PID params:");
        Serial.print("P: ");
        Serial.println(regulator.P());
        Serial.print("I: ");
        Serial.println(regulator.I());
        Serial.print("D: ");
        Serial.println(regulator.D());

        #endif
        
        //servo, channel 3 , 50 Hz , 16 bits resolution
        ledcSetup(3,50,16);

        ledcAttachPin(servo,3);

        mpu.initialize();

        mpu.setFullScaleAccelRange(MPU6050_IMU::ACCEL_FS::MPU6050_ACCEL_FS_8);
        mpu.setFullScaleGyroRange(MPU6050_IMU::GYRO_FS::MPU6050_GYRO_FS_500);
        mpu.setFIFOEnabled(true);
        mpu.CalibrateGyro();

        set_angel(angel_offset);

        regulator.setMax(90);
        regulator.setMin(-90);
        
        mpu_error();

        Serial.print("Gyro error: ");
        Serial.println(gyro_error);
    }

    void mpu_error(uint32_t n=30)
    {

        for(uint8_t i=0;i<n;++i)
        {
            gyro_error+=mpu.getRotationZ();
        }

        gyro_error/=n;
    }

    // main loop, dt - timestamp
    void loop(double dt)
    {

        if(SoftStart)
        {
            if(ticks_elapsed==1)
            {
                _m.Update(Motor::FORWARD,max_speed/3);
            }
            else if(ticks_elapsed==2)
            {
                _m.Update(Motor::FORWARD,max_speed/2);
            }
            else if(ticks_elapsed>=3)
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

        int32_t z=mpu.getRotationZ()-gyro_error;

        // filter some noises
        if(abs(z)<=2)
        {
            return;
        }

        z_axis+=(z/32767.0)*500.0*dt;

        angel=regulator.step(z_axis,dt);

        set_angel(target_angel+angel+angel_offset);

        #ifdef DEBUG
        
        Serial.println("Gyroscope:");
        Serial.print("z: ");
        Serial.print(z);
        Serial.print(" z axis: ");
        Serial.println(z_axis);
        Serial.println(" Angel: ");
        Serial.print(angel+angel_offset);
        Serial.println();

        #endif
    }

    void stop()
    {
        _m.stop();
    }

    void terminal(char c)
    {
        if((c=='\n')||(c=='\r'))
        {
            cmd_control|=(1<<1);
        }
        else if(!(cmd_control&(1<<0)))
        {
            cmd=c;
            cmd_control|=(1<<0);
        }
        else
        {
            buff[cmd_iter++]=c;

            if(cmd_iter>=32)
            {
                cmd_control|=(1<<1);
            }
        }

        execute_cmd();
    }

    ~Robot()
    {
        stop();
    }

};