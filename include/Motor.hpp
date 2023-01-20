#ifndef MOTOR_HH
#define MOTOR_HH

#include <Arduino.h>

class Motor
{
    public:

    enum Modes
    {
        FORWARD=0,
        BACKWARD=1,
        STOP=2
    };  

    protected:

    uint8_t motor_A;
    uint8_t motor_B;

    uint16_t speed;
    uint8_t* curr_motor;

    Modes mode;

    public:

    Motor(uint8_t mA,uint8_t mB)
    : motor_A(mA),
    motor_B(mB),
    speed(0)
    {
        pinMode(motor_A,OUTPUT);
        pinMode(motor_B,OUTPUT);
        curr_motor=NULL;

        SetDirection(STOP);
    }

    Modes Direction() const
    {
        return mode;
    }

    void SetDirection(const Modes& _m)
    {
        if(_m==mode)
        {
            return;
        }
        speed=0;
        mode=_m;
        switch(mode)
        {
            case FORWARD:

            digitalWrite(motor_A,HIGH);
            analogWrite(motor_B,65535);
            curr_motor=&motor_B;

            break;

            case BACKWARD:

            analogWrite(motor_A,65535);
            digitalWrite(motor_B,HIGH);
            curr_motor=&motor_A;

            break;

            case STOP:
            default:
            digitalWrite(motor_A,HIGH);
            digitalWrite(motor_B,HIGH);
        }
        
    }

    void setSpeed(const uint16_t& _speed)
    {
        if(!curr_motor)
        {
            return;
        }
        speed=_speed;
        analogWrite(*curr_motor,65535-speed);
    }

    uint16_t Speed() const
    {
        return speed;
    }


};


#endif