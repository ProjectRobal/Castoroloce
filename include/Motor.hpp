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

    Modes mode;

    public:

    Motor(uint8_t mA,uint8_t mB)
    : motor_A(mA),
    motor_B(mB),
    speed(0)
    {
        pinMode(motor_A,OUTPUT);
        pinMode(motor_B,OUTPUT);

        Update(STOP,0);
    }

    Modes Direction() const
    {
        return mode;
    }

    void Update(const Modes& _m,const uint16_t _speed)
    {
        speed=0;
        mode=_m;
        switch(mode)
        {
            case FORWARD:

            digitalWrite(motor_A,HIGH);
            analogWrite(motor_B,65535-speed);

            break;

            case BACKWARD:

            analogWrite(motor_A,65535-speed);
            digitalWrite(motor_B,HIGH);

            break;

            case STOP:
            default:
            digitalWrite(motor_A,HIGH);
            digitalWrite(motor_B,HIGH);
        }
        
    }

    uint16_t Speed() const
    {
        return speed;
    }


};


#endif