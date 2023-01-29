#ifndef MOTOR_HH
#define MOTOR_HH

#include <Arduino.h>

#define FREQUENCY 10000

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

        ledcSetup(1,FREQUENCY,16);
        ledcSetup(2,FREQUENCY,16);

        ledcAttachPin(motor_A,1);
        ledcAttachPin(motor_B,2);

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

            ledcWrite(1,65535);
            ledcWrite(2,65535-speed);

            break;

            case BACKWARD:

            ledcWrite(2,65535);
            ledcWrite(1,65535-speed);

            break;

            case STOP:
            default:
            ledcWrite(2,65535);
            ledcWrite(1,65535);
        }
        
    }

    uint16_t Speed() const
    {
        return speed;
    }


};


#endif