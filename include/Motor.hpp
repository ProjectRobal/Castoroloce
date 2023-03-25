#ifndef MOTOR_HH
#define MOTOR_HH

#include <Arduino.h>

#define FREQUENCY 100

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

    gpio_num_t motor_A;
    gpio_num_t motor_B;

    uint16_t speed;

    Modes mode;

    public:

    Motor(gpio_num_t mA,gpio_num_t mB)
    : motor_A(mA),
    motor_B(mB),
    speed(0)
    {
        gpio_set_direction(motor_A,GPIO_MODE_OUTPUT);
        gpio_set_direction(motor_B,GPIO_MODE_OUTPUT);

        gpio_set_level(motor_A,1);
        gpio_set_level(motor_B,1);

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

    void Update(Modes _m,const uint16_t _speed)
    {
        speed=_speed;
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

    void stop()
    {
        Update(STOP,0);
    }


};


#endif