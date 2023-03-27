#include <Arduino.h>

#include <Robot.hpp>

#define TIMESTAMP 0.001


#define SDA 23
#define SCL 22

// set appropiet pins
#define MOTORA (gpio_num_t)16
#define MOTORB (gpio_num_t)17

#define STARTER (gpio_num_t)39

#define SENSOR ADC2_CHANNEL_0

// set appropiet pin
#define SERVO_PIN (gpio_num_t)5

#define LED (gpio_num_t)2

// for same or less the sensor is on black
#define SENSOR_ON_BLACK 280

Robot *robot;

SemaphoreHandle_t xSemaphore=xSemaphoreCreateBinary();

hw_timer_t *loop_timer=NULL;

void IRAM_ATTR onTimer()
{
  xSemaphoreGive(xSemaphore);
}

void setup() {

  gpio_set_direction(MOTORA,GPIO_MODE_OUTPUT);
  gpio_set_direction(MOTORB,GPIO_MODE_OUTPUT);

  gpio_set_level(MOTORA,1);
  gpio_set_level(MOTORB,1);

  delay(50);

  gpio_set_direction(STARTER,GPIO_MODE_INPUT);

  gpio_set_direction(LED,GPIO_MODE_OUTPUT);

  xSemaphoreGive(xSemaphore);
  // put your setup code here, to run once:
  Serial.begin(115200);

  //adc_gpio_init(ADC_UNIT_2,ADC_CHANNEL_0);

  Wire.begin(SDA,SCL);

  loop_timer=timerBegin(0,80,true);
  timerAttachInterrupt(loop_timer,&onTimer,true);
  timerAlarmWrite(loop_timer,1000,true);
  timerAlarmEnable(loop_timer);

  xSemaphoreTake(xSemaphore,portMAX_DELAY);

  robot=new Robot(MOTORA,MOTORB,SERVO_PIN,SENSOR);


}

void loop() {


  if(!gpio_get_level(STARTER))
  {
    robot->stop();
    return;
  }
  
  if(xSemaphoreTake(xSemaphore,portMAX_DELAY))
  {
  timerAlarmDisable(loop_timer);
  robot->loop(TIMESTAMP);

  xSemaphoreGive(xSemaphore);
  timerAlarmEnable(loop_timer);
  }


}