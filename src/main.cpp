#include <Arduino.h>

#include <Robot.hpp>

// set appropiet pins
#define MOTORA (gpio_num_t)16
#define MOTORB (gpio_num_t)17

#define STARTER (gpio_num_t)2

#define SENSOR ADC2_CHANNEL_0

// set appropiet pin
#define SERVO_PIN (gpio_num_t)5


// for same or less the sensor is on black
#define SENSOR_ON_BLACK 280

Robot robot(MOTORA,MOTORB,SERVO_PIN,SENSOR);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //adc_gpio_init(ADC_UNIT_2,ADC_CHANNEL_0);

}

int adc_out;

void loop() {

  robot.loop();

  delay(500);

}