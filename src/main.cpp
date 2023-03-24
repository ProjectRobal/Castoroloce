#include <Arduino.h>

#include <ESP32Servo.h>

#include <Motor.hpp>

#include <driver/adc.h>

// set appropiet pins
#define MOTORA (gpio_num_t)16
#define MOTORB (gpio_num_t)17

#define STARTER (gpio_num_t)2

#define SENSOR ADC2_CHANNEL_0

// set appropiet pin
#define SERVO_PIN (gpio_num_t)5


// for same or less the sensor is on black
#define SENSOR_ON_BLACK 280

bool on_track()
{
  int adc_out=0;

  adc2_get_raw(ADC2_CHANNEL_0,ADC_WIDTH_BIT_10,&adc_out);

  return SENSOR_ON_BLACK >= adc_out;
}


Motor *engine;

Servo s_wheel;


void setup() {
  engine=new Motor(MOTORA,MOTORB);
  // put your setup code here, to run once:
  Serial.begin(115200);

  Serial.println("Bluetooth has started");

  s_wheel.setPeriodHertz(50);
  s_wheel.attach(SERVO_PIN,500,2500);
  s_wheel.write(90);

  //adc_gpio_init(ADC_UNIT_2,ADC_CHANNEL_0);

}

int adc_out;

void loop() {

  Serial.println(on_track());

  delay(500);

}