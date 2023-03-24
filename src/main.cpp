#include <Arduino.h>

#include <ESP32Servo.h>

#include <Motor.hpp>

// set appropiet pins
#define MOTORA (gpio_num_t)16
#define MOTORB (gpio_num_t)17

#define STARTER (gpio_num_t)2

#define SENSOR (gpio_num_t)4

// set appropiet pin
#define SERVO_PIN (gpio_num_t)5


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

}

void loop() {
  

}