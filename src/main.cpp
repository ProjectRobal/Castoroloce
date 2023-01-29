#include <Arduino.h>

#include <BluetoothSerial.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#include <Motor.hpp>

#define NAME "Bober"

// set appropiet pins
#define MOTORA 27
#define MOTORB 26

// set appropiet pin
#define SERVO_PIN 13

#define INFO_LED 2

BluetoothSerial SerialBT;

Motor engine(MOTORA,MOTORB);

Servo s_wheel;

/*
  JSON:
    angel - an angel of steering wheel
    speed - an motor speed
    mode - a direction in which motor turns
*/

void read_serial()
{
  StaticJsonDocument<128> doc;

  DeserializationError err= deserializeJson(doc,SerialBT);

  if(err == DeserializationError::Ok)
  {
    uint16_t angel=doc["angel"].as<uint16_t>();
    uint16_t speed=doc["speed"].as<uint16_t>();
    uint8_t dir=doc["mode"].as<uint8_t>();

    Serial.print("Angel: ");
    Serial.println(angel);


    s_wheel.write(angel);

    engine.Update(static_cast<Motor::Modes>(dir),speed);
  }
  else
  {
    Serial.println("Error reading JSON");
    Serial.println(err.c_str());
    while(SerialBT.available()>0)
    {
      SerialBT.read();
    }
  }
}

hw_timer_t *info_blink = NULL;

void IRAM_ATTR blink()
{
  digitalWrite(INFO_LED,!digitalRead(INFO_LED));
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SerialBT.begin(NAME);

  Serial.println("Bluetooth has started");

  pinMode(INFO_LED,OUTPUT);

  // timer blinking
  info_blink=timerBegin(0,80,true);
  timerAttachInterrupt(info_blink, &blink, true);
  // call function each 500 ms
  timerAlarmWrite(info_blink, 500000, true);
  timerAlarmEnable(info_blink);

  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  s_wheel.setPeriodHertz(50);
  s_wheel.attach(SERVO_PIN,500,2500);
  s_wheel.write(90);

}

void loop() {
  // put your main code here, to run repeatedly:
  //when connection is lost, stop immediately
  if(!SerialBT.connected())
  {
    engine.Update(Motor::STOP,0);
    timerAlarmEnable(info_blink);
    s_wheel.write(45);
    return;
  }
  
  timerAlarmDisable(info_blink);
  digitalWrite(INFO_LED,LOW);

  if(SerialBT.available())
  {
    read_serial();
  }

}