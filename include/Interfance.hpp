#pragma once

#include <Arduino.h>

#include <ArduinoJson.h>

#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define NAME "Bober"

/*
  JSON:
    angel - an angel of steering wheel
    speed - an motor speed
    mode - a direction in which motor turns
*/

void read_serial(Stream& serial)
{
  StaticJsonDocument<128> doc;

  DeserializationError err= deserializeJson(doc,serial);

  if(err == DeserializationError::Ok)
  {
    uint16_t angel=doc["angel"].as<uint16_t>();
    uint16_t speed=doc["speed"].as<uint16_t>();
    uint8_t dir=doc["mode"].as<uint8_t>();

    if(angel>180)
    {
      angel-=180;
    }

    //angel=map(angel,0,180,140,180);

    s_wheel.write(angel);

    Serial.print("Angel: ");
    Serial.println(angel);

    if(speed<500)
    {
      speed=0;
    }

    engine->Update(static_cast<Motor::Modes>(dir),speed);
  }
  else
  {
    Serial.println("Error reading JSON");
    Serial.println(err.c_str());
    while(serial.available()>0)
    {
      serial.read();
    }
  }
}
