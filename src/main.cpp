#include <Arduino.h>

#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#include <Motor.hpp>

#define NAME "Bober"

#define START_BYTE 0x11

// set appropiet pins
#define MOTORA 21
#define MOTORB 22

BluetoothSerial SerialBT;

Motor engine(MOTORA,MOTORB);

// every data frame:
// [ start_byte , direction , speed(2 bytes)]


uint8_t read_buffer[3];
volatile uint8_t counter=0;
volatile bool transmision_started=false;

void read_serial(uint8_t byte)
{
  if(byte==START_BYTE)
  {
    transmision_started=true;
    counter=0;
  }

  if(!transmision_started)
  {
    return;
  }

  read_buffer[++counter]=byte;
}

// get information from read_buffer
void decode()
{
  Motor::Modes direction=static_cast<Motor::Modes>(read_buffer[0]);

  uint16_t speed=0;

  memmove((uint8_t*)&speed,read_buffer+1,sizeof(uint16_t));

  engine.SetDirection(direction);

  engine.setSpeed(speed);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SerialBT.begin(NAME);

  Serial.println("Bluetooth has started");

}

void loop() {
  // put your main code here, to run repeatedly:
  //when connection is lost, stop immediately
  if(!SerialBT.connected())
  {
    engine.SetDirection(Motor::STOP);

    // try to connect again
    SerialBT.connect();
    return;
  }

  if(SerialBT.available())
  {
    if((transmision_started)&&(counter==3))
    {
      decode();
      transmision_started=false;
    }

    read_serial(SerialBT.read());
  }

}