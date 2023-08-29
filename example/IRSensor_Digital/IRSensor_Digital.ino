#include "TwoWDRobot.h"
#define SIGNAL 2 //Arduino pin connected to IR sensor

infraredDigital sensor(SIGNAL);//Object to access the sensor method

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  bool IR_status = sensor.getInfraredDigitalSignal();//to read the sensor status
  Serial.print("IR Signal: ");Serial.println(IR_status);
}
