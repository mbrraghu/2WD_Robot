#include "TwoWDRobot.h"
#define SIGNAL A0 //Arduino pin connected to IR sensor

infraredAnalog sensor(SIGNAL);//Object to access the sensor method

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  bool IR_value = sensor.getInfraredAnalogSignal();//to read the sensor analog value
  Serial.print("IR Signal: ");Serial.println(IR_value);
}
