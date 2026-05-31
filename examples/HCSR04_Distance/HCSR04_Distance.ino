#include "TwoWDRobot.h"
#define TRIG 2 //Arduino pin connected to HC-SR04 trigger pin
#define ECHO 3 //Arduino pin connected to HC-SR04 echo pin

ultraSound HCSR04(TRIG, ECHO);//ultrasound sensor object

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  float distance = HCSR04.getUltrasoundDistance(); //get the distance to from the HC-SR04
  Serial.print("Distance: ");Serial.println(distance);
}