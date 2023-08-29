#include "TwoWDRobot.h"

//PWM_PWM: both the terminals of the motors are connected to PWM pins
//PWM_DIR: one of the terminals of the motor are connected to PWM pin
MDD3A driver(PWM_PWM,5,6,9,10);//objects to access the methods

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  //first two arguments represents the PWM values to control the speed of the motor
  //third argument is the delay to excite the motors
  driver.motorDrive(50,50,1000);//2WD robot forward movement

  //use negative sign before the PWM value to rotate the motor in opposite direction
  driver.motorDrive(50,-50,1000);//2WD robot; inplace right turn
}