#include "TwoWDRobot.h"
#define LEFT_IR 7 // Arduino pin connected to HC-SR04 trigger pin.
#define RIGHT_IR 8 // Arduino pin connected to HC-SR04 echo pin.

// Object to access the methods to run cliff detection robot.
// It takes inputs for two IR sensors pins and MDD3A motor driver pins.
cliffDetectionRobot myRobot(LEFT_IR, RIGHT_IR,5,9,10,11, PWM_PWM);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  // Method takes delay time to deviate from the cliff and sensor type.
  // if sensors are active high, arguments will be 'true' by default else it should be 'false'.
  // By default PWM values for motors are set to 50 each, user can change as per their needs.
  myRobot.runCliffRobot(ACTIVE_LOW, 500);
}