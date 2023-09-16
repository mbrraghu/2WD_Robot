#include "TwoWDRobot.h"
#define LEFT_IR 2 // Arduino pin connected to left IR signal pin.
#define RIGHT_IR 3 // Arduino pin connected to right IR signal pin.

// Object to access the methods to run cliff detection robot.
// It takes inputs for two IR sensors pins and MDD3A motor driver pins.
cliffDetectionRobot myRobot(LEFT_IR, RIGHT_IR,5,6,9,10, PWM_PWM);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  // Method takes delay time to deviate from the cliff and sensor type.
  // if sensors are active high, arguments will be 'true' by default else it should be 'false'.
  // By default PWM values for motors are set to 50 each, user can change as per their needs.
  myRobot.runCliffRobot(ACTIVE_LOW, 1500,30,30);
}