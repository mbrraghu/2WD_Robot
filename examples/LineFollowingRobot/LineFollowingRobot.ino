#include "TwoWDRobot.h"

// IR sensor array pins
int IRArrayPins[5] = {2, 3, 4, 6, 7};

// Object to access the methods to run line following robot using IR line sensor array.
// It takes inputs for control pins of MDD3A motor driver,IR array sensor signal pins

lineFollowRobot myRobot(IRArrayPins,5,9,10,11,PWM_PWM);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  // Method to run the line following robot.
  // both the motor speeds are set to 50 PWM values by default. User can key in the required speeds.
  myRobot.runLineFollowRobot(ACTIVE_HIGH);
}