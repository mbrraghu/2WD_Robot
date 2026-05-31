#include "TwoWDRobot.h"

// ultrasound sensor trigger and echo pins
int us_front_pins[] = {2, 3};
int us_side_pins[] = {4, 6};

// Object to access the methods to run wall following robot two ultrasound sensors
// It takes inputs for control pins of MDD3A motor driver, ultrasound sensor control pins
// Input concerned to the operation of robot like left or right wall following robot.
wallFollowRobot myRobot(LEFT_FOLLOW, us_front_pins, us_side_pins, 5, 9, 10, 11, PWM_PWM);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  // Method to run the wall following robot.
  // both the motor speeds are set to 20 PWM values by default. User can key in the required speeds.
  // user have to specify the sensor distances to be maintained by the robot.
  myRobot.runWallFollowRobot(30, 20);
}