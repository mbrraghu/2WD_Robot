#include "TwoWDRobot.h"

// Object to access the methods to run bluetooth controlled robot.
// It takes inputs for control pins of MDD3A motor driver.
bluetoothRobotCar myRobot(5, 9, 10, 11, PWM_PWM);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  // Method to run the bluetooth controlled robot.
  // both the motor speeds are set to 50 PWM values by default. User can key in the required speeds.
  myRobot.runRobotCar();
  // Method to print the data received by the bluetooth.
  Serial.println(myRobot.bluetoothData());
}