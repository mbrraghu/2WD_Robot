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
  myRobot.runRobotCar(70, 70);
  Serial.println(myRobot.bluetoothData());
}