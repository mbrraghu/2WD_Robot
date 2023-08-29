#include "TwoWDRobot.h"
#define TRIG 2 //Arduino pin connected to HC-SR04 trigger pin.
#define ECHO 3 //Arduino pin connected to HC-SR04 echo pin.

//Object to access the methods to run obstacle avoidance robot.
//It takes inputs for ultrasound control pins and MDD3A motor driver pins.
obstacleAvoidanceRobot myRobot(TRIG,ECHO,5,6,9,10,PWM_PWM);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  Serial.print("Sensor Distance: ");Serial.println(myRobot.getUltrasoundDistance());
  //Method takes minimum obstacle distance to avoid and delay time to deviate from the obstacle.
  //By default PWM values for motors are set to 50 each, user can change as per their needs.
  myRobot.runObstacleRobot(30,500);
}