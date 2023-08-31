#include <stdint.h>
#ifndef TwoWDRobot_H
#define TwoWDRobot_H

#include <Arduino.h>

enum MODE
{
  PWM_PWM,
  PWM_DIR
};

class MDD3A
{
protected:
  MODE _mode;
  uint8_t motorPins[4];

public:
  MDD3A(MODE mode, uint8_t M1_pinA, uint8_t M1_pinB, uint8_t M2_pinA, uint8_t M2_pinB);
  void motorDrive(int M1_speed, int M2_speed, int duration);
};

class ultraSound
{
protected:
  int echo, trigger;

public:
  ultraSound(int trigPin, int echoPin);
  float getUltrasoundDistance();
};

class infraredDigital
{
protected:
  int digitalPin;

public:
  infraredDigital(int IR_signalPin);
  bool getInfraredDigitalSignal();
};

class infraredAnalog
{
protected:
  int analogPin;

public:
  infraredAnalog(int IR_signalPin);
  int getInfraredAnalogSignal();
};

class obstacleAvoidanceRobot : public ultraSound, public MDD3A
{
public:
  obstacleAvoidanceRobot(int trigPin, int echoPin, uint8_t M1_A, uint8_t M1_B, uint8_t M2_A, uint8_t M2_B, MODE motorsPinMode);
  void runObstacleRobot(int obstacleDistance, int turnTime, int leftWheelSpeed = 50, int rightWheelSpeed = 50);
};

class cliffDetectionRobot : public MDD3A
{
private:
  int leftSignalPin, rightSignalPin;

public:
  cliffDetectionRobot(int leftSensorPin, int rightSensorPin, uint8_t M1_A, uint8_t M1_B, uint8_t M2_A, uint8_t M2_B, MODE motorsPinMode);
  void runCliffRobot(int turnDelay, bool sensorType = true, int leftWheelSpeed = 50, int rightWheelSpeed = 50);
};

class bluetoothRobotCar : public MDD3A
{
public:
  bluetoothRobotCar(uint8_t M1_A, uint8_t M1_B, uint8_t M2_A, uint8_t M2_B, MODE motorsPinMode);
  void runRobotCar(int leftWheelSpeed = 50, int rightWheelSpeed = 50);
  char bluetoothData();
};

#endif