#include "Arduino.h"
#include "TwoWDRobot.h"

MDD3A::MDD3A(MODE mode, uint8_t M1_pinA, uint8_t M1_pinB, uint8_t M2_pinA, uint8_t M2_pinB)
{
  _mode = mode;
  motorPins[0] = M1_pinA;
  motorPins[1] = M1_pinB;
  motorPins[2] = M2_pinA;
  motorPins[3] = M2_pinB;

  for (int i = 0; i < 4; i++)
  {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }
}

void MDD3A::motorDrive(int16_t M1_speed, int16_t M2_speed, int delay_duration)
{

  if (M1_speed > 255 || M1_speed < -255)
  {
    M1_speed = 255;
  }

  if (M2_speed > 255 || M2_speed < -255)
  {
    M2_speed = 255;
  }

  switch (_mode)
  {
  case PWM_PWM:
    if (M1_speed >= 0)
    {
      analogWrite(motorPins[0], M1_speed);
      analogWrite(motorPins[1], 0);
    }
    else
    {
      analogWrite(motorPins[0], 0);
      analogWrite(motorPins[1], abs(M1_speed));
    }

    if (M2_speed >= 0)
    {
      analogWrite(motorPins[2], M2_speed);
      analogWrite(motorPins[3], 0);
    }
    else
    {
      analogWrite(motorPins[2], 0);
      analogWrite(motorPins[3], abs(M2_speed));
    }
    delay(delay_duration);
    break;

  case PWM_DIR:
    if (M1_speed >= 0)
    {
      analogWrite(motorPins[0], M1_speed);
      digitalWrite(motorPins[1], LOW);
    }
    else
    {
      analogWrite(motorPins[0], abs(M1_speed));
      digitalWrite(motorPins[1], HIGH);
    }

    if (M2_speed >= 0)
    {
      analogWrite(motorPins[2], M2_speed);
      digitalWrite(motorPins[3], LOW);
    }
    else
    {
      analogWrite(motorPins[2], abs(M2_speed));
      digitalWrite(motorPins[3], HIGH);
    }
    delay(delay_duration);
    break;
  }
}

ultraSound ::ultraSound(int trigPin, int echoPin)
{
  echo = echoPin;
  trigger = trigPin;
  pinMode(echo, INPUT);
  pinMode(trigger, OUTPUT);
}

float ultraSound ::getUltrasoundDistance()
{
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  float duration = pulseIn(echo, HIGH);
  float distance = duration * 0.034 / 2;
  return distance;
}

infraredDigital::infraredDigital(int IR_signalPin)
{
  digitalPin = IR_signalPin;
  pinMode(digitalPin, INPUT);
}

bool infraredDigital::getInfraredDigitalSignal()
{
  return digitalRead(digitalPin);
}

infraredAnalog::infraredAnalog(int IR_signalPin)
{
  analogPin = IR_signalPin;
  pinMode(analogPin, INPUT);
}

int infraredAnalog::getInfraredAnalogSignal()
{
  return analogRead(analogPin);
}

obstacleAvoidanceRobot::obstacleAvoidanceRobot(int trigPin, int echoPin, uint8_t M1_A, uint8_t M1_B, uint8_t M2_A, uint8_t M2_B, MODE motorsPinMode) : ultraSound(trigPin, echoPin), MDD3A(motorsPinMode, M1_A, M1_B, M2_A, M2_B)
{
}
void obstacleAvoidanceRobot::runObstacleRobot(int obstacleDistance, int turnTime, int leftWheelSpeed = 50, int rightWheelSpeed = 50)
{
  float distance = getUltrasoundDistance();

  if (distance > obstacleDistance)
  {
    motorDrive(leftWheelSpeed, rightWheelSpeed, 50);
  }

  else
  {
    motorDrive(leftWheelSpeed, -rightWheelSpeed, turnTime);
  }
}

cliffDetectionRobot::cliffDetectionRobot(int leftSensorPin, int rightSensorPin, uint8_t M1_A, uint8_t M1_B, uint8_t M2_A, uint8_t M2_B, MODE motorsPinMode) : MDD3A(motorsPinMode, M1_A, M1_B, M2_A, M2_B)
{
  leftSignalPin = leftSensorPin;
  rightSignalPin = rightSensorPin;
}

void cliffDetectionRobot::runCliffRobot(int turnDelay, bool sensorType = true, int leftWheelSpeed = 50, int rightWheelSpeed = 50)
{
  int sensor_type = int(sensorType);
  infraredDigital left_sensor(leftSignalPin);
  infraredDigital right_sensor(rightSignalPin);

  bool leftSensorStatus = left_sensor.getInfraredDigitalSignal();
  bool rightSensorStatus = right_sensor.getInfraredDigitalSignal();

  switch (sensor_type)
  {
  case 0:
    if (leftSensorStatus == LOW && rightSensorStatus == LOW)
    {
      motorDrive(leftWheelSpeed, rightWheelSpeed, 50);
    }
    else if (leftSensorStatus == LOW && rightSensorStatus == HIGH)
    {
      motorDrive(-leftWheelSpeed, -rightWheelSpeed, turnDelay);
      motorDrive(-leftWheelSpeed, rightWheelSpeed, turnDelay);
    }
    else if (leftSensorStatus == HIGH && rightSensorStatus == LOW)
    {
      motorDrive(-leftWheelSpeed, -rightWheelSpeed, turnDelay);
      motorDrive(leftWheelSpeed, -rightWheelSpeed, turnDelay);
    }
    else if (leftSensorStatus == HIGH && rightSensorStatus == HIGH)
    {
      motorDrive(-leftWheelSpeed, -rightWheelSpeed, turnDelay);
      motorDrive(leftWheelSpeed, -rightWheelSpeed, turnDelay);
    }
    break;

  case 1:
    if (leftSensorStatus == HIGH && rightSensorStatus == HIGH)
    {
      motorDrive(leftWheelSpeed, rightWheelSpeed, 50);
    }
    else if (leftSensorStatus == HIGH && rightSensorStatus == LOW)
    {
      motorDrive(-leftWheelSpeed, -rightWheelSpeed, turnDelay);
      motorDrive(-leftWheelSpeed, rightWheelSpeed, turnDelay);
    }
    else if (leftSensorStatus == LOW && rightSensorStatus == HIGH)
    {
      motorDrive(-leftWheelSpeed, -rightWheelSpeed, turnDelay);
      motorDrive(leftWheelSpeed, -rightWheelSpeed, turnDelay);
    }
    else if (leftSensorStatus == LOW && rightSensorStatus == LOW)
    {
      motorDrive(-leftWheelSpeed, -rightWheelSpeed, turnDelay);
      motorDrive(leftWheelSpeed, -rightWheelSpeed, turnDelay);
    }
    break;

  default:
    break;
  }
}

bluetoothRobotCar::bluetoothRobotCar(uint8_t M1_A, uint8_t M1_B, uint8_t M2_A, uint8_t M2_B, MODE motorsPinMode) : MDD3A(motorsPinMode, M1_A, M1_B, M2_A, M2_B)
{
  Serial.begin(9600);
}

void bluetoothRobotCar::runRobotCar(int leftWheelSpeed = 50, int rightWheelSpeed = 50)
{
  char data;
  if (Serial.available())
  {
    data = Serial.read();
    delay(20);
    switch (data)
    {
    case 'F':
      motorDrive(leftWheelSpeed, rightWheelSpeed, 50);
      break;

    case 'B':
      motorDrive(-leftWheelSpeed, -rightWheelSpeed, 50);
      break;

    case 'R':
      motorDrive(leftWheelSpeed, -rightWheelSpeed, 50);
      break;

    case 'L':
      motorDrive(-leftWheelSpeed, rightWheelSpeed, 50);
      break;

    case 'G':
      motorDrive(leftWheelSpeed / 2, rightWheelSpeed, 50);
      break;

    case 'I':
      motorDrive(leftWheelSpeed, rightWheelSpeed / 2, 50);
      break;

    case 'H':
      motorDrive(-leftWheelSpeed / 2, -rightWheelSpeed, 50);
      break;

    case 'J':
      motorDrive(-leftWheelSpeed, -rightWheelSpeed / 2, 50);
      break;

    default:
      motorDrive(0, 0, 50);
      break;
    }
  }
}

// To display the data received by the Bluetooth device
char bluetoothRobotCar::bluetoothData()
{
  if (Serial.available())
  {
    return Serial.read();
    delay(20);
  }
}
