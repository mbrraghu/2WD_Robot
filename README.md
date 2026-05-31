# 2WD_Robot

An Arduino library for controlling 2-wheel drive robots with multiple autonomous and remote-controlled modes.

## Features

- **Obstacle Avoidance** - Ultrasonic sensor-based obstacle detection and avoidance
- **Cliff Detection** - IR sensor-based edge/cliff detection
- **Line Following** - 5-sensor IR array line following
- **Wall Following** - Dual ultrasonic wall following (left or right)
- **Bluetooth Control** - Remote control via Bluetooth serial
- **Motor Driver** - Cytron MDD3A motor driver support (PWM_PWM and PWM_DIR modes)

## Hardware Requirements

- Arduino board (Uno, Mega, Nano, etc.)
- Cytron MDD3A motor driver
- 2 DC gear motors with wheels
- HC-SR04 ultrasonic sensor (for obstacle/wall following)
- IR sensor array (5 sensors for line following)
- IR digital sensors (for cliff detection)
- Bluetooth module (HC-05/HC-06 for remote control)
- Robot chassis with battery pack

## Pin Configuration

### Motor Driver (MDD3A)
| Parameter | Description |
|-----------|-------------|
| M1_pinA   | Motor 1 terminal A |
| M1_pinB   | Motor 1 terminal B |
| M2_pinA   | Motor 2 terminal A |
| M2_pinB   | Motor 2 terminal B |
| MODE      | PWM_PWM or PWM_DIR |

### Sensor Pins
| Sensor | Pins |
|--------|------|
| HC-SR04 | Trigger, Echo |
| IR Digital | Signal pin |
| IR Analog | Analog signal pin |
| IR Array | 5 digital pins |

## Installation

### Via Arduino Library Manager (Recommended)
1. Open Arduino IDE
2. Go to Sketch > Include Library > Manage Libraries...
3. Search for "2WD_Robot"
4. Click Install

### Manual Installation
1. Download this repository
2. Copy the folder to your Arduino libraries folder (usually `Documents/Arduino/libraries/`)
3. Restart the Arduino IDE

## API Reference

### MDD3A (Motor Driver)

```cpp
MDD3A(MODE mode, uint8_t M1_pinA, uint8_t M1_pinB, uint8_t M2_pinA, uint8_t M2_pinB);
void motorDrive(int M1_speed, int M2_speed, int duration);
```

**MODE Options:**
- `PWM_PWM` - Both motor terminals connected to PWM pins
- `PWM_DIR` - One terminal PWM, other terminal direction (HIGH/LOW)

**motorDrive Parameters:**
- `M1_speed`: -255 to 255 (negative = reverse)
- `M2_speed`: -255 to 255 (negative = reverse)
- `duration`: Motor activation time in milliseconds

### ultraSound (HC-SR04)

```cpp
ultraSound(int trigPin, int echoPin);
float getUltrasoundDistance(); // Returns distance in cm
```

### infraredDigital

```cpp
infraredDigital(int IR_signalPin);
bool getInfraredDigitalSignal(); // Returns HIGH or LOW
```

### infraredAnalog

```cpp
infraredAnalog(int IR_signalPin);
int getInfraredAnalogSignal(); // Returns 0-1023
```

### obstacleAvoidanceRobot

```cpp
obstacleAvoidanceRobot(int trigPin, int echoPin, uint8_t M1_A, uint8_t M1_B, uint8_t M2_A, uint8_t M2_B, MODE motorsPinMode);
void runObstacleRobot(int obstacleDistance, int turnTime, int leftWheelSpeed = 50, int rightWheelSpeed = 50);
```

### cliffDetectionRobot

```cpp
cliffDetectionRobot(int leftSensorPin, int rightSensorPin, uint8_t M1_A, uint8_t M1_B, uint8_t M2_A, uint8_t M2_B, MODE motorsPinMode);
void runCliffRobot(sensorType type, int turnDelay, int leftWheelSpeed = 50, int rightWheelSpeed = 50);
```

**sensorType Options:**
- `ACTIVE_LOW` - Sensor outputs LOW when obstacle detected
- `ACTIVE_HIGH` - Sensor outputs HIGH when obstacle detected

### lineFollowRobot

```cpp
lineFollowRobot(int sensorpins[], uint8_t M1_A, uint8_t M1_B, uint8_t M2_A, uint8_t M2_B, MODE motorsPinMode);
void runLineFollowRobot(sensorType type, int leftWheelSpeed = 50, int rightWheelSpeed = 50);
byte lineSensorRead(int pins[]);
```

### wallFollowRobot

```cpp
wallFollowRobot(wallFollowMode mode, int USfrontPins[2], int USsidePins[2], uint8_t M1_A, uint8_t M1_B, uint8_t M2_A, uint8_t M2_B, MODE motorsPinMode);
void runWallFollowRobot(float frontDistance, float sideDistance, int leftWheelSpeed = 20, int rightWheelSpeed = 20);
```

**wallFollowMode Options:**
- `LEFT_FOLLOW` - Follow wall on left side
- `RIGHT_FOLLOW` - Follow wall on right side

### bluetoothRobotCar

```cpp
bluetoothRobotCar(uint8_t M1_A, uint8_t M1_B, uint8_t M2_A, uint8_t M2_B, MODE motorsPinMode);
void runRobotCar(int leftWheelSpeed = 50, int rightWheelSpeed = 50);
char bluetoothData();
```

**Bluetooth Commands:**
| Command | Action |
|---------|--------|
| F | Forward |
| B | Backward |
| R | Turn Right |
| L | Turn Left |
| G | Forward Left |
| I | Forward Right |
| H | Backward Left |
| J | Backward Right |

## Examples

### Basic Motor Control
```cpp
#include "TwoWDRobot.h"

MDD3A driver(PWM_PWM, 5, 6, 9, 10);

void setup() {
  Serial.begin(9600);
}

void loop() {
  driver.motorDrive(50, 50, 1000);   // Forward
  driver.motorDrive(50, -50, 1000);  // Right turn
  driver.motorDrive(-50, -50, 1000); // Backward
  driver.motorDrive(-50, 50, 1000);  // Left turn
}
```

### Obstacle Avoidance Robot
```cpp
#include "TwoWDRobot.h"

#define TRIG 2
#define ECHO 3

obstacleAvoidanceRobot myRobot(TRIG, ECHO, 5, 6, 9, 10, PWM_PWM);

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.print("Distance: ");
  Serial.println(myRobot.getUltrasoundDistance());
  myRobot.runObstacleRobot(30, 500);
}
```

### Line Following Robot
```cpp
#include "TwoWDRobot.h"

int IRArrayPins[5] = {2, 3, 4, 6, 7};
lineFollowRobot myRobot(IRArrayPins, 5, 9, 10, 11, PWM_PWM);

void setup() {
  Serial.begin(9600);
}

void loop() {
  myRobot.runLineFollowRobot(ACTIVE_HIGH);
}
```

### Bluetooth Controlled Robot
```cpp
#include "TwoWDRobot.h"

bluetoothRobotCar myRobot(5, 9, 10, 11, PWM_PWM);

void setup() {
  Serial.begin(9600);
}

void loop() {
  myRobot.runRobotCar();
  Serial.println(myRobot.bluetoothData());
}
```

### Wall Following Robot
```cpp
#include "TwoWDRobot.h"

int us_front_pins[] = {2, 3};
int us_side_pins[] = {4, 6};

wallFollowRobot myRobot(LEFT_FOLLOW, us_front_pins, us_side_pins, 5, 9, 10, 11, PWM_PWM);

void setup() {
  Serial.begin(9600);
}

void loop() {
  myRobot.runWallFollowRobot(30, 20);
}
```

### Cliff Detection Robot
```cpp
#include "TwoWDRobot.h"

#define LEFT_IR 2
#define RIGHT_IR 3

cliffDetectionRobot myRobot(LEFT_IR, RIGHT_IR, 5, 6, 9, 10, PWM_PWM);

void setup() {
  Serial.begin(9600);
}

void loop() {
  myRobot.runCliffRobot(ACTIVE_LOW, 1500, 30, 30);
}
```

## Troubleshooting

- **Motors not spinning**: Check motor driver wiring and power supply
- **Ultrasonic returns 0**: Verify trigger and echo pin connections
- **Line following erratic**: Calibrate IR sensors and ensure proper line contrast
- **Bluetooth not connecting**: Check baud rate (default 9600) and pairing status

## License

Open source - feel free to use and modify.

## Author

Raghunandan B
