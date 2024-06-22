#include "Wire.h"
#include <MPU6050_light.h>
#include <SoftwareSerial.h>

// Startup params
const bool ULTRASONIC_ENABLED = 0;

// Motor control pins
const int MOTOR1_P1 = 4;
const int MOTOR1_P2 = 5;
const int MOTOR2_P1 = 6;
const int MOTOR2_P2 = 7;

// Sensor Pins
const int SENSOR_L = A0;
const int SENSOR_R = A1;

// Start/stop button pin
const int CONTROL_BUTTON = 2;

// Speed control factors for motors
float motor1scaling = 0.75f;
float motor2scaling = 0.75f;

const int MAX_DUTY = 255;
const float POWER_TARGET = 0.50f;

// Direction control variables
float headingAngle = 0.f;
float facingAngle = 0.f;

const float MAX_POWER_DELTA = 0.25f;
const float ANGLE_TOLERANCE = 0.75f;

byte lastBT = 0x00;

int turnCount = 0;

// Gyroscope HAL instance
MPU6050 mpu( Wire );

// Instantiate bluetooth chip serial connection
SoftwareSerial Bluetooth(8, 9);

// State machine members
enum tractorState {
  DRIVE,
  STOP,
  ESTOP
};
tractorState tState = STOP;

enum turnState {
  LEFT1,
  LEFT2,
  RIGHT1,
  RIGHT2
};
turnState rState = LEFT1;

void setup() {
  // Set up all motor control pins as output
  pinMode(MOTOR1_P1, OUTPUT);
  pinMode(MOTOR1_P2, OUTPUT);
  pinMode(MOTOR2_P1, OUTPUT);
  pinMode(MOTOR2_P2, OUTPUT);

  // Start/stop button as input
  pinMode(CONTROL_BUTTON, INPUT);
  // Set up interrupt binding to manage button events
  attachInterrupt(digitalPinToInterrupt(CONTROL_BUTTON), toggleStop, FALLING);

  // Sensor pins
  pinMode(SENSOR_L, INPUT);
  pinMode(SENSOR_R, INPUT);

  pinMode(3, INPUT);
  // attachInterrupt(digitalPinToInterrupt(3), sensorStop, CHANGE);

  // Start BT serial
  Bluetooth.begin(9600);

  // Start serial monitoring channel
  Serial.begin(115200);

  // Start I2C connection
  Wire.begin();
  mpu.begin(0, 0);
  // Calibrates gyro to surface while stationary
  mpu.calcOffsets();
  mpu.update();
  facingAngle = mpu.getAngleZ();
  headingAngle = facingAngle;
}

void loop() {
  int sensorLeft = analogRead(SENSOR_L);

  if (sensorLeft < 100) {
    turn();
  }
  
  // Gyroscope ping
  mpu.update(); // Ask gyrocope for data
  facingAngle = mpu.getAngleZ(); // Fetch yaw angle from Gyroscope

  float aberration = headingAngle - facingAngle;

  if (aberration > ANGLE_TOLERANCE) {
    powerNudgeRight();
  } else if (aberration < -ANGLE_TOLERANCE) {
    powerNudgeLeft();
  } else {
    resetPowerScaling();
  }
  powerScale();

  mpu.update(); // Documentation reccommends calling this method often to improve position integral accuracy
  
  if (Bluetooth.available()) {
    byte BTCommand = Bluetooth.read();
    if (BTCommand != lastBT) {
      toggleStop();
      lastBT = BTCommand;
    }
  }

  mpu.update();

  // State control flow
  switch (tState) {
  case STOP :
    motor1_stop();
    motor2_stop();
    // This lets us pick up tractor and adjust heading
    mpu.update();
    facingAngle = mpu.getAngleZ();
    headingAngle = facingAngle;
    break;
  case DRIVE :
    motor1_forward();
    motor2_forward();
    break;
  case ESTOP :
    motor1_stop();
    motor2_stop();
    break;
  }

  // sensorStop();
  
  // printMotorScaling();

}

void printMotorScaling() {
  Serial.print(motor1scaling);
  Serial.print("   ");
  Serial.print(motor2scaling);
  Serial.print("\n");
}

void powerNudgeLeft() {
  if (abs(motor1scaling - motor2scaling) < MAX_POWER_DELTA) {
    motor1scaling += 0.05f;
  }
}

void powerNudgeRight() {
  if (abs(motor1scaling - motor2scaling) < MAX_POWER_DELTA) {
    motor2scaling += 0.05f;
  }
}

void resetPowerScaling() {
  motor1scaling = POWER_TARGET;
  motor2scaling = POWER_TARGET;
}

void powerScale() {
  float currentPower = (motor1scaling + motor2scaling) / 2;
  float correctionFactor = POWER_TARGET / currentPower;
  motor1scaling  *= correctionFactor;
  motor2scaling *= correctionFactor;
}

void toggleStop() {
  // Toggle state
  if (tState == STOP) {
    tState = DRIVE;
    Serial.print("Drive (button)");
    Serial.print("\n");
  } else if (tState == DRIVE) {
    tState = STOP;
    Serial.print("Stop (button)");
    Serial.print("\n"); 
  }
}

void eStop() {
  if (tState == DRIVE) {
    tState = ESTOP;
    Serial.print("Stop (sensor)");
    Serial.print("\n"); 
  } 
}

void clearStart() {
  if (tState == ESTOP) {
    tState = DRIVE; 
  } 
}

void sensorStop() {
  int sensorState = digitalRead(3);
  if (tState == DRIVE && sensorState == HIGH) {
    tState = ESTOP;
    Serial.print("Stop (sensor)");
    Serial.print("\n"); 
  } else if (tState == ESTOP && sensorState == LOW){
    tState = DRIVE;
    Serial.print("Start (sensor)");
    Serial.print("\n"); 
  }
}

void turn() {
  motor1_stop();
  motor2_stop();
  mpu.update();
  facingAngle = mpu.getAngleZ();
  int targetAngle = 0;
  if (rState == LEFT1 || rState == LEFT2) {
    targetAngle = headingAngle + 90;
  } else {
    targetAngle = headingAngle - 90;
  }
  headingAngle = targetAngle;
  while (true) {
    mpu.update();
    facingAngle = mpu.getAngleZ();
    if (rState == LEFT1 || rState == LEFT2) {
      motor1_stop();
      motor2_forward();
    } else {
      motor1_forward();
      motor2_stop();
    }
    if (facingAngle > targetAngle && (rState == LEFT1 || rState == LEFT2)) {
      if (rState == LEFT1) {
        rState = LEFT2;
      } else {
        rState = RIGHT1;
      }
      turnCount++;
      motor1_stop();
      motor2_stop();
      return;
    }
    if (facingAngle < targetAngle && (rState == RIGHT1 || rState == RIGHT2)) {
      if (rState == RIGHT1) {
        rState = RIGHT2;
      } else {
        rState = LEFT1;
      }
      turnCount++;
      motor1_stop();
      motor2_stop();
      return;
    }
  }
}

// Motor HAL below this point

void motor1_forward() {
  int motorPower = floor(MAX_DUTY * motor1scaling);
  digitalWrite(MOTOR1_P1, LOW);
  analogWrite(MOTOR1_P2, motorPower);
}

void motor1_stop() {
  digitalWrite(MOTOR1_P1, LOW);
  analogWrite(MOTOR1_P2, 0);
}

void motor2_forward() {
  int motorPower = floor(MAX_DUTY * motor2scaling);
  analogWrite(MOTOR2_P1, motorPower);
  digitalWrite(MOTOR2_P2, LOW);
}

void motor2_stop() {
  analogWrite(MOTOR2_P1, 0);
  digitalWrite(MOTOR2_P2, LOW);
}
