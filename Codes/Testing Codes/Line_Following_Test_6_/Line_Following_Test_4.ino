#include <L298N.h>
#include <Arduino.h>

// Function prototypes
void readSensors(int *values);
float calculatePID(int *sensorValues);
void PID_Linefollow(float pidValue);
void motor_drive(float speed, float turn);
void stop_motors();

// Have to redefine those below pins as we plugged
#define AIN1 49
#define BIN1 51
#define AIN2 47
#define BIN2 53
#define PWMA 4
#define PWMB 3

// Initializing motors.
L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

// Constants
const int SensorCount = 8;
const int analogSensorCount = 8;
const int analogSensorPins[analogSensorCount] = {A0, A1, A2, A3, A4, A5, A6, A7};
int IR_weight[8] = {-30, -20, -10, -5, 5, 10, 20, 30};

// PID control parameters
float Kp = 4;
float Ki = 0.0;
float Kd = 2;
float error;

// PID variables
float P, I, D, previousError = 0;

// Speed parameter
int lfspeed = 100;

void setup() {
  // Initialize analog sensor pins as input
  for (int i = 0; i < analogSensorCount; i++) {
    pinMode(analogSensorPins[i], INPUT);
  }

  // Initialize motor driver pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Start with motors stopped
  stop_motors();
  Serial.begin(9600);
}

void loop() {
  int sensorValues[SensorCount];
  readSensors(sensorValues);
  
    float pidValue = calculatePID(sensorValues);
    PID_Linefollow(pidValue);
  
}

void readSensors(int *values) {
  for (int i = 0; i < analogSensorCount; i++) {
    values[i] = analogRead(analogSensorPins[i]) > 300 ? 0 : 1;
  }
}

float calculatePID(int *sensorValues) {
  float position = 0;
  int onLine = 0;

  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] == 1) {
      position += IR_weight[i];
      onLine++;
    }
  }

  if (onLine == 0) {
    error = -previousError;
  } else {
    position /= onLine;
    error = position;
  }

  P = error;
  I += error;
  D = error - previousError;
  previousError = error;

  float pidValue = (Kp * P) + (Ki * I) + (Kd * D);
  return pidValue;
}

void PID_Linefollow(float pidValue) {
  float speed = lfspeed;
  float turn = pidValue;

  motor_drive(speed, turn);
}

void motor_drive(float speed, float turn) {
  // Calculate motor speeds
  float left_speed = speed + turn;
  float right_speed = speed - turn;

  // Ensure motor speeds are within limits
  left_speed = constrain(left_speed, -255, 255);
  right_speed = constrain(right_speed, -255, 255);

  // Set motor speeds and directions
  if (left_speed > 0) {
    motor2.setSpeed(left_speed);
    motor2.forward();
  } else {
    motor2.setSpeed(-left_speed);
    motor2.backward();
  }

  if (right_speed > 0) {
    motor1.setSpeed(right_speed);
    motor1.forward();
  } else {
    motor1.setSpeed(-right_speed);
    motor1.backward();
  }
}

void stop_motors() {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
}
