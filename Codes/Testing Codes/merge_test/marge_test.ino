#include <L298N.h>
#include <Arduino.h>

// Motor pin definitions
#define AIN1 49
#define BIN1 51
#define AIN2 47
#define BIN2 53
#define PWMA 4
#define PWMB 3

// Infrared sensor definitions
const int SensorCount = 8;
const int analogSensorCount = 8;
const int analogSensorPins[analogSensorCount] = {A0, A1, A2, A3, A4, A5, A6, A7};
int IR_weight[8] = {-30, -20, -10, -5, 5, 10, 20, 30};

// PID control parameters
float Kp = 5;
float Ki = 0.01;
float Kd = 7;

// PID variables
float P, I, D, previousError = 0;
float error;

float lsp, rsp;
int lfspeed = 150;
int turnspeed = 100;

// Ultrasonic sensor definitions
#define LEFT_SENSOR_TRIG A8
#define LEFT_SENSOR_ECHO A9
#define FRONT_SENSOR_TRIG A10
#define FRONT_SENSOR_ECHO A11
#define RIGHT_SENSOR_TRIG A12
#define RIGHT_SENSOR_ECHO A13

// Initializing motors
L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

// Define maximum and minimum distances for wall following
#define MAX_DISTANCE 20
#define MIN_DISTANCE 10

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
  pinMode(LED_BUILTIN, OUTPUT);

  // Start with motors stopped
  motor1.stop();
  motor2.stop();

  // Initialize ultrasonic sensor pins
  pinMode(LEFT_SENSOR_TRIG, OUTPUT);
  pinMode(LEFT_SENSOR_ECHO, INPUT);
  pinMode(FRONT_SENSOR_TRIG, OUTPUT);
  pinMode(FRONT_SENSOR_ECHO, INPUT);
  pinMode(RIGHT_SENSOR_TRIG, OUTPUT);
  pinMode(RIGHT_SENSOR_ECHO, INPUT);

  // Start serial communication for debugging
  Serial.begin(9600);
}

void readSensors(int *values) {
  // Read analog sensors
  for (int i = 0; i < analogSensorCount; i++) {
    values[i] = analogRead(analogSensorPins[i]) > 300 ? 0 : 1;
    Serial.print(values[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
}

float calculateError(int *sensorValues) {
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

  return error;
}

float calculatePID(float error) {
  P = error;
  I += error;
  D = error - previousError;
  previousError = error;

  float pidValue = (Kp * P) + (Ki * I) + (Kd * D);

  return pidValue;
}

void PID_Linefollow(float pidValue) {
  lsp = lfspeed + pidValue;
  rsp = lfspeed - pidValue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < -255) {
    lsp = -255;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < -255) {
    rsp = -255;
  }
  motor_drive(lsp, rsp);
}

void motor_drive(float left, float right) {
  int absRight = abs(right);
  int absLeft = abs(left);

  if (left > 0) {
    motor2.setSpeed(left);
    motor2.forward();
  } else {
    motor2.setSpeed(absLeft);
    motor2.backward();
  }

  if (right > 0) {
    motor1.setSpeed(right);
    motor1.forward();
  } else {
    motor1.setSpeed(absRight);
    motor1.backward();
  }
}

int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;

  return distance;
}

void moveForward() {
  motor1.setSpeed(lfspeed);
  motor1.forward();
  motor2.setSpeed(lfspeed);
  motor2.forward();
}

void turnLeft() {
  motor1.setSpeed(turnspeed);
  motor1.forward();
  motor2.setSpeed(turnspeed);
  motor2.backward();
}

void turnRight() {
  motor1.setSpeed(turnspeed);
  motor1.backward();
  motor2.setSpeed(turnspeed);
  motor2.forward();
}

void loop() {
  int leftDistance = getDistance(LEFT_SENSOR_TRIG, LEFT_SENSOR_ECHO);
  int frontDistance = getDistance(FRONT_SENSOR_TRIG, FRONT_SENSOR_ECHO);
  int rightDistance = getDistance(RIGHT_SENSOR_TRIG, RIGHT_SENSOR_ECHO);

  int safeDistance = MAX_DISTANCE + 5;
  int turnThreshold = 10;

  if (frontDistance < safeDistance || rightDistance < safeDistance || leftDistance < safeDistance) {
    if (frontDistance > rightDistance && frontDistance > leftDistance) {
      moveForward();
    } else {
      if (rightDistance > leftDistance + turnThreshold) {
        turnRight();
      } else if (leftDistance > rightDistance + turnThreshold) {
        turnLeft();
      }
    }
  } else {
    lineFollowing();
  }
}

void lineFollowing() {
  int sensorValues[SensorCount];
  readSensors(sensorValues);

  if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0) {
    motor1.stop();
    motor2.stop();
  } else {
    float error = calculateError(sensorValues);
    float pidValue = calculatePID(error);
    PID_Linefollow(pidValue);
  }
}
