#include <L298N.h>
#include <Arduino.h>

//////////////
// Have to redefine those below pins as we plugged
#define AIN1 49
#define BIN1 51
#define AIN2 47
#define BIN2 53
#define PWMA 4
#define PWMB 3
//#define IR_L A9
//#define IR_R A8
//motor1=right
////////

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

// Constants
const int SensorCount = 8; // Total number of sensors
const int analogSensorCount = 8; // Number of analog sensors
const int analogSensorPins[analogSensorCount] = {A7, A6, A5, A4, A3, A2, A1, A0}; // Analog pins for the remaining sensors
int IR_weight[8] = { -30, -20, -10, -5, 5, 10, 20, 30};

// PID control parameters
float Kp = 3; // Proportional term
float Ki = 0.0; // Integral term
float Kd = 2; // Derivative term

float error;

// PID variables
float P, I, D, previousError = 0;

float lsp, rsp;
int lfspeed = 90;
int turnSpeed = 100;

// Function prototypes
void readSensors(int *values);
float calculatePID(int *sensorValues);
void motor_drive(int left, int right);

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
  motor1.stop();
  motor2.stop();
  Serial.begin(9600);
}

void loop() {
  int sensorValues[SensorCount];
  readSensors(sensorValues);
  

  if (isImmediateTurn(sensorValues)) {
//    motor1.stop();
//    motor2.stop();
//    delay(200);
    turn(sensorValues);
  }
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0) {
    motor1.stop();
    motor2.stop();
  }
  else {

    float pidValue = calculatePID(sensorValues);
    PID_Linefollow(pidValue);

    // You may adjust the delay for performance tuning
  }
  //delay(10);
}

void readSensors(int *values) {
  // Read analog sensors
  for (int i = 0; i < analogSensorCount; i++) {
    values[i] = analogRead(analogSensorPins[i]) > 300 ? 0 : 1; // Assuming higher values indicate no line
    Serial.print(values[i]);
    Serial.print(" ");

  }
//  Serial.print(analogRead(IR_L)> 100 ? 0 : 1);
//  Serial.print(" ");
//  Serial.print(analogRead(IR_R)> 100 ? 0 : 1);
  Serial.println(" ");
}

float calculatePID(int *sensorValues) {
  float position = 0;
  int onLine = 0;

  // Loop through all sensors
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] == 1) { // Assuming 1 indicates line detected
      position += IR_weight[(i)];
      onLine++;
    }
  }

  // If no line is detected by any sensor, use the last known error value
  if (onLine == 0) {
    // If previous error is not available, assume the line is straight ahead
    error = -previousError;
  }
  else {
    // Calculate the average position of the line
    position /= onLine;
    // Calculate error based on sensor position
    error = position;
  }
  // PID terms
  P = error;
  I += error;
  D = error - previousError;
  previousError = error;

  // Calculate PID value
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
  int absRight = abs(right); // Absolute value for right speed
  int absLeft = abs(left);   // Absolute value for left speed

  if (left > 0) {
    motor2.setSpeed(absLeft);
    motor2.forward();
  } else {
    motor2.setSpeed(absLeft);
    motor2.backward();
  }

  if (right > 0) {
    motor1.setSpeed(absRight);
    motor1.forward();
  } else {
    motor1.setSpeed(absRight);
    motor1.backward();
  }
}
bool isImmediateTurn(int *sensorValues) {
  bool turnLeft = ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 ) ||(sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1));
  bool turnRight = ((sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) || (sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) );
  return (turnLeft || turnRight);
}
void turn(int *sensorValues) {

  // Check if A0, A1, A2, A3 are detecting the line (indicating a left turn)
  if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 ) ||(sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1)){
    motor1.forward();
    motor2.forward();
    delay(500);
    // Perform a sharp left turn
    // Assuming motor1 is the right motor and motor2 is the left motor
    bool enoughturnLeft = ( sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] )== 0;
    while (!enoughturnLeft) {
      readSensors(sensorValues);
      enoughturnLeft =  (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7]) == 0;
      motor1.setSpeed(2*turnSpeed);   // Set right motor speed
      motor1.forward();       // Move right motor forward
      //motor2.setSpeed(turnSpeed); // Set left motor speed
      //motor2.backward();      // Move left motor backward for a sharper turn
      
      //analogWrite(3 , turnSpeed);
      //digitalWrite(53 , HIGH);
      //digitalWrite(51 ,LOW);

    }
//    motor1.stop();
//    motor2.stop();

  }
  // Check if A4, A5, A6, A7 are detecting the line (indicating a right turn)
  if ((sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) || (sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) ) {
    motor1.forward();
    motor2.forward();
    delay(500);
    // Perform a sharp right turn
    // Assuming motor1 is the right motor and motor2 is the left motor
    bool enoughturnRight = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0;
    while (!enoughturnRight) {
      readSensors(sensorValues);
      enoughturnRight = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0;
      motor1.setSpeed(turnSpeed);   // Set right motor speed
      motor1.backward();      // Move right motor backward for a sharper turn
      motor2.setSpeed(turnSpeed);   // Set left motor speed
      motor2.forward();       // Move left motor forward

    }
//    motor1.stop();
//    motor2.stop();

  }
//  else {
//    // If no sharp turn is detected, stop the motors
//    motor1.stop();
//    motor2.stop();
//  }
}
