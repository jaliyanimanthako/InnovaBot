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
const int analogSensorPins[analogSensorCount] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Analog pins for the remaining sensors
int IR_weight[8] = { -30, -20, -10, -5, 5, 10, 20, 30};


// PID control parameters
float Kp = 5; // Proportional term
float Ki = 0; // Integral term
float Kd = 4; // Derivative term

float error;

// PID variables
float P, I, D, previousError = 0;

float lsp, rsp;
int lfspeed =180;
int turnSpeed =100;

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
  pinMode(LED_BUILTIN , OUTPUT);

  // Start with motors stopped
  motor1.stop();
  motor2.stop();
  Serial.begin(9600);
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

float calculateError(int *sensorValues) {
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
//  // PID terms
//  P = error;
//  I += error;
//  D = error - previousError;
//  previousError = error;
// 
//  
//  // Calculate PID value
//  float pidValue = (Kp * P) + (Ki * I) + (Kd * D);
//
//
//  return pidValue;
  return error;
}

float calculatePID(float error) {
 
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

//1



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

  //stop
  int sensorValues[SensorCount];
  readSensors(sensorValues);
  
//  if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0){
//    motor1.stop();
//    motor2.stop();
// }
//  if (isImmediateTurn(sensorValues)){
//    turn(sensorValues);
//  }
// 

      float error = calculateError(sensorValues);
      float pidValue = calculatePID(error);
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
      
      if (error >0){
        motor1.setSpeed(rsp);       
        motor1.backward();
        motor2.setSpeed(lsp); 
        motor2.forward();
      }
      if (error<0){
        motor1.setSpeed(rsp);       
        motor1.backward();
        motor2.setSpeed(lsp); 
        motor2.forward();
      }
      if(error=0){
        motor1.setSpeed(180);       
        motor1.forward();
        motor2.setSpeed(180); 
        motor2.forward();
      }
  
}
      
//    PID_Linefollow(pidValue);

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
