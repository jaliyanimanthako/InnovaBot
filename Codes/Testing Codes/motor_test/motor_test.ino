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
float Kp = 9.4; // Proportional term
float Ki = 0.0; // Integral term
float Kd = 0; // Derivative term

float error;

// PID variables
float P, I, D, previousError = 0;

float lsp, rsp;
int lfspeed = 70;

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
  // put your main code here, to run repeatedly:
  motor2.setSpeed(100);
  motor2.forward();
//  delay(10000);
//  motor2.setSpeed(100);
//  motor2.backward();
//  delay(10000);
 // motor1.setSpeed(100);
//  motor1.forward();
//  delay(10000);
//  motor1.setSpeed(100);
//  motor1.backward();
//  delay(10000);
  
//  motor1.forward();
  
 

  
  

}
