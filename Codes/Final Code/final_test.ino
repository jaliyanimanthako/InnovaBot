#include <L298N.h>
#include <Arduino.h>
#include <arduinoFFT.h>
#include <Servo.h>
#include <Wire.h>

#define SAMPLES 256
#define SAMPLING_FREQUENCY 10000.0

#define AIN1 49
#define BIN1 51
#define AIN2 47
#define BIN2 53
#define PWMA 4
#define PWMB 3

// Ultrasonic sensor definitions
#define LEFT_SENSOR_TRIG A8
#define LEFT_SENSOR_ECHO A9
#define FRONT_SENSOR_TRIG A10
#define FRONT_SENSOR_ECHO A11
#define RIGHT_SENSOR_TRIG A12
#define RIGHT_SENSOR_ECHO A13

#define CASEONE 12
#define CASETWO 11
#define CASETHREE 10
#define CASEFOUR 9



#define FRONT_SENSOR_TRIG 7
#define FRONT_SENSOR_ECHO 8

const int S0 = 31;
const int S1 = 33;
const int S2 = 34;
const int S3 = 35;
const int OUT = 36;

int redFrequency, greenFrequency, blueFrequency;

// Define threshold values for red and blue
const int redThreshold = 200;  // Adjust this value based on your calibration
const int blueThreshold = 250; // Adjust this value based on your calibration

// Define maximum and minimum distances for wall following
#define MAX_DISTANCE 20
#define MIN_DISTANCE 10

arduinoFFT FFT = arduinoFFT();

Servo ENTC;

unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[SAMPLES];
double vImag[SAMPLES];

L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

const int SensorCount = 8; // Total number of sensors
const int analogSensorCount = 8; // Number of analog sensors
const int analogSensorPins[analogSensorCount] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Analog pins for the remaining sensors
int IR_weight[8] = { -30, -20, -10, -5, 5, 10, 20, 30};

float Kp = 4.7; // Proportional term
float Ki = 0.001; // Integral term
float Kd = 5; // Derivative term

float error;

// PID variables
float P, I, D, previousError = 0;

float lsp, rsp;
int lfspeed =200;
int turnSpeed = 150;
int fTime=20;

void readSensors(int *values);
float calculatePID(int *sensorValues);
void motor_drive(int left, int right);

int count=0;

bool caseone = false;
bool casetwo;
bool casethree;
bool casefour;


void setup() {
  

   Serial.begin(115200);
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));



  for (int i = 0; i < analogSensorCount; i++) {
    pinMode(analogSensorPins[i], INPUT);
  }
  
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);

  // Setting frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  // Initialize motor driver pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(LED_BUILTIN , OUTPUT);

    // Initialize ultrasonic sensor pins
  pinMode(LEFT_SENSOR_TRIG, OUTPUT);
  pinMode(LEFT_SENSOR_ECHO, INPUT);
  pinMode(FRONT_SENSOR_TRIG, OUTPUT);
  pinMode(FRONT_SENSOR_ECHO, INPUT);
  pinMode(RIGHT_SENSOR_TRIG, OUTPUT);
  pinMode(RIGHT_SENSOR_ECHO, INPUT);
  pinMode(CASEONE, INPUT);
  pinMode(CASETWO, INPUT);
  pinMode(CASETHREE, INPUT);
  pinMode(CASEFOUR, INPUT);


  pinMode(13, OUTPUT);

  // Start with motors stopped
  motor1.stop();
  motor2.stop();
}
void lineFollowing();
void wallFollowing();
void boxDetection();
void colorboxDetection();
void soundDetection();
void arm();
int getdistance(int trigPin, int echoPin);
void lineFollowing();
void readSensors(int *values);
float calculatePID(int *sensorValues);
void motor_drive(int left, int right);
void setup();
void loop();
bool isImmediateTurn(int *sensorValues);
void turn(int *sensorValues);
void turnLeft(int *sensorValues);
void turnRight(int *sensorValues);
void loop() {
  caseone = digitalRead(CASEONE);
  casetwo= digitalRead(CASETWO);
  casethree= digitalRead(CASETHREE);
  casefour= digitalRead(CASEFOUR);
  if(CASEONE == HIGH){
    caseone = true;
    }
  switch(count){
    case (0):
      lineFollowing();
      break;
    case (1):
      digitalWrite(13,HIGH);
      delay(100);
      digitalWrite(13,LOW);
      delay(100);
      wallFollowing();
      break;
    case (2):
    digitalWrite(13,HIGH);
      delay(400);
      digitalWrite(13,LOW);
      delay(400);
      lineFollowing();
      break;
    case (3):
      lineFollowing();
      //t eka gena balanna
      break;
    case (4):
      boxDetection();
      break;
    case (5):
      colorboxDetection();
      break;
    case (6):
      soundDetection();
      break;
    default:
       lineFollowing();
    }

}

void soundDetection(){
  
  /* Sampling */
  for (int i = 0; i < SAMPLES; i++) {
    microseconds = micros();

    vReal[i] = analogRead(A14);
    vImag[i] = 0;

    while (micros() < (microseconds + sampling_period_us)) {
      // Wait
    }
  }

  /* Compute FFT */
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  /* Find the peak frequency */
  int peakIndex = 0;
  double peak = 0.0;
  for (int i = 1; i < SAMPLES / 2; i++) {
    if (vReal[i] > peak) {
      peak = vReal[i];
      peakIndex = i;
    }
  }

  double frequency = peakIndex * SAMPLING_FREQUENCY / SAMPLES;

  /* Print the result */
  Serial.print("Frequency: ");
  Serial.print(frequency);
  Serial.println(" Hz");

  delay(200);

  if (frequency > 1200 && frequency < 1300){
      motor1.stop();
      motor2.stop();
    }else{
     //lineFollowing();
    }
  
  }

void colorboxDetection(){
  int frontDistance = getdistance(FRONT_SENSOR_TRIG, FRONT_SENSOR_ECHO);
  if(frontDistance <  4){

    //stop()

  // Read the color frequencies
  redFrequency = pulseIn(OUT, LOW);
  greenFrequency = pulseIn(OUT, LOW);
  blueFrequency = pulseIn(OUT, LOW);


  // Check if the color is closer to red or blue based on thresholds
  if (redFrequency && greenFrequency && blueFrequency <redThreshold ) {
//    Serial.println("Detected Red Color!");
  } else if ((redFrequency && greenFrequency && blueFrequency< blueThreshold) &&(redFrequency && greenFrequency && blueFrequency> redThreshold))
  {
//    Serial.println("Detected Blue Color!");
  } 
  
   // Delay for readability
}
}

int getdistance(int trigPin, int echoPin) {
  // Generate ultrasonic pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the pulse duration
  long duration = pulseIn(echoPin, HIGH);

  // Convert the duration to distance
  int distance = duration * 0.034 / 2;

  return distance;
}

  
  

void arm (){
  ENTC.attach(20);


  ENTC.write(-50);
  delay(1000);
  ENTC.write(90); //degrees
  delay(1000);
  
  }

void boxDetection (){
  int frontDistance = getDistance(FRONT_SENSOR_TRIG, FRONT_SENSOR_ECHO);
  

  int safeDistance = MAX_DISTANCE + 5;
  int turnThreshold = 10;

  if (frontDistance < safeDistance ) {
    //turn 90 degree
    arm();
  } else {
    lineFollowing();
  }
  }



void wallFollowing (){
  int leftDistance = getDistance(LEFT_SENSOR_TRIG, LEFT_SENSOR_ECHO);
  int frontDistance = getDistance(FRONT_SENSOR_TRIG, FRONT_SENSOR_ECHO);
  int rightDistance = getDistance(RIGHT_SENSOR_TRIG, RIGHT_SENSOR_ECHO);

  int safeDistance = MAX_DISTANCE + 5;
  int turnThreshold = 10;

  if (frontDistance < safeDistance || rightDistance < safeDistance || leftDistance < safeDistance) {
    if (frontDistance > rightDistance && frontDistance > leftDistance) {
      motor1.forward();
      motor2.forward();
    } else {
      if (rightDistance > leftDistance + turnThreshold) {
        motor1.forward();
         motor2.backward();
      } else if (leftDistance > rightDistance + turnThreshold) {
        motor2.forward();
         motor1.backward();
      }
    }
  } else {
    lineFollowing();
  }
  }
 
int getDistance(int trigPin, int echoPin) {
  // Generate ultrasonic pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the pulse duration
  long duration = pulseIn(echoPin, HIGH);

  // Convert the duration to distance
  int distance = duration * 0.034 / 2;

  return distance;
}
  
void lineFollowing (){
  
  int sensorValues[SensorCount];
  readSensors(sensorValues);
//  if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0){
//    motor1.stop();
//    motor2.stop();
//  }

 // else
  if (isImmediateTurn(sensorValues)) {
    turn(sensorValues);
    
  }
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0){
    motor1.stop();
    motor2.stop();
   
  }else if(sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1){

    delay(50);
    if(sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1){
        count += 1;
        motor1.stop();
        motor2.stop();
        delay(100);  
      }
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
//    Serial.print(analogRead(analogSensorPins[i]));
//    Serial.print(" ");

  }
//  Serial.print(analogRead(IR_L)> 100 ? 0 : 1);
//  Serial.print(" ");
//  Serial.print(analogRead(IR_R)> 100 ? 0 : 1);
//  Serial.println(" ");
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
  bool turnLeft = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1;
  bool turnRight = sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  return (turnLeft || turnRight);
}
void turn(int *sensorValues) {
  // Check if A0, A1, A2, A3 are detecting the line (indicating a left turn)
  if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) {
    motor1.setSpeed(lfspeed);   // Set right motor speed
    motor1.forward();       // Move right motor forward
    motor2.setSpeed(lfspeed);   // Set left motor speed
    motor2.forward();  
    delay(650);
    turnLeft(sensorValues);

  }
  // Check if A4, A5, A6, A7 are detecting the line (indicating a right turn)
  else if (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) {
    motor1.setSpeed(lfspeed);   // Set right motor speed
    motor1.forward();       // Move right motor forward
    motor2.setSpeed(lfspeed);   // Set left motor speed
    motor2.forward();
    delay(650);  
    turnRight(sensorValues);
  }
  else {
    // If no sharp turn is detected, stop the motors
   
    delay(500);
    motor1.stop();
    motor2.stop();
  }
}

void turnLeft(int *sensorValues) {
  // Perform a sharp left turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  while (!enoughturn && !whitedetect) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0;
    whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (enoughturn || whitedetect) {
      break;
    }
    motor1.setSpeed(turnSpeed);   // Set right motor speed
    motor1.backward();       // Move right motor forward
    motor2.setSpeed(turnSpeed);   // Set left motor speed
    motor2.forward();      // Move left motor backward for a sharper turn
    digitalWrite(26, HIGH);
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
}

void turnRight(int *sensorValues) {
  // Perform a sharp right turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  while (!enoughturn && !whitedetect) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0;
    whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (enoughturn || whitedetect) {
      break;
    }
    motor1.setSpeed(turnSpeed);   // Set right motor speed
    motor1.forward();       // Move right motor forward
    motor2.setSpeed(turnSpeed);   // Set left motor speed
    motor2.backward();      // Move left motor backward for a sharper turn
    digitalWrite(28, HIGH);
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
}
