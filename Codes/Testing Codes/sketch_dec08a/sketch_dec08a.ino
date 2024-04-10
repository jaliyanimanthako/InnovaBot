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

#define LEFT_SENSOR_TRIG A8
#define LEFT_SENSOR_ECHO A9
#define FRONT_SENSOR_TRIG A10
#define FRONT_SENSOR_ECHO A11
#define RIGHT_SENSOR_TRIG A12
#define RIGHT_SENSOR_ECHO A13

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


// Define maximum and minimum distances for wall following
#define MAX_DISTANCE 20  // Maximum distance to the wall (adjust as needed)
#define MIN_DISTANCE 10  // Minimum distance to the wall (adjust as needed)


float lfspeed = 150;
float turnspeed = 200; 

void setup() {
  // Initialize motor pins as output
//  pinMode(LEFT_MOTOR_1, OUTPUT);
//  pinMode(LEFT_MOTOR_2, OUTPUT);
//  pinMode(RIGHT_MOTOR_1, OUTPUT);
//  pinMode(RIGHT_MOTOR_2, OUTPUT);

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

void loop() {
  // Read distances from sensors
  int leftDistance = getDistance(LEFT_SENSOR_TRIG, LEFT_SENSOR_ECHO);
  int frontDistance = getDistance(FRONT_SENSOR_TRIG, FRONT_SENSOR_ECHO);
  int rightDistance = getDistance(RIGHT_SENSOR_TRIG, RIGHT_SENSOR_ECHO);

  // Adjustments for better obstacle avoidance
  int safeDistance = MAX_DISTANCE + 5;  // Add a safety margin
  int turnThreshold = 10;  // Minimum difference in distances to initiate a turn

  // Obstacle avoidance logic
  if (frontDistance < safeDistance || rightDistance < safeDistance || leftDistance < safeDistance) {
    if (frontDistance > rightDistance && frontDistance > leftDistance) {
      // Move forward
      moveForward();
    } else {
      // Determine the safest direction to turn
      if (rightDistance > leftDistance + turnThreshold) {
        // Turn right
        turnRight();
      } else if (leftDistance > rightDistance + turnThreshold) {
        // Turn left
        turnLeft();
      } 
    }
  } else {
    // No obstacle, move forward
    moveForward();
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

void moveForward() {
  motor1.setSpeed(lfspeed);   // Set right motor speed
  motor1.forward();       // Move right motor forward
  motor2.setSpeed(lfspeed);   // Set left motor speed
  motor2.forward();  
}  

void turnLeft() {
  motor1.setSpeed(turnspeed);   // Set right motor speed
  motor1.forward();       // Move right motor forward
  motor2.setSpeed(turnspeed);   // Set left motor speed
  motor2.backward();  
}

void turnRight() {
  motor1.setSpeed(turnspeed);   // Set right motor speed
  motor1.backward();       // Move right motor forward
  motor2.setSpeed(turnspeed);   // Set left motor speed
  motor2.forward();  
}
