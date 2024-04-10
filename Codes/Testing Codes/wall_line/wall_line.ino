
#define LEFT_SENSOR_TRIG A8
#define LEFT_SENSOR_ECHO A9
#define FRONT_SENSOR_TRIG A10
#define FRONT_SENSOR_ECHO A11
#define RIGHT_SENSOR_TRIG A12
#define RIGHT_SENSOR_ECHO A13
#define line 10
#define wall 9
#define MAX_DISTANCE 20
#define MIN_DISTANCE 10

const int motorRPin1 = 47; // signal pin 1 for the right motor, connect to IN1               
const int motorRPin2 = 49; // signal pin 2 for the right motor, connect to IN2
const int motorREnable = 4; // enable pin for the right motor (needs to be PWM enabled)

const int motorLPin1 = 51; // signal pin 1 for the left motor, connect to IN3 (was 5 - need to change)             
const int motorLPin2 = 53; // signal pin 2 for the left motor, connect to IN4
const int motorLEnable = 3; // enable pin for the left motor (needs to be PWM enabled)

/* Define the pins for the IR receivers */
const int irPins[8] = {A0, A1, A3, A0, A4, A5, A6, A7};

/* Define values for the IR Sensor readings */

// an array to hold values from analogRead on the ir sensor (0-1023)
int irSensorAnalog[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// an array to hold boolean values (1/0) for the ir sensors, based on the analog read and the predefined threshold
int irSensorDigital[8] = {0, 0, 0, 0, 0, 0, 0, 0}; 

// the value above which we determine an IR sensor reading indicates the sensor is over a line
int threshold = 700; 

// binary representation of the sensor reading from left to right
int irSensors = B00000000; 

// sensors detecting the line
int count = 0; 

// a score to determine deviation from the line [-180 ; +180]. Negative means the robot is left of the line.
int error = 0;

// store the last value of error
int errorLast = 0;  

// a correction value, based on the error that is used to change motor speed with PWM
int correction = 0; 

// keep track of the laps
int lap = 0; 

/* Set up maximum speed and speed for turning (to be used with PWM) */

// PWM to control motor speed [0 - 255]
int maxSpeed = 255; 

/* variables to keep track of current speed of motors */
int motorLSpeed = 0;
int motorRSpeed = 0;

void setup() {
  Serial.begin(9600);
  pinMode(motorLPin1, OUTPUT);        
  pinMode(motorLPin2, OUTPUT);
  pinMode(motorLEnable, OUTPUT);
  
  pinMode(motorRPin1, OUTPUT);        
  pinMode(motorRPin2, OUTPUT);
  pinMode(motorREnable, OUTPUT);

  pinMode(line, INPUT);
  pinMode(wall, INPUT);
  pinMode(43, INPUT);
  Serial.println(digitalRead(43));
   
  /* Set up IR sensor pins as input */
  for (int i = 0; i < 8; i++) {
    pinMode(irPins[i], INPUT);
  }
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); 
  TCCR0B = _BV(CS00); 
  // put your setup code here, to run once:

}


void linefollow() {
  Scan();
  UpdateError();
  UpdateCorrection();
  Drive();
  
  // put your main code here, to run repeatedly:

}
void Scan() {
  // Initialize counters, sums, etc.
  count = 0;
  irSensors = B00000000;
    
  for (int i = 0; i < 8; i++) {
    irSensorAnalog[i] = analogRead(irPins[i]);

    if (irSensorAnalog[i] < threshold) { // Change the condition here
      irSensorDigital[i] = 1;
    } else {
      irSensorDigital[i] = 0;
    }
    
    Serial.print(irSensorAnalog[i]);
    Serial.print("|");
    count = count + irSensorDigital[i];
    int b = 7 - i;
    irSensors = irSensors + (irSensorDigital[i] << b);
  }    
}
void linefollow();
void wallFollowing();

void UpdateError() {
  errorLast = error;  
  
  switch (irSensors) {
    case B00000000:
      if (errorLast < 0) {
        error = -180;
      } else if (errorLast > 0) {
        error = 180;
      }
      break;

    case B10000000: // leftmost sensor on the line
      error = -150;
      break;

    case B01000000:
      error = -120;
      break;

    case B00100000: 
      error = -90;
      break;

    case B00010000:  
      error = -60;
      break;

    case B00001000: 
      error = -30;
      break;

    case B00000100: 
      error = 30;
      break;

    case B00000010: 
      error = 60;
      break;

    case B00000001: // rightmost sensor on the line
      error = 150;
      break;

    /* Add cases for 2, 3, 4, 5, 6, 7 sensors on the line as needed */

    case B11000000:
      error = -135;
      break;

    case B01100000:
      error = -105;
      break;

    case B00110000: 
      error = -75;
      break;

    case B00011000: 
      error = -45;
      break;

    case B00001100:
      error = -15;
      break;

    case B00000110:
      error = 15;
      break;

    case B00000011:
      error = 45;
      break;

    case B11100000:
    case B01110000:
      error = -150;
      break;

    case B00011100:
    case B00111000:
      error = 150;
      break;

    case B11110000:
      error = -150;
      break;

    case B11101000:
      error = -150;
      break;

    case B00111100:
      error = 150;
      break;

    case B01011100:
      error = 150;
      break;

    case B11111000:
      error = -150;
      break;

    case B01111100:
      error = +150;
      break;

    case B11111100:
      lap = lap + 1; // increment laps when start/finish line is crossed
      error = 0;
      break;

    /* Add cases for 8 sensors on the line as needed */

    default:
      error = errorLast;
  }
}

void UpdateCorrection() {
  if (error >= 0 && error < 30) {
    correction = 0;
  } else if (error >= 30 && error < 60) {
    correction = 15;
  } else if (error >= 60 && error < 90) {
    correction = 40;
  } else if (error >= 90 && error < 120) {
    correction = 55;
  } else if (error >= 120 && error < 150) {
    correction = 75;
  } else if (error >= 150 && error < 180) {
    correction = 255;
  } else if (error >= 180) {
    correction = 305;
  }

  if (error <= 0 && error > -30) {
    correction = 0;
  } else if (error <= -30 && error > -60) {
    correction = -15;
  } else if (error <= -60 && error > -90) {
    correction = -40;
  } else if (error <= -90 && error > -120) {
    correction = -55;
  } else if (error <= -120 && error > -150) {
    correction = -75;
  } else if (error <= -150 && error > -180) {
    correction = -255;
  } else if (error <= -180) {
    correction = -305;
  }

  if (correction >= 0) {
    motorRSpeed = maxSpeed - correction;
    motorLSpeed = maxSpeed;
  } else if (correction < 0) {
    motorRSpeed = maxSpeed;
    motorLSpeed = maxSpeed + correction;
  }
}

void Drive() {
  if (motorRSpeed > 255) {
    motorRSpeed = 255;
  } else if (motorRSpeed < -255) {
    motorRSpeed = -255;
  }

  if (motorLSpeed > 255) {
    motorLSpeed = 255;
  } else if (motorLSpeed < -255) {
    motorLSpeed = -255;
  }

  if (motorRSpeed > 0) { // right motor forward (using PWM)
    analogWrite(motorREnable, motorRSpeed);
    digitalWrite(motorRPin1, HIGH);
    digitalWrite(motorRPin2, LOW);
  } else if (motorRSpeed < 0) { // right motor reverse (using PWM)
    analogWrite(motorREnable, abs(motorRSpeed));
    digitalWrite(motorRPin1, LOW);
    digitalWrite(motorRPin2, HIGH);
  } else if (motorRSpeed == 0) { // right motor fast stop
    digitalWrite(motorREnable, HIGH);
    digitalWrite(motorRPin1, LOW);
    digitalWrite(motorRPin2, LOW);
  }

  if (motorLSpeed > 0) { // left motor forward (using PWM)
    analogWrite(motorLEnable, motorLSpeed);
    digitalWrite(motorLPin1, HIGH);
    digitalWrite(motorLPin2, LOW);
  } else if (motorLSpeed < 0) { // left motor reverse (using PWM)
    analogWrite(motorLEnable, abs(motorLSpeed));
    digitalWrite(motorLPin1, LOW);
    digitalWrite(motorLPin2, HIGH);
  } else if (motorLSpeed == 0) { // left motor fast stop
    digitalWrite(motorLEnable, HIGH);
    digitalWrite(motorLPin1, LOW);
    digitalWrite(motorLPin2, LOW);
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
      analogWrite(motorREnable, motorRSpeed);
      digitalWrite(motorRPin1, HIGH);
      digitalWrite(motorRPin2, LOW);
      analogWrite(motorLEnable, motorLSpeed);
      digitalWrite(motorLPin1, HIGH);
      digitalWrite(motorLPin2, LOW);
    } else {
      if (rightDistance > leftDistance + turnThreshold) {
      analogWrite(motorREnable, motorRSpeed);
      digitalWrite(motorRPin1, HIGH);
      digitalWrite(motorRPin2, LOW);
      analogWrite(motorLEnable, abs(motorLSpeed));
      digitalWrite(motorLPin1, LOW);
      digitalWrite(motorLPin2, HIGH);
      } else if (leftDistance > rightDistance + turnThreshold) {
        analogWrite(motorLEnable, motorLSpeed);
        digitalWrite(motorLPin1, HIGH);
        digitalWrite(motorLPin2, LOW);
        analogWrite(motorREnable, abs(motorRSpeed));
        digitalWrite(motorRPin1, LOW);
        digitalWrite(motorRPin2, HIGH);
      }
    }
  } else {
    linefollow();
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

void loop() {
 
 
  wallFollowing ();
    
  
  
}
