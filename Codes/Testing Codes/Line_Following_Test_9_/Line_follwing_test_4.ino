// Motor driver pins
const int leftMotorEnablePin = 34;
const int leftMotorDirectionPin = 8;
const int leftMotorSpeedPin = 12;

const int rightMotorEnablePin = 32;
const int rightMotorDirectionPin = 4;
const int rightMotorSpeedPin = 11;

// Sensor pins
const int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// PID constants
double kp = 0.5;  // proportional gain
double ki = 0.0;  // integral gain
double kd = 0.2;  // derivative gain

// PID variables
int prevError = 0;
int integral = 0;

void setup() {
  // Motor setup
  pinMode(leftMotorEnablePin, OUTPUT);
  pinMode(leftMotorDirectionPin, OUTPUT);
  pinMode(leftMotorSpeedPin, OUTPUT);

  pinMode(rightMotorEnablePin, OUTPUT);
  pinMode(rightMotorDirectionPin, OUTPUT);
  pinMode(rightMotorSpeedPin, OUTPUT);
  Start();
  delay(10);
  
  digitalWrite(leftMotorEnablePin,HIGH);
 
  digitalWrite(rightMotorEnablePin,HIGH);
  
}

void loop() {
  // Read sensor values
  int sensorValues[8];
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }

  // Calculate weighted average position
  int weightedSum = 0;
  int total = 0;
  for (int i = 0; i < 8; i++) {
    weightedSum += i * sensorValues[i];
    total += sensorValues[i];
  }

  double position = (double)weightedSum / total;

  // Calculate error
  int error = 3.5 - position;  // Assuming center value is 3.5

  // Calculate integral
  integral += error;

  // Calculate derivative
  int derivative = error - prevError;

  // Calculate PID output
  double pidOutput = kp * error + ki * integral + kd * derivative;

  // Update previous error
  prevError = error;

  // Motor speed and direction control
  int leftSpeed = 100 + pidOutput;
  int rightSpeed = 100 - pidOutput;

  // Clip motor speeds to valid range (0 to 255)
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Apply motor speeds
  
  // Direction control
  if (error > 0) {
    // Adjust as needed based on your motor driver and setup
    analogWrite(rightMotorSpeedPin, rightSpeed);


  } else {
    // Adjust as needed based on your motor driver and setup
    analogWrite(leftMotorSpeedPin, rightSpeed);

   
  }
}

void Start() {
  digitalWrite(leftMotorEnablePin,HIGH);
  digitalWrite(leftMotorDirectionPin, HIGH);
  analogWrite(leftMotorSpeedPin, 255);

  digitalWrite(rightMotorEnablePin,HIGH);
  digitalWrite(rightMotorDirectionPin, HIGH);
  analogWrite(rightMotorSpeedPin, 255);
}
