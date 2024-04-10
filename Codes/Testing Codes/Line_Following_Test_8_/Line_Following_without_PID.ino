int leftMotorEnablePin = 6;
int leftMotorD1 = 2;
int leftMotorD2 = 4;
int leftMotorSpeedPin = 9;
  
int rightMotorEnablePin = 12;
int rightMotorD1 = 7;
int rightMotorD2 = 8;
int rightMotorSpeedPin =10;

const int numSensors = 6;
const int sensorPins[numSensors] = {A0, A1, A2, A3, A4, A5};

int leftSensorValue = analogRead(sensorPins[0]);
int rightSensorValue = analogRead(sensorPins[numSensors - 1]);


void setup() {
  // put your setup code here, to run once:
  pinMode(leftMotorEnablePin, OUTPUT);
  pinMode(leftMotorD1, OUTPUT);
  pinMode(leftMotorD2, OUTPUT);
  pinMode(leftMotorSpeedPin, OUTPUT);

  pinMode(rightMotorEnablePin, OUTPUT);
  pinMode(rightMotorD1, OUTPUT);
  pinMode(rightMotorD1, OUTPUT);
  pinMode(rightMotorSpeedPin, OUTPUT);
  Start();
  delay(10);
}

void loop() {
  // put your main code here, to run repeatedly:
  int leftSensorValue = analogRead(sensorPins[0]);
  int rightSensorValue = analogRead(sensorPins[numSensors - 1]);

  // Simple line-following logic
  if (leftSensorValue < 500 && rightSensorValue < 500) {
    
    // Both sensors off the line, move forward
    moveForward();
  } else if (leftSensorValue < 500) {
    // Left sensor off the line, turn right
    turnRight();
  } else if (rightSensorValue < 500) {
    // Right sensor off the line, turn left
    turnLeft();
  } else {
    // Both sensors on the line, stop
    stopMotors();
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

void moveForward() {
  digitalWrite(leftMotorEnablePin,HIGH);
  digitalWrite(leftMotorDirectionPin, HIGH);
  analogWrite(leftMotorSpeedPin, 200);

  digitalWrite(rightMotorEnablePin,HIGH);
  digitalWrite(rightMotorDirectionPin, HIGH);
  analogWrite(rightMotorSpeedPin, 200);
}

// Function to turn the robot left
void turnLeft() {
  digitalWrite(leftMotorEnablePin,HIGH);
  digitalWrite(leftMotorDirectionPin, LOW);
  analogWrite(leftMotorSpeedPin, 200);

  digitalWrite(rightMotorEnablePin,HIGH);
  digitalWrite(rightMotorDirectionPin, HIGH);
  analogWrite(rightMotorSpeedPin, 200);
}

// Function to turn the robot right
void turnRight() {
  digitalWrite(leftMotorEnablePin,HIGH);
  digitalWrite(leftMotorDirectionPin, HIGH);
  analogWrite(leftMotorSpeedPin, 150);

  digitalWrite(rightMotorEnablePin,HIGH);
  digitalWrite(rightMotorDirectionPin, LOW);
  analogWrite(rightMotorSpeedPin, 150);
}

// Function to stop the motors
void stopMotors() {
  digitalWrite(leftMotorEnablePin,HIGH);
  digitalWrite(rightMotorEnablePin,HIGH);
  digitalWrite(leftMotorSpeedPin, LOW);
  digitalWrite(rightMotorSpeedPin, LOW);
}
