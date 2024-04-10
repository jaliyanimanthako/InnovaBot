// Motor Driver Pins
const int leftMotorEnablePin = 34;
const int leftMotorDirectionPin = 8;
const int leftMotorSpeedPin = 12;

const int rightMotorEnablePin = 32;
const int rightMotorDirectionPin = 4;
const int rightMotorSpeedPin = 11;

// QTR Sensor Pins
const int numSensors = 8;
const int sensorPins[numSensors] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Motor Speeds
const int baseSpeed = 150;
int leftSpeed = baseSpeed;
int rightSpeed = baseSpeed;

void setup() {
  // Motor Driver Setup
  pinMode(leftMotorEnablePin, OUTPUT);
  pinMode(leftMotorDirectionPin, OUTPUT);
  pinMode(leftMotorSpeedPin, OUTPUT);

  pinMode(rightMotorEnablePin, OUTPUT);
  pinMode(rightMotorDirectionPin, OUTPUT);
  pinMode(rightMotorSpeedPin, OUTPUT);
}

void loop() {
  // Read sensor values
  int sensorValues[numSensors];
  for (int i = 0; i < numSensors; ++i) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }

  // Calculate error
  int error = calculateError(sensorValues);

  // Adjust motor speeds based on error
  leftSpeed = baseSpeed - error;
  rightSpeed = baseSpeed + error;

  // Clip motor speeds to a reasonable range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Set motor speeds
  analogWrite(leftMotorSpeedPin, leftSpeed);
  analogWrite(rightMotorSpeedPin, rightSpeed);

  // Line-following logic
  if (error > 0) {
    // Turn right
    turnRight();
  } else {
    // Turn left
    turnLeft();
  }
}

void turnRight() {
  digitalWrite(leftMotorDirectionPin, HIGH);
  digitalWrite(rightMotorDirectionPin, LOW);
  analogWrite(leftMotorSpeedPin, baseSpeed);
  analogWrite(rightMotorSpeedPin, baseSpeed);
}

void turnLeft() {
  digitalWrite(leftMotorDirectionPin, LOW);
  digitalWrite(rightMotorDirectionPin, HIGH);
  analogWrite(leftMotorSpeedPin, baseSpeed);
  analogWrite(rightMotorSpeedPin, baseSpeed);
}

void moveForward() {
  digitalWrite(leftMotorDirectionPin, HIGH);
  digitalWrite(rightMotorDirectionPin, HIGH);
  analogWrite(leftMotorSpeedPin, baseSpeed);
  analogWrite(rightMotorSpeedPin, baseSpeed);
}

int calculateError(const int sensorValues[]) {
  // You may need to adjust these values based on your specific setup
  const int threshold = 500;  // Adjust threshold based on sensor characteristics
  const int centerSensor = numSensors / 2;

  int weightedSum = 0;
  int totalWeight = 0;

  for (int i = 0; i < numSensors; ++i) {
    int weight = i - centerSensor;
    totalWeight += weight;
    if (sensorValues[i] > threshold) {
      weightedSum += weight;
    }
  }

  if (totalWeight == 0) {
    return 0;  // Avoid division by zero
  }

  // Calculate error as the weighted sum divided by the total weight
  return (weightedSum * 100) / totalWeight;
}
