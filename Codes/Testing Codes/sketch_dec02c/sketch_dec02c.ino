int leftMotorEnablePin = 6;
int leftMotorD1 = 4;
int leftMotorD2 = 5;
int leftMotorSpeedPin = 7;

int rightMotorEnablePin = 10;
int rightMotorD1 = 8;
int rightMotorD2 = 9;
int rightMotorSpeedPin = 11;

const int minPin = 2;
const int maxPin = 13;

void setup() {
  // put your setup code here, to run once:
  initializePins();
}

void loop() {
  moveForward();
  delay(200);
}

void initializePins() {
  if (!validatePins()) {
    // Handle invalid pin configuration
    while (1) {
      // Enter an infinite loop to indicate an error
    }
  }
  
  pinMode(leftMotorEnablePin, OUTPUT);
  pinMode(leftMotorD2, OUTPUT);
  pinMode(leftMotorD1, OUTPUT);
  pinMode(leftMotorSpeedPin, OUTPUT);

  pinMode(rightMotorEnablePin, OUTPUT);
  pinMode(rightMotorD2, OUTPUT);
  pinMode(rightMotorD1, OUTPUT);
  pinMode(rightMotorSpeedPin, OUTPUT);
}

bool validatePins() {
  if (
    leftMotorEnablePin < minPin  leftMotorEnablePin > maxPin 
    leftMotorD1 < minPin  leftMotorD1 > maxPin 
    leftMotorD2 < minPin  leftMotorD2 > maxPin 
    leftMotorSpeedPin < minPin  leftMotorSpeedPin > maxPin 
    rightMotorEnablePin < minPin  rightMotorEnablePin > maxPin 
    rightMotorD1 < minPin  rightMotorD1 > maxPin 
    rightMotorD2 < minPin  rightMotorD2 > maxPin 
    rightMotorSpeedPin < minPin || rightMotorSpeedPin > maxPin
  ) {
    return false; // Invalid pin configuration
  }
  return true;
}

void moveForward() {
  digitalWrite(leftMotorEnablePin, HIGH);
  digitalWrite(leftMotorD2, HIGH);
  digitalWrite(leftMotorD1, LOW);
  analogWrite(leftMotorSpeedPin, 200);

  digitalWrite(rightMotorEnablePin, HIGH);
  digitalWrite(rightMotorD2, HIGH);
  digitalWrite(rightMotorD1, LOW);
  analogWrite(rightMotorSpeedPin, 200);
}

void stopMotors() {
  digitalWrite(leftMotorEnablePin, LOW);
  digitalWrite(rightMotorEnablePin, LOW);
}
