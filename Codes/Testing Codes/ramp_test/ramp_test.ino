int leftMotorEnablePin = 6;
int leftMotorD1 = 4;
int leftMotorD2 = 5;
int leftMotorSpeedPin = 7;
  
int rightMotorEnablePin = 10;
int rightMotorD1 = 8;
int rightMotorD2 = 9;
int rightMotorSpeedPin =11;



void setup() {
  // put your setup code here, to run once:
  pinMode(leftMotorEnablePin, OUTPUT);
  pinMode(leftMotorD1, OUTPUT);
  pinMode(leftMotorD2, OUTPUT);
  pinMode(leftMotorSpeedPin, OUTPUT);

  pinMode(rightMotorEnablePin, OUTPUT);
  pinMode(rightMotorD1, OUTPUT);
  pinMode(rightMotorD2, OUTPUT);
  pinMode(rightMotorSpeedPin, OUTPUT);
}

void loop() {
  moveForward();
  delay(200);
  

}

void moveForward() {
  digitalWrite(leftMotorEnablePin,HIGH);
  digitalWrite(leftMotorD1, HIGH);
  digitalWrite(leftMotorD2, LOW);
  analogWrite(leftMotorSpeedPin, 200);

  digitalWrite(rightMotorEnablePin,HIGH);
  digitalWrite(rightMotorD2, HIGH);
  digitalWrite(rightMotorD1, LOW);
  analogWrite(rightMotorSpeedPin, 200);
}
