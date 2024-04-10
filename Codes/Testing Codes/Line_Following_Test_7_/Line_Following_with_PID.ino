int leftMotorEnablePin = 3;
int leftMotorDirectionPin = 5;
int leftMotorSpeedPin = 4;
  
int rightMotorEnablePin = 6;
int rightMotorDirectionPin = 8;
int rightMotorSpeedPin = 7;

const int numSensors = 8;
const int sensorPins[numSensors] = {A0, A1, A2, A3, A4, A5,};

int setpoint = numSensors / 2;  // Center sensor as the setpoint
double Kp = 1.0;  // Proportional gain
double Ki = 0.0;  // Integral gain
double Kd = 0.0;  // Derivative gain

double previousError = 0.0;
double integral = 0.0;

void setup() {
  pinMode(leftMotorEnablePin, OUTPUT);
  pinMode(leftMotorDirectionPin, OUTPUT);
  pinMode(leftMotorSpeedPin, OUTPUT);

  pinMode(rightMotorEnablePin, OUTPUT);
  pinMode(rightMotorDirectionPin, OUTPUT);
  pinMode(rightMotorSpeedPin, OUTPUT);

  Start();
  delay(10);
}

void loop() {
  int leftSensorValue = analogRead(sensorPins[0]);
  int rightSensorValue = analogRead(sensorPins[numSensors - 1]);

  int error = rightSensorValue - leftSensorValue;

  double correction = Kp * error + Ki * integral + Kd * (error - previousError);

  // Adjust motor speeds based on correction
  int leftSpeed = 200 + correction;
  int rightSpeed = 200 - correction;

  // Clip motor speeds to a reasonable range (0 to 255)
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Set motor speeds
  analogWrite(leftMotorSpeedPin, leftSpeed);
  analogWrite(rightMotorSpeedPin, rightSpeed);

  // Update integral and previous error for the next iteration
  integral += error;
  previousError = error;
}

void Start() {
  digitalWrite(leftMotorEnablePin, HIGH);
  digitalWrite(leftMotorDirectionPin, HIGH);
  analogWrite(leftMotorSpeedPin, 255);

  digitalWrite(rightMotorEnablePin, HIGH);
  digitalWrite(rightMotorDirectionPin, HIGH);
  analogWrite(rightMotorSpeedPin, 255);
}
