#define AIN1 49
#define BIN1 51
#define AIN2 47
#define BIN2 53
#define PWMA 4
#define PWMB 3

const int numSensors = 8;
const int sensorPins[numSensors] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Calibrated sensor values for non-reflective surface
int sensorMin[numSensors];

// Calibrated sensor values for reflective surface (over the line)
int sensorMax[numSensors];

void setup() {
  Serial.begin(9600);
  delay(1000);
  calibrateSensors();
}

void loop() {
  int sensorValues[numSensors];

  // Read sensor values
  readSensorValues(sensorValues);

  // Check if a sensor is over the line based on the threshold
  for (int i = 0; i < numSensors; i++) {
    if (sensorValues[i] > (sensorMax[i] + sensorMin[i]) / 2) {
      // Sensor is over the line
      Serial.print("1 ");
    } else {
      // Sensor is not over the line
      Serial.print("0 ");
    }
  }
  Serial.println();
  
  delay(100);
}

void calibrateSensors() {
  Serial.println("Calibrating sensors. Ensure the sensor array is over a non-reflective surface.");
  delay(2000);

  // Calibration for non-reflective surface
  readSensorValues(sensorMin);

  Serial.println("Place the sensor array over a reflective surface (over the line) and press enter.");
  while (!Serial.available()) {
    // Wait for user input
  }
  Serial.read(); // Consume the input buffer

  Serial.println("Calibrating sensors. Ensure the sensor array is over the reflective surface (over the line).");
  delay(2000);

  // Calibration for reflective surface (over the line)
  readSensorValues(sensorMax);

  Serial.println("Calibration complete.");
  Serial.println("Place the sensor array over the surface you want to follow and press enter.");
  while (!Serial.available()) {
    // Wait for user input
  }
  Serial.read(); // Consume the input buffer
}

void readSensorValues(int* values) {
  for (int i = 0; i < numSensors; i++) {
    values[i] = analogRead(sensorPins[i]);
  }
}
