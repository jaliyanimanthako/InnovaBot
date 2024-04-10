const int numSensors = 8;
const int sensorPins[numSensors] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Arrays to store min and max values for each sensor
int sensorMin[numSensors] ={0,0,0,0,0,0,0,0};
int sensorMax[numSensors]={0,0,0,0,0,0,0,0};

void setup() {
  Serial.begin(9600);
  delay(1000);
  calibrateSensors();
  for(int i = 1; i < 8; i++){
  Serial.print(sensorMax[i]);
  }
  
}

void loop() {
  // Your line-following code goes here
  
  delay(100);
}

void calibrateSensors() {
  Serial.println("Rotate the robot to expose each sensor to the non-reflective and reflective surfaces.");
  
  Serial.read(); // Consume the input buffer

  for (int i = 0; i < numSensors; i++) {
    Serial.print("Calibrating sensor ");
    Serial.println(i);

    // Measure min and max values during a 1-second interval
   

    int minValue = 1023;
    int maxValue = 0;

    
    int sensorValue = analogRead(sensorPins[i]);

    if (sensorValue > maxValue) {
      maxValue = sensorValue;
    }
    if (sensorValue < minValue) {
      minValue = sensorValue;
      }

    // Store min and max values in the arrays
    sensorMin[i] = minValue;
    sensorMax[i] = maxValue;
    Serial.println(sensorMin[i]);
    Serial.println(sensorMax[i]);
    
    

    Serial.println("Sensor calibration complete.");
  }
  

  Serial.println("Calibration complete.");
  Serial.println("Place the sensor array over the surface you want to follow and press enter.");
  while (!Serial.available()) {
    // Wait for user input
  }
  Serial.read(); // Consume the input buffer
}
