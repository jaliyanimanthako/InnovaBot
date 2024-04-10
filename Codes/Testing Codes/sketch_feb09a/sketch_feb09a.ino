#include <Arduino.h>

const int analogInputPin = 35;  // GPIO36, you can use any other GPIO that supports ADC
const int bufferSize = 1024;     // Size of the audio buffer
uint16_t buffer[bufferSize];     // Audio buffer

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Recording
  for (int i = 0; i < bufferSize; i++) {
    buffer[i] = analogRead(analogInputPin);  // Read analog input and store in buffer
    delayMicroseconds(100);  // Adjust delay as needed for sampling frequency
  }

  // Playback
  for (int i = 0; i < bufferSize; i++) {
    dacWrite(analogInputPin, buffer[i]);  // Output audio data from buffer to DAC
    delayMicroseconds(100);  // Adjust delay as needed for playback frequency
  }
}
