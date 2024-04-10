#include <L298N.h>
#include <Arduino.h>

//////////////
// Have to redefine those below pins as we plugged
#define AIN1 49
#define BIN1 51
#define AIN2 47
#define BIN2 53
#define PWMA 3
#define PWMB 4
//motor1=right
////////

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB,  BIN1, BIN2);

void setup() {
  // put your setup code here, to run once:
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(28, OUTPUT);
  

  // Start with motors stopped
  motor1.stop();
  motor2.stop();
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  motor2.setSpeed(70);   // Set right motor speed
  motor2.backward();
 
  delay(1000);
  
  
}
