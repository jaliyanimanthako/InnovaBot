#define IR1 A0
#define IR2 A1
#define IR3 A2
#define IR4 A3
#define IR5 A4
#define IR6 A5
#define IR7 A6
#define IR8 A7

#define ENA 5
#define motorInput1 7
#define motorInput2 8
#define motorInput3 9
#define motorInput4 10
#define ENB 6

#define MAX_SPEED 150

int MotorBasespeed1 = 100; 

int IR_val[] = {0, 0, 0, 0, 0, 0, 0, 0};
int IR_weights[8] = {-20, -15, -10, -5, 5, 10, 15, 20};

int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speedAdjust = 0;

float P, I, D;
float error = 0;
float previousError = 0;
float Kp = 4;
float Kd = 0;
float Ki = 0;

void PID_control();
void read_IR();
void set_speed();
void set_forward();

void setup() {
  Serial.begin(115200);
  
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);
  pinMode(IR7, INPUT);
  pinMode(IR8, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);

  set_forward();
  delay(2000);
}

void loop() {
  
  read_IR();
  for (int i = 0; i < 8; i++) {
    Serial.print(IR_val[i]); // Print the sensor values to the serial monitor
    Serial.print('\t'); // Add a tab character for formatting
    }
  Serial.println(); // Add a line break
  delay(100); // Wait for a short time before reading the sensor again

  if (IR_val[0] >100 && IR_val[1] >100 && IR_val[2] >100 && IR_val[3] >100 && IR_val[4] >100 && IR_val[5] >100 && IR_val[6] >100 && IR_val[7] >100) {
    // Stop the motors when all sensors detect black
    stop();
  } else {
    PID_control();
    set_speed();
  }
}

void PID_control() {
  error = 0;

  for (int i = 0; i < 8; i++) {
    error += IR_weights[i] * IR_val[i];
  }
  P = error;
  I = I + error; 
  D = error - previousError;

  previousError = error;

  speedAdjust = (Kp * P + Ki * I + Kd * D);

  LMotorSpeed = 150 - speedAdjust;
  RMotorSpeed = 150 + speedAdjust;

  if (LMotorSpeed < 0) {
    LMotorSpeed = 0;
  }
  if (RMotorSpeed < 0) {
    RMotorSpeed = 0;
  }
  if (RMotorSpeed > MAX_SPEED) {
    RMotorSpeed = MAX_SPEED;
  }
  if (RMotorSpeed > MAX_SPEED) {
    RMotorSpeed = MAX_SPEED;
  }
}

void read_IR() {
  IR_val[0] = analogRead(IR1) ;
  IR_val[1] = analogRead(IR2) ;
  IR_val[2] = analogRead(IR3) ;
  IR_val[3] = analogRead(IR4) ;
  IR_val[4] = analogRead(IR5) ;
  IR_val[5] = analogRead(IR6) ;
  IR_val[6] = analogRead(IR7) ;
  IR_val[7] = analogRead(IR8) ;
}

void set_speed() {
  analogWrite(ENA, LMotorSpeed);
  analogWrite(ENB, RMotorSpeed);
}

void set_forward() {
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}

void stop() {
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
}
