#define IR1 A0
#define IR2 A1
#define IR3 A2
#define IR4 A3
#define IR5 A4
#define IR6 A5
#define RMotorA 2
#define RMotorB 3
#define RMotorPWM 9
#define LMotorA 4
#define LMotorB 5
#define LMotorPWM 10
#define MAX_SPEED 150

int motorbasespeed = MAX_SPEED; // Set the initial motor speed
int IR_val[6] = {0, 0, 0, 0, 0, 0};
int IR_weights[6] = {-20, -10, -5, 5, 10, 20};
int LMotorspeed = 0;
int RMotorspeed = 0;
float P, I, D;
float error = 0;
float previous_error = 0;
float KP = 5.5;
float KD = 10;
float KI = 0;

void PID_control();
void read_IR();
void set_speed(int leftSpeed, int rightSpeed);
void set_forward();
void stop();

void setup() {
  Serial.begin(9600);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);

  pinMode(LMotorA, OUTPUT);
  pinMode(LMotorB, OUTPUT);
  pinMode(LMotorPWM, OUTPUT);
  pinMode(RMotorA, OUTPUT);
  pinMode(RMotorB, OUTPUT);
  pinMode(RMotorPWM, OUTPUT);

  set_forward();
  delay(2000);
}

void loop() {
  read_IR();
  if (IR_val[0] == 0 && IR_val[1] == 0 && IR_val[2] == 0 && IR_val[3] == 0 && IR_val[4] == 0 && IR_val[5] == 0) {
    stop();
  } else {
    PID_control();
    set_speed(LMotorspeed, RMotorspeed);
  }
}

void PID_control() {
  error = 0;

  for (int i = 0; i < 6; i++) {
    error += IR_weights[i] * IR_val[i];
  }

  P = error;
  I += error;
  D = error - previous_error;
  previous_error = error;
  LMotorspeed = motorbasespeed - (KP * P + KI * I + KD * D);
  RMotorspeed = motorbasespeed + (KP * P + KI * I + KD * D);

  if (LMotorspeed < 0) {
    LMotorspeed = 0;
  }
  if (RMotorspeed < 0) {
    RMotorspeed = 0;
  }
  if (LMotorspeed > MAX_SPEED) {
    LMotorspeed = MAX_SPEED;
  }
  if (RMotorspeed > MAX_SPEED) {
    RMotorspeed = MAX_SPEED;
  }
}

void read_IR() {
  IR_val[0] = digitalRead(IR1);
  IR_val[1] = digitalRead(IR2);
  IR_val[2] = digitalRead(IR3);
  IR_val[3] = digitalRead(IR4);
  IR_val[4] = digitalRead(IR5);
  IR_val[5] = digitalRead(IR6);
}

void set_speed(int leftSpeed, int rightSpeed) {
  analogWrite(LMotorPWM, leftSpeed);
  analogWrite(RMotorPWM, rightSpeed);
}

void set_forward() {
  digitalWrite(LMotorA, HIGH);
  digitalWrite(LMotorB, LOW);
  digitalWrite(RMotorA, HIGH);
  digitalWrite(RMotorB, LOW);
}

void stop() {
  analogWrite(LMotorPWM, 0);
  analogWrite(RMotorPWM, 0);
}
