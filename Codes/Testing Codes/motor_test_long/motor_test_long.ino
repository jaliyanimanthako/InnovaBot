int motor1pin1 =47;
int motor1pin2 =49;

int motor2pin1 =51 ;
int motor2pin2 =53 ;

int ena = 3;
int enb = 4;

void setup() {
  // put your setup code here, to run once:
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:   
  digitalWrite(ena ,HIGH);
  digitalWrite(enb ,LOW);
  
  //digitalWrite(motor1pin1, HIGH);
  //digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
  
  delay(1000);

  //digitalWrite(motor1pin1, LOW);
  //digitalWrite(motor1pin2, HIGH);

  
  //digitalWrite(motor2pin1, LOW);
  //digitalWrite(motor2pin2, HIGH);

  //delay(1000);
  
}
  
