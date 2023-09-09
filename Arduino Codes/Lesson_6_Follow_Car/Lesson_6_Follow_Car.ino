#include<Servo.h>
Servo myServo;
#define ENA  5             //pin of controlling speed---- ENA of motor driver board
#define ENB  6            //pin of controlling speed---- ENB of motor driver board
int pinLB=2;             //pin of controlling turning---- IN1 of motor driver board
int pinLF=4;             //pin of controlling turning---- IN2 of motor driver board
int pinRB=7;            //pin of controlling turning---- IN3 of motor driver board
int pinRF=8;            //pin of controlling turning---- IN4 of motor driver board
const int intSpeedPWM=180;  //Set the initial speed of the trolley operation
volatile int DL;
volatile int DM;
volatile int DR;
float checkdistance() {
  digitalWrite(A1, LOW);
  delayMicroseconds(2);
  digitalWrite(A1, HIGH);
  delayMicroseconds(10);
  digitalWrite(A1, LOW);
  float distance = pulseIn(A0, HIGH) / 58.00;
  delay(10);
  return distance;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A1, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(pinLB,OUTPUT);  // pin 2
  pinMode(pinLF,OUTPUT);  // pin 4
  pinMode(pinRB,OUTPUT);  // pin 7
  pinMode(pinRF,OUTPUT);  // pin 8
  pinMode(ENA,OUTPUT);    // pin 5(PWM) 
  pinMode(ENB,OUTPUT);    // pin 6(PWM) 
  myServo.attach(A2);
  myServo.write(90);
}

void loop() {
   DM=checkdistance();  //Get the current distance
   analogWrite(ENA,intSpeedPWM);
   analogWrite(ENB,intSpeedPWM);
   if(DM<=10){
    Serial.println("stop");
    pause();
    delay(300);
    DM = checkdistance();
    delay(500);
  }
  else { //There are no obstacles ahead, the car is moving forward
    Serial.println("forward");
    forward();
  }
 }
//===============
// forward
//===============
void forward(){
    digitalWrite(pinRB,LOW);  // making motor move towards right rear
    digitalWrite(pinRF,HIGH);
    digitalWrite(pinLB,LOW);  // making motor move towards left rear
    digitalWrite(pinLF,HIGH); 
}
//===============
// back
//===============
void backward(){
    digitalWrite(pinRB,HIGH);  //making motor move towards right rear     
    digitalWrite(pinRF,LOW);
    digitalWrite(pinLB,HIGH);  //making motor move towards left rear
    digitalWrite(pinLF,LOW);
}
//===============
// stop
//===============
void pause(){
     digitalWrite(pinRB,LOW);
     digitalWrite(pinRF,LOW);
     digitalWrite(pinLB,LOW);
     digitalWrite(pinLF,LOW);
}

