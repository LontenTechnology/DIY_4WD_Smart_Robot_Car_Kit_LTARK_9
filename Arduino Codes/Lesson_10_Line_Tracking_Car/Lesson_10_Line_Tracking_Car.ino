#include <Servo.h>  
#define STOP      0
#define FORWARD   1
#define BACKWARD  2
#define TURNLEFT  3
#define TURNRIGHT 4

#define ENA  5             //pin of controlling speed---- ENA of motor driver board
#define ENB  6            //pin of controlling speed---- ENB of motor driver board
int pinLB=2;             //pin of controlling turning---- IN1 of motor driver board
int pinLF=4;             //pin of controlling turning---- IN2 of motor driver board
int pinRB=7;            //pin of controlling turning---- IN3 of motor driver board
int pinRF=8;            //pin of controlling turning---- IN4 of motor driver board
Servo myservo;

const int trac1 = 11; //Sort from the leftmost direction of the front of the vehicle (sensors)
const int trac2 = 10; 
const int trac3 = 9; 
const int trac4 = 3; 

void setup() {

  Serial.begin(9600);  //initialization Serialport
  myservo.attach(A2);// attaches the servo on A2 to the servo object
  pinMode(pinLB,OUTPUT);  // pin 2
  pinMode(pinLF,OUTPUT);  // pin 4
  pinMode(pinRB,OUTPUT);  // pin 7
  pinMode(pinRF,OUTPUT);  // pin 8
  pinMode(ENA,OUTPUT);    // pin 5(PWM) 
  pinMode(ENB,OUTPUT);    // pin 6(PWM) 

  myservo.write(90); //Initialize the servo angle to 90 degrees
 
  pinMode(trac1, INPUT);
  pinMode(trac2, INPUT);
  pinMode(trac3, INPUT);
  pinMode(trac4, INPUT);
}

void loop()
{
  tracing(); 
}

void tracing()          
{
  int data[4];             
  data[0] = digitalRead(11);//the left
  data[1] = digitalRead(10);
  data[2] = digitalRead(9);
  data[3] = digitalRead(3);
  
  //High level - off state (without reflection) - black line - light not on; Low level - reception (reflection) - no black line - light on
  
  //No black lines detected on either side, straight ahead
  if(!data[0] && !data[1] && !data[2] && !data[3])     //0000 Lighting
  {
    motorRun(FORWARD, 180,180);
  }
  if(!data[0] && data[1] && data[2] && !data[3])   //0110
  {
    motorRun(FORWARD, 180,180);
  }

  //Black line detected on the right, turn right
  if(!data[0] && !data[1] && data[2] && !data[3])    //0010
  {
    motorRun(TURNRIGHT, 250,220);
  }
  if(!data[0] && !data[1] && data[2] && data[3])   //0011
  {
    motorRun(TURNRIGHT, 250,250);
  }
  if(!data[0] && !data[1] && !data[2] && data[3])    //0001
  {
    motorRun(TURNRIGHT, 255,230);
  }
  
  //Black line detected on the left, turn left
  if(!data[0] && data[1] && !data[2] && !data[3])    //0100
  {
    motorRun(TURNLEFT, 220,250);
  }
  if(data[0] && data[1] && !data[2] && !data[3])   //1100
  {
    motorRun(TURNLEFT, 250,250);
  }
  if(data[0] && !data[1] && !data[2] && !data[3])    //1000
  {
    motorRun(TURNLEFT, 230,255);
  }
  
  //Black lines detected on both sides indicate a stop
  if(data[0] && data[1] && data[2] && data[3])     //1111
  {
    motorRun(STOP, 0,0);
    while(1);
  }

  Serial.print(data[0]);
  Serial.print("---");
  Serial.print(data[1]);
  Serial.print("---");
  Serial.print(data[2]);
  Serial.print("---");
  Serial.println(data[3]);
}

void motorRun(int cmd,int left_speed,int right_speed) 
{
  //Speed
  analogWrite(ENA,right_speed);
  analogWrite(ENB,left_speed);
  //Direction
  switch(cmd)
  {
     case FORWARD:
     Serial.println("FORWARD"); 
     digitalWrite(pinRB,LOW);  // making motor move towards right rear
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,LOW);  // making motor move towards left rear
     digitalWrite(pinLF,HIGH); 
     break; 
     case TURNLEFT:
      Serial.println("TURN  LEFT"); 
      digitalWrite(pinRB,LOW);  //making motor move towards right rear
      digitalWrite(pinRF,HIGH);
      digitalWrite(pinLB,HIGH);
      digitalWrite(pinLF,LOW);  //making motor move towards left front
      break;
     case TURNRIGHT:
      Serial.println("TURN  RIGHT"); 
      digitalWrite(pinRB,HIGH);
      digitalWrite(pinRF,LOW );   //making motor move towards right front
      digitalWrite(pinLB,LOW);   //making motor move towards left rear
      digitalWrite(pinLF,HIGH);
      break;
     default:
      //If none of the above situations are true, output STOP and all motor outputs are low level
      Serial.println("STOP");
      digitalWrite(pinRB,LOW);
      digitalWrite(pinRF,LOW);
      digitalWrite(pinLB,LOW);
      digitalWrite(pinLF,LOW);
    }
}
