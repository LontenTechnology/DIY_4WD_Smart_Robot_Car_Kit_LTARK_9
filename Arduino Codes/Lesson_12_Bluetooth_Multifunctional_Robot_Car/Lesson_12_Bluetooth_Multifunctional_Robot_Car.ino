#include <Wire.h>              //including libraries of I2C
#include <IRremote.h>          //including libraries of remote control
#define RECV_PIN  12        //pin 12 of IR remoter control receiver
#include <Servo.h>
IRrecv irrecv(RECV_PIN);      //defining pin 12 of IR remoter control
Servo myservo;
decode_results res;
decode_results results;         //cache of decode of IR remoter control
#define IR_Go       0x00FF18E7 //going forward
#define IR_Back     0x00FF4AB5  //going backward
#define IR_Left     0x00FF10EF//turning left
#define IR_Right    0x00FF5AA5  //turning right
#define IR_Stop     0x00FF38C7  //stop 
//////////////////////////////////////////////////
#define STOP      0
#define FORWARD   1
#define BACKWARD  2
#define TURNLEFT  3
#define TURNRIGHT 4
const int trac1 = 11; //Sort from the leftmost direction of the front of the vehicle (sensors)
const int trac2 = 10; 
const int trac3 = 9; 
const int trac4 = 3; 
int data[4];
int inputPin=A0;  // ultrasonic module   ECHO to A0
int outputPin=A1;  // ultrasonic module  TRIG to A1
volatile int DL;
volatile int DM;
volatile int DR;
unsigned char bluetooth_data;       
unsigned long Key;
unsigned long Key1;
#define Rpwm_pin  5     //pin of controlling speed---- ENA of motor driver board
#define Lpwm_pin  6    //pin of controlling speed---- ENB of motor driver board
int pinLB=2;             //pin of controlling turning---- IN1 of motor driver board
int pinLF=4;             //pin of controlling turning---- IN2 of motor driver board
int pinRB=7;            //pin of controlling turning---- IN3 of motor driver board
int pinRF=8;            //pin of controlling turning---- IN4 of motor driver board
int flag=0;
int Car_state=0;             //the working state of car
int myangle;                //defining variable of angle
int pulsewidth;              //defining variable of pulse width
unsigned char DuoJiao=90;    //initialized angle of motor at 90°
const int intSpeedPWM=180;  //Set the initial speed of the trolley operation
void Sensor_IO_Config()     //IO initialized function of three line tracking , all setting at input
{
  pinMode(trac1, INPUT);
  pinMode(trac2, INPUT);
  pinMode(trac3, INPUT);
  pinMode(trac4, INPUT);
  pinMode(inputPin, INPUT);      //starting receiving IR remote control signal
  pinMode(outputPin, OUTPUT);    //IO of ultrasonic module
}

void Sensor_Scan(void) //function of reading-in signal of line tracking module 
{            
  data[0] = digitalRead(11);//the left
  data[1] = digitalRead(10);
  data[2] = digitalRead(9);
  data[3] = digitalRead(3);
}

void M_Control_IO_config(void)
{
  pinMode(pinLB,OUTPUT); // /pin 2
  pinMode(pinLF,OUTPUT); // pin 4
  pinMode(pinRB,OUTPUT); // pin 7
  pinMode(pinRF,OUTPUT);  // pin 8
  pinMode(Lpwm_pin,OUTPUT);  // pin 5 (PWM) 
  pinMode(Rpwm_pin,OUTPUT);  // pin6(PWM)   
}
void Set_Speed(unsigned char pwm) //function of setting speed
{
  analogWrite(Lpwm_pin,pwm);
  analogWrite(Rpwm_pin,pwm);
}
void advance()    //  going forward
    {
     digitalWrite(pinRB,LOW);  // making motor move towards right rear
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,LOW);  // making motor move towards left rear
     digitalWrite(pinLF,HIGH); 
     Car_state = 1; 
       
    }
void turnR()        //turning right(dual wheel)
    {
     digitalWrite(pinRB,HIGH);
     digitalWrite(pinRF,LOW );   //making motor move towards right front
     digitalWrite(pinLB,LOW);   //making motor move towards left rear
     digitalWrite(pinLF,HIGH);
     Car_state = 4;
    
    }
void turnL()         //turning left(dual wheel)
    {
     digitalWrite(pinRB,LOW);  //making motor move towards right rear
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,HIGH);
     digitalWrite(pinLF,LOW);  //making motor move towards left front
     Car_state = 3;
     
    }    
void stopp()        //stop
    {
     digitalWrite(pinRB,LOW);
     digitalWrite(pinRF,LOW);
     digitalWrite(pinLB,LOW);
     digitalWrite(pinLF,LOW);
     Car_state = 5;
    
    }
void back()         //back up
    {
     digitalWrite(pinRB,HIGH);  //making motor move towards right rear     
     digitalWrite(pinRF,LOW);
     digitalWrite(pinLB,HIGH);  //making motor move towards left rear
     digitalWrite(pinLF,LOW);
     Car_state = 2;
         
    }
         
void Follow()
{
  flag = 0;
  while (flag == 0) 
  {
    DM=checkdistance();  //Get the current distance
    Serial.println(DM);
    analogWrite(Rpwm_pin,intSpeedPWM);
    analogWrite(Lpwm_pin,intSpeedPWM);
    if(DM<=10){
      Serial.println("stop");
      stopp();
      delay(300);
      DM = checkdistance();
      delay(500);
    }
    else { //There are no obstacles ahead, the car is moving forward
      Serial.println("forward");
      advance();
   }
    if (Serial.available())
      {
        bluetooth_data = Serial.read();
        if (bluetooth_data == 'S') {
          flag = 1;
        }
      }
  }
}

void Line_Tracking(void) //function of line tracking 
{
  flag = 0;
  while (flag == 0) 
  {
    Sensor_Scan();
  if(!data[0] && !data[1] && !data[2] && !data[3])     //0000 Lighting
  {
    motorRun(FORWARD, 120,120);
  }
  if(!data[0] && data[1] && data[2] && !data[3])   //0110
  {
    motorRun(FORWARD, 120,120);
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
  if (Serial.available())
   {
      bluetooth_data = Serial.read();
      if (bluetooth_data == 'S') {
        flag = 1;
      }
   }
}
}
void motorRun(int cmd,int left_speed,int right_speed) 
{
  //Speed 
  analogWrite(Rpwm_pin,right_speed);
  analogWrite(Lpwm_pin,left_speed);
  //Direction
  switch(cmd)
  {
     case FORWARD:
     Serial.println("FORWARD"); 
     advance();
     break; 
     case TURNLEFT:
      Serial.println("TURN  LEFT"); 
      turnL();
      break;
     case TURNRIGHT:
      Serial.println("TURN  RIGHT"); 
      turnR();
      break;
     default:
      //If none of the above situations are true, output STOP and all motor outputs are low level
      Serial.println("STOP");
      stopp();
    }
}



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

void Detect_obstacle_distance() {
  myservo.write(160);
  for (int i = 0; i < 3; i = i + 1) {
    DL = checkdistance();
    delay(100);
  }
  myservo.write(20);
  for (int i = 0; i<3; i = i + 1) {
    DR = checkdistance();
    delay(100);
  }
}

void Ultrasonic_Obstacle_Avoidance()
{
  flag = 0;
  while (flag == 0) 
  {
  DM = checkdistance();
  if (DM < 30) {
    stopp();
    Set_Speed(0);
    delay(1000);
    Detect_obstacle_distance();
    if (DL < 50 || DR < 50) {
      if (DL > DR) {
        myservo.write(90);
        turnL();
        Set_Speed(200);
        delay(200);
        advance();
        Set_Speed(200);
      } else {
        myservo.write(90);
        turnR();
        Set_Speed(200);
        delay(200);
        advance();
        Set_Speed(200);
      }
    } else {
      if (random(1, 10) > 5) {
        myservo.write(90);
        turnL();
        Set_Speed(200);
        delay(200);
        advance();
        Set_Speed(200);
      } else {
        myservo.write(90);
        turnR();
        Set_Speed(200);
        delay(200);
        advance();
        Set_Speed(200);
      }
    }
  } else {
    advance();
    Set_Speed(130);
  }
  if (Serial.available())
     {
      bluetooth_data = Serial.read();
      if (bluetooth_data == 'S') {
        flag = 1;
      }
     }
 }
}






void Infrared_Remote_Control(void)   //remote control，when pressing“#”，it quitting from the mode
{
  flag = 0;
  while (flag == 0) 
  {
   if(irrecv.decode(&results))  //to judge whether serial port receive data
    {
     Key = results.value;
     switch(Key)
     {
       case IR_Go:advance();Set_Speed(200);   //UP
       break;
       case IR_Back: back();Set_Speed(200);   //back
       break;
       case IR_Left:turnL();Set_Speed(200);   //Left    
       break;
       case IR_Right:turnR();Set_Speed(200); //Righ
       break;
       case IR_Stop:stopp();Set_Speed(0);   //stop
       break;
       default: 
       break;      
     } 
     irrecv.resume(); // Receive the next value
    }
    if (Serial.available())
     {
      bluetooth_data = Serial.read();
      if (bluetooth_data == 'S') {
        flag = 1;
      }
     }
  }
}


void setup() 
{ 
   myservo.attach(A2);
   M_Control_IO_config();     //motor controlling the initialization of IO
   Set_Speed(200);  //setting initialized speed
  
   Sensor_IO_Config();            //initializing IO of line tracking module 
   irrecv.enableIRIn();           //starting receiving IR remote control signal
   Serial.begin(9600);            //initialized serial port , using Bluetooth as serial port, setting baud 
   myservo.write(DuoJiao);
   stopp();                       //stop
   delay(1000);
   DL = 0;
   DM = 0;
   DR = 0;
} 
void loop() 
{  
  if (Serial.available())
  {
    bluetooth_data = Serial.read();
    Serial.println((char)bluetooth_data);
  }
  switch ((char)bluetooth_data) {
   case 'U':
    advance();
    Set_Speed(200);
    break;
   case 'D':
    back();
    Set_Speed(200);
    break;
   case 'L':
    turnL();
    Set_Speed(200);
    break;
   case 'R':
    turnR();
    Set_Speed(200);
    break;
   case 'S':
    stopp();
    Set_Speed(0);
    break;
   case 'T':
    stopp();
    Line_Tracking();
    break;
   case 'O':
    stopp();
    Ultrasonic_Obstacle_Avoidance();
    break;
   case 'I':
    stopp();
    Infrared_Remote_Control();
    break;
   case 'G':
    stopp();
    Follow();
    break;
   default: 
    break;
  }
}
