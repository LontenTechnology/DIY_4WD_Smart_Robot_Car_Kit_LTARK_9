const int trac1 = 11; //Sort from the leftmost direction of the front of the vehicle (sensors)
const int trac2 = 10; 
const int trac3 = 9; 
const int trac4 = 3; 

void setup() {
  Serial.begin(9600);  //initialization Serialport
  pinMode(trac1, INPUT);
  pinMode(trac2, INPUT);
  pinMode(trac3, INPUT);
  pinMode(trac4, INPUT);
}

void loop()
{
   int data[4];             
   data[0] = digitalRead(11);//the left
   data[1] = digitalRead(10);
   data[2] = digitalRead(9);
   data[3] = digitalRead(3);
   Serial.print(data[0]);
   Serial.print("---");
   Serial.print(data[1]);
   Serial.print("---");
   Serial.print(data[2]);
   Serial.print("---");
   Serial.println(data[3]);
   delay(1000);
}
