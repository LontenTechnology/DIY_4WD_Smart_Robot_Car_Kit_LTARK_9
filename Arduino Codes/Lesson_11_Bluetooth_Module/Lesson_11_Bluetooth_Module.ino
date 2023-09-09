char app_key_value; 
void setup() 
{ 
Serial.begin(9600);
}
 void loop()
{ if(Serial.available()) //to judge whether the serial port receives the data.
    {
     app_key_value=Serial.read();
     Serial.println(app_key_value);
    }
}
