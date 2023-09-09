long f;

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
  pinMode(A1, OUTPUT);
  pinMode(A0, INPUT);
  Serial.begin(9600);
  delay(1000);
}

void loop() {
   f=checkdistance();
   Serial.print(f);
   Serial.println("cm");
   delay(1000);
}
