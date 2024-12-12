void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT);

}

void loop() {

  if (digitalRead(2))
  {
    Serial.println("White color");
    delay(500);
  }
  else
  {
    Serial.println("Black color");
    delay(500);
  }
}
