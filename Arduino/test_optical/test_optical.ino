  #define LASER 4 // задаем имя для Pin10

void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT);
  // pinMode(A0, INPUT);
  pinMode(LASER, OUTPUT); // инициализируем Pin10 как выход

}

void loop() {

  digitalWrite(LASER, HIGH);
  if (digitalRead(2))
  {
    Serial.println("Black color");
    // delay(500);
  }
  else
  {
    Serial.println("White color");
    // delay(500);
  }

  Serial.println(digitalRead(2));
  delay(5);


}
