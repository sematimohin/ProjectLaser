//ОБЪЯВЛЕНИЕ КОНСТАНТ И ПЕРЕМЕННЫХ.
  #include "median3.h"
  #include "kalman.h"
  #include "GyverPID.h"
  //GKalman testFilter(140, 68, 1);

  
//  GMedian3<int> testFilter;
  const int o_ENA = 11; // Вывод управления скоростью вращения мотора №1
  const int o_IN1 = 0; // Вывод управления направлением вращения мотора №1
  const int o_IN2 = 1; // Вывод управления направлением вращения мотора №1

  const int o_ENB = 10; // Вывод управления скоростью вращения мотора №2
  const int o_IN3 = 2; // Вывод управления направлением вращения мотора №2
  const int o_IN4 = 3; // Вывод управления направлением вращения мотора №2


  const int i_SENSOR1_DIGITAL = 8; // Цифровой вывод датчика (подключен к 8 пину Arduino)
  const int i_SENSOR2_DIGITAL = 12;

  const int o_LASER = 4; // Вывод с которого подается сигнал на лазер.

  // Переменная, в которой будет храниться текущее полученное значение с датчика.
//  int sensor1_value = 0;
//  int start_count = 0;
//  int rotation_period_1 = 0;
//  int period1 = 0;
//  
//  int current_count = 0;
//  int sensor2_value = 0;
//  
//  int rotation_period_2 = 0;
//  int period2 = 0;
//  int current_count1 = 0;
//  
//  uint8_t Motor1_power = 0; // Значение ШИМ (или скорости вращения) 8-битный целочисленный тип без знака
//  uint8_t Motor2_power = 0;

//int calculate_period_1()
//{
//  current_count = millis();
//while(sensor1_value == 1)
//{
//  sensor1_value = digitalRead(i_SENSOR1_DIGITAL);
//}
//  int rotation_count = millis();
//  int rotation_period_1 = rotation_count - current_count; 
//  return rotation_period_1; 
//} 
//
//int calculate_period_2()
//{
//  current_count = millis();
//while(sensor2_value == 1)
//{
//  sensor2_value = digitalRead(i_SENSOR2_DIGITAL);
//}
//  int rotation_count = millis();
//  int rotation_period_2 = rotation_count - current_count; 
//  return rotation_period_2;
//} 
  bool t = HIGH;
  int a = 0;
  int sensor1_value = 0;
  int start_count = 0;
  int period = 0;
  int period1 = 0;
  int current_count = 0;
  int sensor2_value = 0;
  int start_count1 = 0;
  int period21 = 0;
  int period2 = 0;
  int current_count1 = 0;
  uint8_t Motor1_power = 0; // Значение ШИМ (или скорости вращения) 8-битный целочисленный тип без знака
  uint8_t Motor2_power = 0;


  int calculate_period(){ 
    current_count = millis();
    period = current_count - start_count;
    start_count = current_count;
    int filtered_period_1 = findMedianN_optim1(period);
    return filtered_period_1;
  }
    int calculate_period1(){ 
    current_count1 = millis();
    period21 = current_count1 - start_count1;
    start_count1 = current_count1;
    int filtered_period_2 = findMedianN_optim2(period21);
    return filtered_period_2;
    
  }
  

GyverPID pid(0.92, 1.07, 0.20);

#define NUM_READ 3  // порядок медианы

// медиана на N значений со своим буфером, ускоренный вариант
float findMedianN_optim1(float newVal) {
  static float buffer[NUM_READ];  // статический буфер
  static byte count = 0;
  buffer[count] = newVal;
  if ((count < NUM_READ - 1) and (buffer[count] > buffer[count + 1])) {
    for (int i = count; i < NUM_READ - 1; i++) {
      if (buffer[i] > buffer[i + 1]) {
        float buff = buffer[i];
        buffer[i] = buffer[i + 1];
        buffer[i + 1] = buff;
      }
    }
  } else {
    if ((count > 0) and (buffer[count - 1] > buffer[count])) {
      for (int i = count; i > 0; i--) {
        if (buffer[i] < buffer[i - 1]) {
          float buff = buffer[i];
          buffer[i] = buffer[i - 1];
          buffer[i - 1] = buff;
        }
      }
    }
  }
  if (++count >= NUM_READ) count = 0;
  return buffer[(int)NUM_READ / 2];
}
float findMedianN_optim2(float newVal) {
  static float buffer[NUM_READ];  // статический буфер
  static byte count = 0;
  buffer[count] = newVal;
  if ((count < NUM_READ - 1) and (buffer[count] > buffer[count + 1])) {
    for (int i = count; i < NUM_READ - 1; i++) {
      if (buffer[i] > buffer[i + 1]) {
        float buff = buffer[i];
        buffer[i] = buffer[i + 1];
        buffer[i + 1] = buff;
      }
    }
  } else {
    if ((count > 0) and (buffer[count - 1] > buffer[count])) {
      for (int i = count; i > 0; i--) {
        if (buffer[i] < buffer[i - 1]) {
          float buff = buffer[i];
          buffer[i] = buffer[i - 1];
          buffer[i - 1] = buff;
        }
      }
    }
  }
  if (++count >= NUM_READ) count = 0;
  return buffer[(int)NUM_READ / 2];
}

void setup()
{
// НАСТРОЙКА ПОРТОВ ВВОДА-ВЫВОДА


    pid.setLimits(0, 255);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
    pid.setDt(15);


    // Включение передачи данных по serial со скоростью 9600 бод
    Serial.begin(9600);

    // ARDUINO --> Driver (OUTPUT)
    pinMode(o_ENA, OUTPUT);
    pinMode(o_IN1, OUTPUT);
    pinMode(o_IN2, OUTPUT);
    pinMode(o_LASER, OUTPUT);

    // Sensor --> Arduino (INPUT)
    pinMode(i_SENSOR1_DIGITAL,INPUT);


    // Команда остановки мотору
    digitalWrite(o_IN1, LOW);
    digitalWrite(o_IN2, LOW);
    digitalWrite(o_IN3, LOW);
    digitalWrite(o_IN4, LOW);

    // Команда включить лазер
    
Motor1_power = 35 ;
//Motor2_power = 35 ; // ШИМ для второго мотора

    
    analogWrite(o_ENA, Motor1_power); // Устанавливаем скорость 1-го мотора
    analogWrite(o_ENB, pid.output); // Устанавливаем скорость 1-го мотора

    digitalWrite(o_IN1, HIGH);
    digitalWrite(o_IN2, LOW);

    digitalWrite(o_IN3, HIGH);
    digitalWrite(o_IN4, LOW);

}

void loop()
{
//  if (Serial.available()) {
//    String command = Serial.readStringUntil('\n');
//    if (command.startsWith("M1:")) {
//      Motor1_power = command.substring(3).toInt();
//      analogWrite(o_ENA, Motor1_power);
//    } else if (command.startsWith("M2:")) {
//      Motor2_power = command.substring(3).toInt();
//      analogWrite(o_ENB, Motor2_power);
//    }
//  }

    sensor1_value = digitalRead(i_SENSOR1_DIGITAL); // Считываем текущее значение с датчика
//    Serial.print("\t Digital Reading=");
//    Serial.println(sensor1_value);
    delay(2);
    if(sensor1_value ==0) // Если пришёл 0 с датчика, то вызываем функцию вычисления периода вращения мотора.
    period1 = calculate_period(); // Запись в переменную period1 вычисленного времени вращения

  
    sensor2_value = digitalRead(i_SENSOR2_DIGITAL);
    delay(2);
    if(sensor2_value ==0) // Если пришёл 0 с датчика, то вызываем функцию вычисления периода вращения мотора.
    period2 = calculate_period1();


    // int input = period2;
    // int setpoint = period1;

    // int getResult();
    
//  Serial.println(period1);
  Serial.println(pid.output + 1000);

    a = millis();
    digitalWrite(o_LASER, HIGH);
    if(a+70 == millis())
    digitalWrite(o_LASER, LOW);
    if(a == millis());
  
//Serial.println(period1);
//Serial.println("\t");
//Serial.println(period2);

//  period1 = testFilter.filtered(period1); 
//  Serial.println(period1);

//  Калман
//  period1 = testFilter.filtered((int)period1);
//  Serial.println(period1);
//  Serial.println(";");

// Serial.print("\t Period1=");
// Serial.println(period1);

}
