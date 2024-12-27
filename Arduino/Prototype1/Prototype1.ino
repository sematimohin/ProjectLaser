//ОБЪЯВЛЕНИЕ КОНСТАНТ И ПЕРЕМЕННЫХ.
  #include "median3.h"
  #include "kalman.h"

  GKalman testFilter(140, 68, 1);

  
//  GMedian3<int> testFilter;
  const int o_ENA = 11; // Вывод управления скоростью вращения мотора №1
  const int o_IN1 = 0; // Вывод управления направлением вращения мотора №1
  const int o_IN2 = 1; // Вывод управления направлением вращения мотора №1

  const int o_ENB = 10; // Вывод управления скоростью вращения мотора №2
  const int o_IN3 = 2; // Вывод управления направлением вращения мотора №2
  const int o_IN4 = 3; // Вывод управления направлением вращения мотора №2


  const int i_SENSOR1_DIGITAL = 8; // Цифровой вывод датчика (подключен к 8 пину Arduino)

  const int o_LASER = 4; // Вывод с которого подается сигнал на лазер.

  // Переменная, в которой будет храниться текущее полученное значение с датчика.
  int sensor1_value = 0;
  int start_count = 0;
  int period = 0;
  int period1 = 0;
  int current_count = 0;
  uint8_t Motor1_power = 0; // Значение ШИМ (или скорости вращения) 8-битный целочисленный тип без знака
  uint8_t Motor2_power = 0;

  int calculate_period(){ 
    current_count = millis();
    period = current_count - start_count;
    start_count = current_count;
    return period;
  }

void setup()
{
// НАСТРОЙКА ПОРТОВ ВВОДА-ВЫВОДА

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
    digitalWrite(o_LASER, LOW);

    Motor1_power = 10 ; // Устанавливаем значение ШИМ 50 для 1 мотора 30 на 65(70) пропеллер адоб
    delay(200);
    Motor2_power = 0 ; // ШИМ для второго мотора

    
    analogWrite(o_ENA, Motor1_power); // Устанавливаем скорость 1-го мотора
    analogWrite(o_ENB, Motor2_power); // Устанавливаем скорость 1-го мотора

    digitalWrite(o_IN1, HIGH);
    digitalWrite(o_IN2, LOW);

    digitalWrite(o_IN3, HIGH);
    digitalWrite(o_IN4, LOW);

}

void loop()
{

    sensor1_value = digitalRead(i_SENSOR1_DIGITAL); // Считываем текущее значение с датчика
//    Serial.print("\t Digital Reading=");
//    Serial.println(sensor1_value);
    delay(2);
    
    if(sensor1_value ==0) // Если пришёл 0 с датчика, то вызываем функцию вычисления периода вращения мотора.
        period1 = calculate_period(); // Запись в переменную period1 вычисленного времени вращения

//Serial.println(period1);

  Serial.println(analogRead(Motor1_power));
//   //Медиана
//      period1 = testFilter.filtered(period1); 
//  Serial.println(period1);

//  Калман
//  period1 = testFilter.filtered((int)period1);
//  Serial.println(period1);
//  Serial.println(";");

// Serial.print("\t Period1=");
// Serial.println(period1);

}
