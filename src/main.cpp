// Urna 2.0 
#include <Arduino.h> 

#include <Wire.h>
#include <VL53L0X.h>

//дальномер
  VL53L0X sensor;
  // Раскомментируйте эту строку, чтобы использовать дальний режим. Этот
  // увеличивает чувствительность датчика и расширяет его
  // потенциальный диапазон, но увеличивает вероятность получения
  // неточное чтение из-за отражений от объектов
  // кроме целевой цели. Лучше всего работает в темноте условия.
  //#define LONG_RANGE


  // Раскомментируйте одну из этих двух строк, чтобы получить
  // - более высокая скорость за счет меньшей точности ИЛИ
  // - более высокая точность за счет меньшей скорости

  //#define HIGH_SPEED
  //#define HIGH_ACCURACY

//
  #define releUpPin 4
  #define releDownPin 5
  #define concDownPin 6
  #define concUpPin   7

  bool concDown = LOW;
  bool concUp   = LOW;
  int concState = 0;
  int motor = 0;

  int range = 0; //храним расстояние
  #define constRange 100 //мм  расстояние сработки
  bool open = false;  // состояние крышки
  uint32_t openMill= 0; //счетчик открытой крышки
  #define constOpenMill 4000 //мс задержка в открытом состоянии


void setup()
{
  Serial.begin(9600);
  Wire.begin();
//дальномер
    sensor.init();
    sensor.setTimeout(500);

  #if defined LONG_RANGE
    // lower the return signal rate limit (default is 0.25 MCPS)
    sensor.setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  #endif

  #if defined HIGH_SPEED
    // reduce timing budget to 20 ms (default is about 33 ms)
    sensor.setMeasurementTimingBudget(20000);
  #elif defined HIGH_ACCURACY
    // increase timing budget to 200 ms
    sensor.setMeasurementTimingBudget(200000);
  #endif

//output-input
  pinMode(releUpPin,   OUTPUT);
  pinMode(releDownPin, OUTPUT);

  pinMode(concDownPin,  INPUT);
  pinMode(concUpPin,    INPUT);

}

void loop()
{
  Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();

  range = sensor.readRangeSingleMillimeters();

  concDown = digitalRead(concDownPin);
  concUp   = digitalRead(concUpPin);
  if     (concDown==LOW  && concUp==LOW ) concState=0; // неисправность (оба концевика видят свое активное положение)
  else if(concDown==HIGH && concUp==LOW ) concState=1; // открыта
  else if(concDown==LOW  && concUp==HIGH) concState=2; // закрыта
  else if(concDown==HIGH && concUp==HIGH) concState=3; // промежуточное положение крышки
  else                                    concState=4; // неисправность

  if(range < constRange){
    open = true;
    openMill = millis();
  }
  else{
    if(millis() - openMill > constOpenMill){
      open = false;
    }
  }

  if(open == true){
    switch (concState){
      case 0:

        break;
      case 1: // открыта
        // ничего не делаем
        break;
      case 2: // закрыта
        motor = 1; // включить мотор ВВЕРХ
        break;
      case 3: // промежуточное положение

        break;
      case 4:

        break;
    }
  }
  else { //open == false
    switch (concState){
      case 0:

        break;
      case 1: // открыта
        motor = 2; // включить мотор ВНИЗ
        break;
      case 2: // закрыта
        // ничео не делаем
        break;
      case 3: // промежуточное положение

        break;
      case 4:

        break;
    }
  }

  switch (motor){
    case 0: // выключить мотор
      digitalWrite(releUpPin,   LOW);
      digitalWrite(releDownPin, LOW);
      break;
    case 1: // включить мотор вверх
      digitalWrite(releUpPin,  HIGH);
      digitalWrite(releDownPin, LOW);
      break;
    case 2: // включить мотор вниз
      digitalWrite(releUpPin,   LOW);
      digitalWrite(releDownPin,HIGH);
      break;
  }
}