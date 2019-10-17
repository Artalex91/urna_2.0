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
  #define concPin 6

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

  pinMode(concPin,      INPUT);

}

void loop()
{
  Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();

  range = sensor.readRangeSingleMillimeters();

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
    
  }
}