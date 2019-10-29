// Urna 3.0
#include <Arduino.h>
#include <avr/wdt.h>

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
#define releUpPin   4
#define releDownPin 5
#define concDownPin 6
#define concUpPin   7
#define ledPin      13

uint32_t printMill = 0;
int range = 0;
const int constRange = 550;//mm
uint32_t openMill = 0;
bool open1 = false;
const int constOpenMill = 5000;// millisecond open state
bool releUp = false;
bool releDown = false;
bool concDown=false;
bool concUp=false;

uint32_t releMill = 0;
bool protect = false;



void setup()
{
   Serial.begin(9600);
   Serial.println(" setup");
  wdt_disable(); 
    delay(3000); // Задержка, чтобы было время перепрошить устройство в случае bootloop
    wdt_enable (WDTO_500MS); // Для тестов не рекомендуется устанавливать значение менее 8 сек.
  
 
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
  pinMode(ledPin,      OUTPUT);


  pinMode(concDownPin, INPUT_PULLUP);
  pinMode(concUpPin,   INPUT_PULLUP);

  if(digitalRead(concUpPin)==HIGH && digitalRead(concDownPin)==HIGH){
    digitalWrite(releUpPin, HIGH);
    delay(100);
    }

    releMill=millis();
    
  
}

void loop()
{
  // дальномер open/close
  if(millis()-printMill>1000){
    printMill=millis();
    Serial.print(sensor.readRangeSingleMillimeters());
    if (sensor.timeoutOccurred())
    {
      Serial.print(" TIMEOUT");
    }
    Serial.println();
  }

  range = sensor.readRangeSingleMillimeters();

  if (range < constRange)
    openMill = millis();
  if (range < constRange && open1 != true)
  {
    open1 = true;
  }
  else if (range >= constRange && open1 != false)
  {
    if (millis() - openMill > constOpenMill)
    {
      open1 = false;
    }
  }
  // концевики (состояния крышки)
  concDown = digitalRead(concDownPin);
  concUp   = digitalRead(concUpPin);

  if(open1==true && protect==false){
         if(                 concUp==HIGH) {releDown=LOW; releUp=HIGH;}
    else if(                 concUp==LOW ) {releDown=LOW; releUp=LOW;}
    }
  else if (open1==false  && protect==false){
         if(concDown==HIGH               ) {releDown=HIGH; releUp=LOW;}
    else if(concDown==LOW                ) {releDown=LOW;  releUp=LOW;}
    }

if(releDown==HIGH && digitalRead(concDownPin)==LOW){
  releDown=LOW;
  digitalWrite(releDownPin, LOW);
  }

if(releUp==HIGH && digitalRead(concUpPin)==LOW){
  releUp=LOW;
  digitalWrite(releUpPin, LOW);
  }

    
  if(releUp==LOW && releDown==LOW){
    releMill=millis();
    
    }
  if(millis()-releMill>1900){
    protect=true;
    releUp=LOW;
    releDown=LOW;
    
    }
  digitalWrite(ledPin, protect);  
  digitalWrite(releUpPin, releUp);
  digitalWrite(releDownPin, releDown);
  wdt_reset();
  }