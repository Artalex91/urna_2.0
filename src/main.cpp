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
#define releUpPin   4
#define releDownPin 5
#define concDownPin 6
#define concUpPin   7
#define ledPin      13

bool concDown           = LOW;
bool concUp             = LOW;
int concState           = 0;
int motorState          = 0;
int motor               = 0;
uint32_t releMill       = 0;
bool releFlag           = false;
#define constTimeMotorOff 1000
bool protect            = false;

int range               = 0;             //храним расстояние
#define constRange        100     //мм  расстояние сработки
bool open               = false;         // состояние крышки
uint32_t openMill       = 0;     //счетчик открытой крышки
#define constOpenMill     4000 //мс задержка в открытом состоянии

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
  pinMode(ledPin,      OUTPUT);

  pinMode(concDownPin, INPUT);
  pinMode(concUpPin,   INPUT);
}

void loop()
{
  // дальномер open/close
  Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred())
  {
    Serial.print(" TIMEOUT");
  }
  Serial.println();

  range = sensor.readRangeSingleMillimeters();

  if (range < constRange)
    openMill = millis();
  if (range < constRange && open != true)
  {
    open = true;
  }
  else if (range >= constRange && open != false)
  {
    if (millis() - openMill > constOpenMill)
    {
      open = false;
    }
  }
  // концевики (состояния крышки)
  concDown = digitalRead(concDownPin);
  concUp   = digitalRead(concUpPin);

       if (concDown == HIGH && concUp == LOW  && concState != 1) concState = 1; // открыта
  else if (concDown == LOW  && concUp == HIGH && concState != 2) concState = 2; // промежуточное положение крышки
  else if (concDown == HIGH && concUp == HIGH && concState != 3) concState = 3; // закрыта
  else if (concDown == LOW  && concUp == LOW  && concState != 4) concState = 4; // неисправность (оба концевика видят свое активное положение)

  // состояние мотора
       if (releUpPin == HIGH && releDownPin == LOW  && motorState != 1) motorState = 1; //движение ВВЕРХ
  else if (releUpPin == LOW  && releDownPin == HIGH && motorState != 2) motorState = 2; //ОСТАНОВЛЕН
  else if (releUpPin == LOW  && releDownPin == LOW  && motorState != 3) motorState = 3; //движение ВНИЗ
  else if (releUpPin == HIGH && releDownPin == HIGH && motorState != 4) motorState = 4; //ошибка

  //конечный автомат
  if (open == true)
  {
    switch (concState)
    {
    case 1: // концевик - открыта
      switch (motorState)
      {
      case 1: // мотор - движится вверх
        motor = 2;
        break;
      case 2: // мотор - остановлен
        // ждем
        break;
      case 3: // мотор - движится вних
        motor = 2;
        break;
      case 4: // мотор - ошибка
        Serial.println("motor error o14");
        motor = 2;
        break;
      }
      break;
    case 2: // концевик - Промежуточное положение
      switch (motorState)
      {
      case 1: // мотор - движится вверх
        // ждем
        break;
      case 2: // мотор - остановлен
        motor = 1;
        break;
      case 3: // мотор - движится вних
        motor = 2;
        break;
      case 4: // мотор - ошибка
        Serial.println("motor error o24");
        motor = 2;
        break;
      }
      break;
    case 3: // концевик - закрыта
      switch (motorState)
      {
      case 1: // мотор - движится вверх
        //ждем
        break;
      case 2: // мотор - остановлен
        motor = 1;
        break;
      case 3: // мотор - движится вних
        motor = 2;
        break;
      case 4: // мотор - ошибка
        Serial.println("motor error o34");
        motor = 2;
        break;
      }
      break;
    case 4: //концевик - неисправность
      Serial.println("conc error o4");
      motor = 2;
      break;
    }
  }

  else
  { //open == false
    switch (concState)
    {
    case 1: // концевик - открыта
      switch (motorState)
      {
      case 1: // мотор - движится вверх
        motor = 2;
        break;
      case 2: // мотор - остановлен
        motor = 3;
        break;
      case 3: // мотор - движится вних
        // ждем
        break;
      case 4: // мотор - ошибка
        Serial.println("motor error c14");
        motor = 2;
        break;
      }
      break;
    case 2: // концевик - Промежуточное положение
      switch (motorState)
      {
      case 1: // мотор - движится вверх
        motor = 2;
        break;
      case 2: // мотор - остановлен
        motor = 2;
        break;
      case 3: // мотор - движится вних
        // ждем
        break;
      case 4: // мотор - ошибка
        Serial.println("motor error c24");
        motor = 2;
        break;
      }
      break;
    case 3: // концевик - закрыта
      switch (motorState)
      {
      case 1: // мотор - движится вверх
        motor = 2;
        break;
      case 2: // мотор - остановлен
        // ждем
        break;
      case 3: // мотор - движится вних
        motor = 2;
        break;
      case 4: // мотор - ошибка
        Serial.println("motor error c34");
        motor = 2;
        break;
      }
      break;
    case 4: //концевик - неисправность
      Serial.println("conc error c4");
      motor = 2;
      break;
    }
  }

  // вкл/выкл привод
  switch (motor)
  {
  case 0:
    // ждем изменений
    break;
  case 1: // включить мотор вверх
    if(protect==false) digitalWrite(releUpPin, HIGH);
    else digitalWrite(releUpPin, LOW);
    digitalWrite(releDownPin, LOW);
    motor = 0;
    break;
  case 2: // ОСТАНОВИТЬ мотор
    digitalWrite(releUpPin, LOW);
    digitalWrite(releDownPin, LOW);
    motor = 0;
    break;
  case 3: // включить мотор вниз
    digitalWrite(releUpPin, LOW);
    if(protect==false) digitalWrite(releDownPin, HIGH);
    else digitalWrite(releDownPin, LOW);
    motor = 0;
    break;
  }

  // отсечка привода по времени (если концевик не сработал)
  if (releUpPin == HIGH || releDownPin == HIGH){
    if(releFlag != true){
      releFlag = true;
      releMill = millis();
      }
  }
  if (protect==false && releFlag==true && millis() - releMill > constTimeMotorOff){
    digitalWrite(releUpPin, LOW);
    digitalWrite(releDownPin, LOW);
    protect=true;
    Serial.print("PROTECT");
    digitalWrite(ledPin, HIGH);
  }
}