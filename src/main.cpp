// Urna 2.0 
#include <Arduino.h> 
/*The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>
VL53L0X sensor;

#define releUpPin 4
#define releDownPin 5
#define 

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();
}

void loop()
{
  Serial.print(sensor.readRangeContinuousMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
}