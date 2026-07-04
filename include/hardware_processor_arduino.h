#pragma once

#include "Arduino.h"
#include "concept_hardware_processor.h"

class HardwareProcessorArduino
{
public:
  static void setup()
  {
    Serial.begin(115200);

    if (!Serial)
    {
        Serial.println("FAILED TO START SERIAL...");
    }
    while (!Serial)
    {
    }
  }

  static unsigned long microsecondsTimestamp()
  {
    return micros();
  }

  static unsigned long millisecondsTimestamp()
  {
    return millis();
  }

  static void sleepMilliseconds(const int milliseconds)
  {
    delay(milliseconds);
  }

  static void print(const char *array)
  {
    Serial.println(array);
  }

  static_assert(HardwareProcessorConcept<HardwareProcessorArduino>, "HardwareProcessorArduino does not implement HardwareProcessorConcept");

};
