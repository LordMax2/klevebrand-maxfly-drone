#pragma once

#include "Arduino.h"

class HardwareProcessorArduino
{
public:
  void setup()
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

  unsigned long microsecondsTimestamp()
  {
    return micros();
  }

  unsigned long millisecondsTimestamp()
  {
    return millis();
  }

  void sleepMilliseconds(const int milliseconds)
  {
    delay(milliseconds);
  }

  void print(const char *array)
  {
    Serial.println(array);
  }

};
