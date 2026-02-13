#ifndef HARDWARE_PROCESSOR_ARDUINO_H
#define HARDWARE_PROCESSOR_ARDUINO_H

#include "Arduino.h"
#include "base_hardware_processor.h"

class HardwareProcessorArduino : public BaseHardwareProcessor
{
public:
  HardwareProcessorArduino() : BaseHardwareProcessor() {};

  void setup() override
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

  unsigned long microsecondsTimestamp() override
  {
    return micros();
  }

  unsigned long millisecondsTimestamp() override
  {
    return millis();
  }

  void sleepMilliseconds(int milliseconds) override
  {
    delay(milliseconds);
  }

  void print(char *array) override
  {
    Serial.println(array);
  }
};

#endif // HARDWARE_PROCESSOR_ARDUINO_H