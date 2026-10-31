#pragma once

#include "Arduino.h"

class HardwareProcessorArduino
{
public:
  static constexpr unsigned long serial_wait_timeout_milliseconds = 2000;

  void setup()
  {
    Serial.begin(115200);

    // The USB serial port only becomes ready when a terminal opens it, so wait for one only briefly: the drone
    // has to start without a computer attached.
    const unsigned long wait_start_milliseconds = millis();

    while (!Serial && millis() - wait_start_milliseconds < serial_wait_timeout_milliseconds)
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
