#pragma once

#include "Arduino.h"
#include <SparkFun_BMP581_Arduino_Library.h>

/*
 * Bosch BMP581 pressure sensor on the default SPI bus, sampling continuously and read at a fixed rate.
 */
class Bmp581Barometer
{
public:
    Bmp581Barometer(uint8_t chip_select_pin, int read_frequency_hz);

    bool setup();

    void run();

    float pressure() const { return _pressure_pa; }

    float temperature() const { return _temperature_celsius; }

private:
    static constexpr uint32_t spi_clock_hz = 1000000;

    BMP581 _sensor;
    const uint8_t _chip_select_pin;
    const unsigned long _read_interval_microseconds;
    unsigned long _last_read_timestamp_microseconds = 0;
    bool _is_ready = false;
    float _pressure_pa = 0.0f;
    float _temperature_celsius = 0.0f;
};
