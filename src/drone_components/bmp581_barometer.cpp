#include "drone_components/bmp581_barometer.h"

#include <SPI.h>

Bmp581Barometer::Bmp581Barometer(const uint8_t chip_select_pin, const int read_frequency_hz)
    : _chip_select_pin(chip_select_pin), _read_interval_microseconds(1000000UL / read_frequency_hz)
{
}

bool Bmp581Barometer::setup()
{
    // The SparkFun driver uses the default SPI bus but never starts it.
    SPI.begin();

    if (_sensor.beginSPI(_chip_select_pin, spi_clock_hz) != BMP5_OK)
    {
        Serial.println(F("FAILED TO SETUP BMP581."));

        return false;
    }

    // 8x pressure oversampling at 50 Hz, well inside what the BMP581 can convert in a 20 ms period.
    bmp5_osr_odr_press_config oversampling = {};
    oversampling.osr_t = BMP5_OVERSAMPLING_1X;
    oversampling.osr_p = BMP5_OVERSAMPLING_8X;

    if (_sensor.setOSRMultipliers(&oversampling) != BMP5_OK || _sensor.setODRFrequency(BMP5_ODR_50_HZ) != BMP5_OK)
    {
        Serial.println(F("FAILED TO CONFIGURE BMP581 SAMPLING."));

        return false;
    }

    _is_ready = true;

    Serial.println(F("SUCCESSFULLY SETUP BMP581"));

    return true;
}

void Bmp581Barometer::run()
{
    const unsigned long now = micros();

    if (!_is_ready || now - _last_read_timestamp_microseconds < _read_interval_microseconds)
    {
        return;
    }

    _last_read_timestamp_microseconds = now;

    bmp5_sensor_data data = {};

    if (_sensor.getSensorData(&data) != BMP5_OK || !(data.pressure > 0.0f))
    {
        return;
    }

    _pressure_pa = data.pressure;
    _temperature_celsius = data.temperature;
}
