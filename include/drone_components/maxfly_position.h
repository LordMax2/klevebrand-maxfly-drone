#pragma once

#include <HardwareSerial.h>
#include "drone_bmp280_neom9n_position.h"
#include "drone_components/bmp581_barometer.h"

/*
 * The library position (NEO-M9N on Serial1, altitude from the GPS) plus the BMP581 for pressure and temperature. The
 * library's own barometer is a BMP280 on I2C, which the board doesn't have: it reports that and carries on.
 */
template <DroneGyroConcept SomeDroneGyroType>
class MaxFlyPositionType : public DroneBmp280Neom9nPosition<SomeDroneGyroType>
{
    using LibraryPosition = DroneBmp280Neom9nPosition<SomeDroneGyroType>;

    static constexpr uint8_t barometer_chip_select_pin = 32;
    static constexpr uint8_t imu_chip_select_pin = 10;

    Bmp581Barometer _barometer;

public:
    explicit MaxFlyPositionType(SomeDroneGyroType *gyro, const int run_interval_hz = 25)
        : LibraryPosition(gyro, Serial1, run_interval_hz),
          _barometer(barometer_chip_select_pin, run_interval_hz)
    {
    }

    void setup()
    {
        // The ICM-45686 shares the BMP581's SPI bus and has no driver yet, so it must stay deselected.
        pinMode(imu_chip_select_pin, OUTPUT);
        digitalWrite(imu_chip_select_pin, HIGH);

        if (!_barometer.setup())
        {
            Serial.println(F("BMP581 UNAVAILABLE. CONTINUING WITHOUT PRESSURE."));
        }

        LibraryPosition::setup();
    }

    void run(const bool has_gyro_update)
    {
        LibraryPosition::run(has_gyro_update);

        _barometer.run();
    }

    float getTemperature() const
    {
        return _barometer.temperature();
    }

    float getPressure() const
    {
        return _barometer.pressure();
    }
};
