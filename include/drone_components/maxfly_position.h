#pragma once

#include <HardwareSerial.h>
#include "drone_bmp581_neom9n_position.h"

/*
 * The position as the board wires it: the NEO-M9N on Serial1 and the BMP581 on the default SPI bus, chip select 32.
 */
template <DroneGyroConcept SomeDroneGyroType>
class MaxFlyPositionType : public DroneBmp581Neom9nPosition<SomeDroneGyroType>
{
    using LibraryPosition = DroneBmp581Neom9nPosition<SomeDroneGyroType>;

    static constexpr uint8_t barometer_chip_select_pin = 32;
    static constexpr uint8_t imu_chip_select_pin = 10;

public:
    explicit MaxFlyPositionType(SomeDroneGyroType *gyro)
        : LibraryPosition(gyro, Serial1, barometer_chip_select_pin)
    {
    }

    void setup()
    {
        // The ICM-45686 shares the BMP581's SPI bus and has no driver yet, so it must stay deselected.
        pinMode(imu_chip_select_pin, OUTPUT);
        digitalWrite(imu_chip_select_pin, HIGH);

        LibraryPosition::setup();
    }
};
