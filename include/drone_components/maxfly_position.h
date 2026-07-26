#pragma once

#include <HardwareSerial.h>
#include "drone_bmp280_neom9n_position.h"

template <DroneGyroConcept SomeDroneGyroType>
class MaxFlyPositionType : public DroneBmp280Neom9nPosition<SomeDroneGyroType>
{
public:
    explicit MaxFlyPositionType(SomeDroneGyroType *gyro, const int run_interval_hz = 25)
        : DroneBmp280Neom9nPosition<SomeDroneGyroType>(gyro, Serial2, run_interval_hz)
    {
    }
};
