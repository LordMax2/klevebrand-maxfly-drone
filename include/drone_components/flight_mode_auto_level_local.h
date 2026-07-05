#pragma once

#include "klevebrand_maxfly_drone.h"

class FlightModeAutoLevelLocal
{
public:
    ControlMode_t type() const
    {
        return auto_level;
    }

    const char* name() const
    {
        return "auto_level";
    }

    PidConstants_t pidConstants() const
    {
        return {
            0.3f, 0.001f, 0.015f,
            0.16f, 0.000f, 0.0400f,
            0.16f, 0.000f, 0.0400f
        };
    }

    bool yawCompassMode() const
    {
        return true;
    }

    void activate(MaxFlyDroneBase* drone) const
    {
//        gyro.reset();
//
//        processor.sleepMilliseconds(1000);
//
//        gyro.setModeEulerAndAcceleration();
//
//        processor.sleepMilliseconds(1000);
//
//        gyro.reload();
//
//        drone->setDesiredYawAngle(gyro.yaw());
    }
};
