#pragma once

#include "klevebrand_maxfly_drone.h"

class FlightModeAutoLevelLocal
{
public:
    ControlMode_t type()
    {
        return auto_level;
    }

    const char* name()
    {
        return "auto_level";
    }

    PidConstants_t pidConstants()
    {
        return {
            0.3f, 0.001f, 0.015f,
            0.16f, 0.000f, 0.0400f,
            0.16f, 0.000f, 0.0400f
        };
    }

    bool yawCompassMode()
    {
        return true;
    }

    void activate(MaxFlyDroneBase* drone)
    {
        drone->gyro.reset();

        drone->processor.sleepMilliseconds(1000);

        drone->gyro.setModeEulerAndAcceleration();

        drone->processor.sleepMilliseconds(1000);

        drone->gyro.reload();

        drone->setDesiredYawAngle(drone->gyro.yaw());
    }
};

static_assert(ControlModeConcept<FlightModeAutoLevelLocal, MaxFlyPid, MaxFlyPosition, MaxFlyGyro, MaxFlyProcessor>);
