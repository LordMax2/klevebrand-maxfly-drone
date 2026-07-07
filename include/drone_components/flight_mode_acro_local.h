#pragma once

#include "klevebrand_maxfly_drone.h"

class FLightModeAcroLocal
{
public:
    ControlMode_t type()
    {
        return acro;
    }

    const char* name()
    {
        return "acro";
    }

    bool yawCompassMode()
    {
        return false;
    }

    PidConstants_t pidConstants()
    {
        return {
            0.04f, 0.25f, 0.004f,
            0.04f, 0.25f, 0.004f,
            0.04f, 0.25f, 0.004f
        };
    }

    void activate(MaxFlyDroneBase* drone)
    {
        drone->gyro.reset();

        drone->processor.sleepMilliseconds(1000);

        drone->gyro.setModeAcro();

        drone->processor.sleepMilliseconds(1000);

        drone->gyro.reload();

        drone->setDesiredYawAngle(drone->getYaw());
    }
};

static_assert(ControlModeConcept<FLightModeAcroLocal, MaxFlyPid, MaxFlyPosition, MaxFlyGyro, MaxFlyProcessor>);
