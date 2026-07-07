#pragma once

#include "klevebrand_maxfly_drone.h"

class FlightModeNoneLocal
{
public:
    ControlMode_t type()
    {
        return none;
    }

    const char* name()
    {
        return "none";
    }

    bool yawCompassMode()
    {
        return false;
    }

    PidConstants_t pidConstants()
    {
        return PidConstants_t{};
    }

    void activate(MaxFlyDroneBase* drone)
    {
        (void)drone;
    }
};

static_assert(ControlModeConcept<FlightModeNoneLocal, MaxFlyPid, MaxFlyPosition, MaxFlyGyro, MaxFlyProcessor>);
