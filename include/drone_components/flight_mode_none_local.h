#pragma once

#include "klevebrand_maxfly_drone.h"

class FlightModeNoneLocal
{
public:
    static ControlMode_t type()
    {
        return none;
    }

    static const char* name()
    {
        return "none";
    }

    static bool yawCompassMode()
    {
        return false;
    }

    static PidConstants_t pidConstants()
    {
        return PidConstants_t{};
    }

    static void activate(MaxFlyDroneBase* drone)
    {
//        (void)drone;
//        (void)gyro;
//        (void)processor;
    }
};
