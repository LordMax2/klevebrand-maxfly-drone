#pragma once

#include "klevebrand_maxfly_drone.h"

class FLightModeAcroLocal
{
public:
    static ControlMode_t type()
    {
        return acro;
    }

    static const char* name()
    {
        return "acro";
    }

    static bool yawCompassMode()
    {
        return false;
    }

    static PidConstants_t pidConstants()
    {
        return {
            0.04f, 0.25f, 0.004f,
            0.04f, 0.25f, 0.004f,
            0.04f, 0.25f, 0.004f
        };
    }

    static void activate(MaxFlyDroneBase* drone)
    {
//        gyro.reset();
//
//        MaxFlyProcessor::sleepMilliseconds(1000);
//
//        gyro.setModeAcro();
//
//        MaxFlyProcessor::sleepMilliseconds(1000);
//
//        gyro.reload();
//
//        drone->setDesiredYawAngle(gyro.yaw());
    }
};
