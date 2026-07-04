#pragma once

#include "concept_control_mode.h"

class FlightModeAutoLevelLocal
{
public:
    ControlMode_t type() const
    {
        return auto_level;
    }

    PidConstants_t pidConstants() const
    {
        return {
            0.3f, 0.001f, 0.015f,
            0.16f, 0.000f, 0.0400f,
            0.16f, 0.000f, 0.0400f
        };
    }

    void activate(BaseDrone* drone) const
    {
//        gyro->reset();
//
//        processor->sleepMilliseconds(1000);
//
//        static_cast<Bno08xDroneGyro*>(gyro)->setModeEulerAndAcceleration();
//
//        processor->sleepMilliseconds(1000);
//
//        gyro->reload();
//
//        drone->setDesiredYawAngle(gyro->yaw());
    }

    bool yawCompassMode() const
    {
        return true;
    }

    static_assert(ControlModeConcept<FlightModeAutoLevelLocal>, "FlightModeAutoLevelLocal does not meet the ControlModeConcept");

};
