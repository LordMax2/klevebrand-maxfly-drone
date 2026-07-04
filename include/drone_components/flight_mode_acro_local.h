#pragma once

#include "concept_control_mode.h"

class FLightModeAcroLocal
{
public:
    ControlMode_t type() const
    {
        return acro;
    }

    PidConstants_t pidConstants() const
    {
        return {
            0.04f, 0.25f, 0.004f,
            0.04f, 0.25f, 0.004f,
            0.04f, 0.25f, 0.004f
        };
    }

    void activate(BaseDrone* drone) const
    {
//        gyro->reset();
//
//        processor->sleepMilliseconds(1000);
//
//        static_cast<Bno08xDroneGyro*>(gyro)->setModeAcro();
//
//        processor->sleepMilliseconds(1000);
//
//        gyro->reload();
//
//        drone->setDesiredYawAngle(gyro->yaw());
    }

    static_assert(ControlModeConcept<FLightModeAcroLocal>, "FLightModeAcroLocal does not meet the ControlModeConcept");
};
