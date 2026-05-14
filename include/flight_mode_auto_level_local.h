//
// Created by Max Klevebrand on 2026-04-13.
//

#ifndef KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_AUTO_LEVEL_LOCAL_H
#define KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_AUTO_LEVEL_LOCAL_H

#include "flight_mode_auto_level.h"

class FlightModeAutoLevelLocal : public FlightModeAutoLevel
{
public:
    int pidConstantsStorageKey() const override
    {
        return 1280;
    }

    PidConstants_t pidConstants() const override
    {
        return {
            0.05f, 0.00f, 0.003f,
            0.4f, 0.04f, 0.03f,
            0.4f, 0.04f, 0.03f
        };
    }

    void activate(BaseDrone* drone, BaseDroneGyro* gyro, BaseHardwareProcessor* processor) const override
    {
        gyro->reset();

        processor->sleepMilliseconds(1000);

        gyro->setModeEuler();

        processor->sleepMilliseconds(1000);

        gyro->reload();

        drone->setDesiredYawAngle(gyro->yaw());
    }

    bool yawCompassMode() const override
    {
        return true;
    }
};

#endif //KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_AUTO_LEVEL_LOCAL_H
