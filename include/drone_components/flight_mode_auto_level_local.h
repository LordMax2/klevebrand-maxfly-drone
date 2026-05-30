#ifndef KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_AUTO_LEVEL_LOCAL_H
#define KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_AUTO_LEVEL_LOCAL_H

#include "base_control_mode.h"

class FlightModeAutoLevelLocal : public BaseControlMode
{
public:
    int pidConstantsStorageKey() const override
    {
        return 1280;
    }

    ControlMode_t type() const override
    {
        return auto_level;
    }

    PidConstants_t pidConstants() const override
    {
        return {
            0.00f, 0.00f, 0.000f,
            0.20f, 0.001f, 0.025f,
            0.20f, 0.001f, 0.025f
        };
    }

    void activate(BaseDrone* drone, BaseDroneGyro* gyro, BaseHardwareProcessor* processor) const override
    {
        gyro->reset();

        processor->sleepMilliseconds(1000);

        static_cast<Bno08xDroneGyro*>(gyro)->setModeEulerAndAcceleration();

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
