#ifndef KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_ACRO_LOCAL_H
#define KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_ACRO_LOCAL_H

#include "base_control_mode.h"

class FLightModeAcroLocal : public BaseControlMode
{
public:
    int pidConstantsStorageKey() const override
    {
        return 1152;
    }

    ControlMode_t type() const override
    {
        return acro;
    }

    PidConstants_t pidConstants() const override
    {
        return {
            0.04f, 0.25f, 0.004f,
            0.04f, 0.25f, 0.004f,
            0.04f, 0.25f, 0.004f
        };
    }

    void activate(BaseDrone* drone, BaseDroneGyro* gyro, BaseHardwareProcessor* processor) const override
    {
        gyro->reset();

        processor->sleepMilliseconds(1000);

        static_cast<Bno08xDroneGyro*>(gyro)->setModeAcro();

        processor->sleepMilliseconds(1000);

        gyro->reload();

        drone->setDesiredYawAngle(gyro->yaw());
    }
};

#endif //KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_ACRO_LOCAL_H
