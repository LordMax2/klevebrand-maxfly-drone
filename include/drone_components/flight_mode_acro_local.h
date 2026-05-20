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

    PidConstants_t pidConstants() const override
    {
        return {
            0.04f, 0.4f, 0.003f,
            0.04f, 0.4f, 0.003f,
            0.04f, 0.4f, 0.003f
        };
    }
};

#endif //KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_ACRO_LOCAL_H
