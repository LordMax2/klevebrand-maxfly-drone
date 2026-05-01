//
// Created by Max Klevebrand on 2026-04-13.
//

#ifndef KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_ACRO_LOCAL_H
#define KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_ACRO_LOCAL_H

#include "flight_mode_acro.h"

class FLightModeAcroLocal : public FlightModeAcro
{
public:
    int pidConstantsStorageKey() const override
    {
        return 1152;
    }

    PidConstants_t pidConstants() const override
    {
        return {
            0.04f, 0.002f / 0.005f, 0.6f * 0.005f,
            0.04f, 0.002f / 0.005f, 0.6f * 0.005f,
            0.04f, 0.002f / 0.005f, 0.6f * 0.005f
        };
    }
};

#endif //KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_ACRO_LOCAL_H
