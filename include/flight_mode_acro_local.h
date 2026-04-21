//
// Created by Max Klevebrand on 2026-04-13.
//

#ifndef KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_ACRO_LOCAL_H
#define KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_ACRO_LOCAL_H

#include "flight_mode_acro.h"

class FLightModeAcroLocal : public FlightModeAcro {
public:
    PidConstants_t pidConstants() const override {
        return {0.04, 0.4, 0.003, 0.04, 0.4, 0.003, 0.04, 0.4, 0.003};
    }
};

#endif //KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_ACRO_LOCAL_H
