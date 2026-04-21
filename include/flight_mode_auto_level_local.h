//
// Created by Max Klevebrand on 2026-04-13.
//

#ifndef KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_AUTO_LEVEL_LOCAL_H
#define KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_AUTO_LEVEL_LOCAL_H

#include "flight_mode_auto_level.h"

class FlightModeAutoLevelLocal : public FlightModeAutoLevel {
public:
    PidConstants_t pidConstants() const override {
        return {0.125f, 0.2f, 0.0125f, 0.125f, 0.2f, 0.00125f, 0.05f, 0.1f, 0.001f};
    }

    bool yawCompassMode() const override {
        return true;
    }
};

#endif //KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_AUTO_LEVEL_LOCAL_H
