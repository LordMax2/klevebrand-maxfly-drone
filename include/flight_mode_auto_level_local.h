//
// Created by Max Klevebrand on 2026-04-13.
//

#ifndef KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_AUTO_LEVEL_LOCAL_H
#define KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_AUTO_LEVEL_LOCAL_H

#include "flight_mode_auto_level.h"

class FlightModeAutoLevelLocal : public FlightModeAutoLevel {
public:
    PidConstants_t pidConstants() const override {
        return {0.125, 0.001, 2.5, 0.125, 0.001, 0.25, 0.05, 0.0005, 0.2};
    }

    bool yawCompassMode() const override {
        return true;
    }
};

#endif //KLEVEBRAND_MAXFLY_DRONE_FLIGHT_MODE_AUTO_LEVEL_LOCAL_H
