#pragma once

#include "pwm_receiver_controller.h"
#include "skywire_drone_controller.h"

enum class ActiveController_t
{
    none,
    pwm,
    skywire
};

class ControlManager
{
public:
    ControlManager(PwmReceiverController *pwm_receiver, SkywireDroneController *skywire_controller)
        : _pwm_receiver_controller(pwm_receiver), _skywire_controller(skywire_controller) {}

    void setup() const;
    void run(KlevebrandMaxFlyDrone *drone);
    ActiveController_t activeControlSource() const;

private:
    PwmReceiverController *_pwm_receiver_controller;
    SkywireDroneController *_skywire_controller;
    ActiveController_t _active_controller = ActiveController_t::none;
};
