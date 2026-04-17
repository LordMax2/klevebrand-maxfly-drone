#ifndef CONTROL_ARBITER_H
#define CONTROL_ARBITER_H

#include "pwm_receiver_controller.h"
#include "skywire_drone_controller.h"

enum class ActiveControlSource_t
{
    none,
    pwm,
    skywire
};

class ControlManager
{
public:
    ControlManager(PwmReceiverController *pwm_receiver, SkywireDroneController *skywire_controller)
        : _pwm_receiver(pwm_receiver), _skywire_controller(skywire_controller) {}

    void setup();
    void run(KlevebrandMaxFlyDrone *drone);
    ActiveControlSource_t activeControlSource() const;

private:
    PwmReceiverController *_pwm_receiver;
    SkywireDroneController *_skywire_controller;
    ActiveControlSource_t _active_control_source = ActiveControlSource_t::none;
};

#endif // CONTROL_ARBITER_H
