#pragma once

#include "concept_pwm_receiver_control_mode.h"

class PwmReceiverControlModeAutoLevel
{
public:
    ControlMode_t controlModeType();

    void applyThrottleYawPitchRoll(KlevebrandMaxFlyDrone* drone, int throttle_pwm, int yaw_pwm, int pitch_pwm,
                                   int roll_pwm);

private:
    mutable unsigned long _last_yaw_set_timestamp_milliseconds = 0;
    mutable unsigned long _last_slew_timestamp_microseconds = 0;

    float getSlewTimeDelta() const;

    static float normalizePwm(int pwm_microseconds);

    static float applyExpo(float input, float expo);

    static float applySlew(float current, float target, float dt);
};

static_assert(ConceptPwmReceiverControlMode<PwmReceiverControlModeAutoLevel>, "Control mode doesnt implement the concept");
