#pragma once

#include "concept_pwm_receiver_control_mode.h"

class PwmReceiverControlModeAcro
{
public:
    ControlMode_t controlModeType();

    void applyThrottleYawPitchRoll(KlevebrandMaxFlyDrone* drone, int throttle_pwm, int yaw_pwm, int pitch_pwm, int roll_pwm);

private:
    static float normalizePwm(int pwm_microseconds);

    static float applyExpo(float input, float expo);
};

static_assert(ConceptPwmReceiverControlMode<PwmReceiverControlModeAcro>, "Control mode doesnt implement the concept");
