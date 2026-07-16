#pragma once

class PwmReceiverControlModeNone
{
public:
    ControlMode_t controlModeType()
    {
        return none;
    }

    void applyThrottleYawPitchRoll(KlevebrandMaxFlyDrone* drone, int throttle_pwm, int yaw_pwm, int pitch_pwm,
                                   int roll_pwm)
    {
        (void)drone;
        (void)throttle_pwm;
        (void)yaw_pwm;
        (void)pitch_pwm;
        (void)roll_pwm;
    }
};

static_assert(ConceptPwmReceiverControlMode<PwmReceiverControlModeNone>, "Control mode doesnt implement the concept");
