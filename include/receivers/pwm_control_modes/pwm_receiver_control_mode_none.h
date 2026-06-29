#ifndef KLEVEBRAND_MAXFLY_DRONE_PWM_RECEIVER_CONTROL_MODE_NONE_H
#define KLEVEBRAND_MAXFLY_DRONE_PWM_RECEIVER_CONTROL_MODE_NONE_H

#include "receivers/pwm_control_modes/base_pwm_receiver_control_mode.h"

class PwmReceiverControlModeNone : public BasePwmReceiverControlMode
{
public:
    ControlMode_t controlModeType() const override
    {
        return none;
    }

    void applyThrottleYawPitchRoll(
        KlevebrandMaxFlyDrone* drone,
        int throttle_pwm,
        int yaw_pwm,
        int pitch_pwm,
        int roll_pwm
    ) const override
    {
        (void)drone;
        (void)throttle_pwm;
        (void)yaw_pwm;
        (void)pitch_pwm;
        (void)roll_pwm;
    }
};

#endif // KLEVEBRAND_MAXFLY_DRONE_PWM_RECEIVER_CONTROL_MODE_NONE_H
