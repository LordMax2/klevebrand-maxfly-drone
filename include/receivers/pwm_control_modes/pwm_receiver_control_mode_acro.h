#ifndef KLEVEBRAND_MAXFLY_DRONE_PWM_RECEIVER_CONTROL_MODE_ACRO_H
#define KLEVEBRAND_MAXFLY_DRONE_PWM_RECEIVER_CONTROL_MODE_ACRO_H

#include "receivers/pwm_control_modes/base_pwm_receiver_control_mode.h"

class PwmReceiverControlModeAcro : public BasePwmReceiverControlMode
{
public:
    ControlMode_t controlModeType() const override;

    void applyThrottleYawPitchRoll(KlevebrandMaxFlyDrone* drone, int throttle_pwm, int yaw_pwm, int pitch_pwm,
                                   int roll_pwm) const override;

private:
    static float normalizePwm(int pwm_microseconds);

    static float applyExpo(float input, float expo);
};

#endif // KLEVEBRAND_MAXFLY_DRONE_PWM_RECEIVER_CONTROL_MODE_ACRO_H
