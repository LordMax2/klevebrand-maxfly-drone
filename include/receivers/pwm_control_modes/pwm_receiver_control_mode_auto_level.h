#ifndef KLEVEBRAND_MAXFLY_DRONE_PWM_RECEIVER_CONTROL_MODE_AUTO_LEVEL_H
#define KLEVEBRAND_MAXFLY_DRONE_PWM_RECEIVER_CONTROL_MODE_AUTO_LEVEL_H

#include "receivers/pwm_control_modes/base_pwm_receiver_control_mode.h"

class PwmReceiverControlModeAutoLevel : public BasePwmReceiverControlMode
{
public:
    ControlMode_t controlModeType() const override;

    void applyThrottleYawPitchRoll(KlevebrandMaxFlyDrone* drone, int throttle_pwm, int yaw_pwm, int pitch_pwm,
                                   int roll_pwm) const override;

private:
    mutable unsigned long _last_yaw_set_timestamp_milliseconds = 0;
    mutable unsigned long _last_slew_timestamp_microseconds = 0;

    float getSlewTimeDelta() const;

    static float normalizePwm(int pwm_microseconds);

    static float applyExpo(float input, float expo);

    static float applySlew(float current, float target, float dt);
};

#endif // KLEVEBRAND_MAXFLY_DRONE_PWM_RECEIVER_CONTROL_MODE_AUTO_LEVEL_H
