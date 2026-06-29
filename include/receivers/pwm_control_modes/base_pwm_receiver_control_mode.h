#ifndef KLEVEBRAND_MAXFLY_DRONE_BASE_PWM_RECEIVER_CONTROL_MODE_H
#define KLEVEBRAND_MAXFLY_DRONE_BASE_PWM_RECEIVER_CONTROL_MODE_H

#include "klevebrand_maxfly_drone.h"

class BasePwmReceiverControlMode
{
public:
    virtual ~BasePwmReceiverControlMode() = default;

    virtual ControlMode_t controlModeType() const = 0;

    virtual void applyThrottleYawPitchRoll(KlevebrandMaxFlyDrone* drone, int throttle_pwm, int yaw_pwm, int pitch_pwm,
                                           int roll_pwm) const = 0;
};

#endif // KLEVEBRAND_MAXFLY_DRONE_BASE_PWM_RECEIVER_CONTROL_MODE_H
