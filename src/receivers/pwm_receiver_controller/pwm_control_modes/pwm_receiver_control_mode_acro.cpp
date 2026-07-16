#include "receivers/pwm_control_modes/pwm_receiver_control_mode_acro.h"

#include <Arduino.h>

#include "drone_components/quadcopter_pid.h"

constexpr int PWM_DEADZONE = 20;

constexpr float ACRO_EXPONENTIAL_INCREASE_RATE = 0.5f;
constexpr float MAX_ACRO_RATE_PITCH_ROLL = 300.0f;
constexpr float MAX_ACRO_RATE_YAW = 300.0f;

ControlMode_t PwmReceiverControlModeAcro::controlModeType()
{
    return acro;
}

void PwmReceiverControlModeAcro::applyThrottleYawPitchRoll(KlevebrandMaxFlyDrone* drone, const int throttle_pwm,
                                                           const int yaw_pwm,
                                                           const int pitch_pwm, const int roll_pwm)
{
    const float throttle_value = map(throttle_pwm, 1000, 2000, THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    drone->setThrottle(throttle_value);

    const float roll = applyExpo(normalizePwm(roll_pwm), ACRO_EXPONENTIAL_INCREASE_RATE);
    const float pitch = applyExpo(normalizePwm(pitch_pwm), ACRO_EXPONENTIAL_INCREASE_RATE);
    const float yaw = applyExpo(normalizePwm(yaw_pwm), ACRO_EXPONENTIAL_INCREASE_RATE);

    drone->setDesiredRollAngle(-(roll * MAX_ACRO_RATE_PITCH_ROLL));
    drone->setDesiredPitchAngle(-(pitch * MAX_ACRO_RATE_PITCH_ROLL));
    drone->setDesiredYawAngle(-(yaw * MAX_ACRO_RATE_YAW));
}

float PwmReceiverControlModeAcro::normalizePwm(const int pwm_microseconds)
{
    const int centered = pwm_microseconds - 1500;

    if (abs(centered) < PWM_DEADZONE)
    {
        return 0.0f;
    }

    return constrain(centered / 500.0f, -1.0f, 1.0f);
}

float PwmReceiverControlModeAcro::applyExpo(const float input, const float expo)
{
    const float sign = (input >= 0.0f) ? 1.0f : -1.0f;
    const float abs_input = abs(input);

    return sign * (abs_input * abs_input * abs_input * expo + abs_input * (1.0f - expo));
}
