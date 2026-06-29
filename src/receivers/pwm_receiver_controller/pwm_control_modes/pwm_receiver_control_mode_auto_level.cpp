#include "receivers/pwm_control_modes/pwm_receiver_control_mode_auto_level.h"

constexpr int PWM_DEADZONE = 20;

constexpr float YAW_STEP_EXPONENTIAL_INCREASE_RATE = 0.5f;
constexpr float MAX_YAW_STEP_DEGREES = 5.0f;
constexpr float MAX_RATE_DEG_PER_SEC = 135.0f;

ControlMode_t PwmReceiverControlModeAutoLevel::controlModeType() const
{
    return auto_level;
}

void PwmReceiverControlModeAutoLevel::applyThrottleYawPitchRoll(KlevebrandMaxFlyDrone* drone, const int throttle_pwm,
                                                                const int yaw_pwm, const int pitch_pwm,
                                                                const int roll_pwm) const
{
    const float throttle_value = map(throttle_pwm, 1000, 2000, THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    drone->setThrottle(throttle_value);

    const float yaw_stick = normalizePwm(yaw_pwm);
    const float yaw_increment = -applyExpo(yaw_stick, YAW_STEP_EXPONENTIAL_INCREASE_RATE) * MAX_YAW_STEP_DEGREES;

    const unsigned long timestamp_milliseconds = millis();

    if (timestamp_milliseconds - _last_yaw_set_timestamp_milliseconds > 50)
    {
        const float new_desired_yaw_angle = constrain(drone->getDesiredYawAngle() + yaw_increment, -180.0f, 180.0f);
        drone->setDesiredYawAngle(new_desired_yaw_angle);

        _last_yaw_set_timestamp_milliseconds = timestamp_milliseconds;
    }

    const float dt = getSlewTimeDelta();

    const float desired_pitch_angle = map(pitch_pwm, 1000, 2000, 60, -60);
    const float current_pitch_angle = drone->getDesiredPitchAngle();
    const float slew_limited_desired_pitch = applySlew(current_pitch_angle, desired_pitch_angle, dt);
    drone->setDesiredPitchAngle(slew_limited_desired_pitch);

    const float desired_roll_angle = map(roll_pwm, 1000, 2000, 60, -60);
    const float current_roll_angle = drone->getDesiredRollAngle();
    const float slew_limited_desired_roll = applySlew(current_roll_angle, desired_roll_angle, dt);
    drone->setDesiredRollAngle(slew_limited_desired_roll);
}

float PwmReceiverControlModeAutoLevel::getSlewTimeDelta() const
{
    const unsigned long timestamp_microseconds = micros();

    if (_last_slew_timestamp_microseconds == 0)
    {
        _last_slew_timestamp_microseconds = timestamp_microseconds;
    }

    float dt = (timestamp_microseconds - _last_slew_timestamp_microseconds) / 1000000.0f;
    _last_slew_timestamp_microseconds = timestamp_microseconds;

    if (dt > 0.1f)
    {
        dt = 0.1f;
    }

    return dt;
}

float PwmReceiverControlModeAutoLevel::normalizePwm(const int pwm_microseconds)
{
    const int centered = pwm_microseconds - 1500;

    if (abs(centered) < PWM_DEADZONE)
    {
        return 0.0f;
    }

    return constrain(centered / 500.0f, -1.0f, 1.0f);
}

float PwmReceiverControlModeAutoLevel::applyExpo(const float input, const float expo)
{
    const float sign = (input >= 0.0f) ? 1.0f : -1.0f;
    const float abs_input = abs(input);

    return sign * (abs_input * abs_input * abs_input * expo + abs_input * (1.0f - expo));
}

float PwmReceiverControlModeAutoLevel::applySlew(const float current, const float target, const float dt)
{
    const float error = target - current;
    const float max_step = MAX_RATE_DEG_PER_SEC * dt;
    const float limited_change = constrain(error, -max_step, max_step);

    return current + limited_change;
}
