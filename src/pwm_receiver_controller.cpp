#include "pwm_receiver_controller.h"

#include "flight_mode_acro_local.h"
#include "flight_mode_auto_level_local.h"

#define PWM_SIGNAL_MINIMUM 1000
#define PWM_SIGNAL_MID_LOW 1250
#define PWM_SIGNAL_MID_HIGH 1750
#define PWM_SIGNAL_ENABLE_THRESHOLD 1050

bool PwmReceiverController::hasValidSignal()
{
    return _receiver.getChannelValue(_throttle_receiver_channel_number) >= 1000 &&
           _receiver.getChannelValue(_yaw_receiver_channel_number) >= 1000 &&
           _receiver.getChannelValue(_pitch_receiver_channel_number) >= 1000 &&
           _receiver.getChannelValue(_roll_receiver_channel_number) >= 1000 &&
           _receiver.getChannelValue(_flight_mode_receiver_channel_number) >= 1000;
}

bool PwmReceiverController::wantsControl()
{
    return hasValidSignal();
}

void PwmReceiverController::apply(KlevebrandMaxFlyDrone *drone)
{
    setThrottleYawPitchRoll(drone);
    setFlightMode(drone);
}

void PwmReceiverController::setThrottleYawPitchRoll(KlevebrandMaxFlyDrone *drone)
{
    float throttle_value = map(_receiver.getChannelValue(_throttle_receiver_channel_number), 1000, 2000, THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    drone->setThrottle(throttle_value);

    if (drone->getFlightMode().type() == acro)
    {
        float desired_yaw_angle = map(_receiver.getChannelValue(_yaw_receiver_channel_number), 1000, 2000, 180, -180);
        if (desired_yaw_angle < 5 && desired_yaw_angle > -5)
        {
            desired_yaw_angle = 0;
        }
        drone->setDesiredYawAngle(desired_yaw_angle);

        float desired_pitch_angle = map(_receiver.getChannelValue(_pitch_receiver_channel_number), 1000, 2000, 120, -120);
        if (desired_pitch_angle < 5 && desired_pitch_angle > -5)
        {
            desired_pitch_angle = 0;
        }
        drone->setDesiredPitchAngle(desired_pitch_angle);

        float desired_roll_angle = map(_receiver.getChannelValue(_roll_receiver_channel_number), 1000, 2000, 120, -120);
        if (desired_roll_angle < 5 && desired_roll_angle > -5)
        {
            desired_roll_angle = 0;
        }
        drone->setDesiredRollAngle(desired_roll_angle);

        return;
    }

    if (drone->getFlightMode().type() == auto_level)
    {
        float desired_yaw_angle = map(_receiver.getChannelValue(_yaw_receiver_channel_number), 1000, 2000, 120, -120);
        if (desired_yaw_angle < 5 && desired_yaw_angle > -5)
        {
            desired_yaw_angle = 0;
        }
        drone->setDesiredYawAngle(desired_yaw_angle);

        float desired_pitch_angle = map(_receiver.getChannelValue(_pitch_receiver_channel_number), 1000, 2000, -60, 60);
        drone->setDesiredPitchAngle(desired_pitch_angle);

        float desired_roll_angle = map(_receiver.getChannelValue(_roll_receiver_channel_number), 1000, 2000, -60, 60);
        drone->setDesiredRollAngle(desired_roll_angle);

        return;
    }
}

void PwmReceiverController::setFlightMode(KlevebrandMaxFlyDrone *drone)
{
    int flight_mode_pwm_signal = _receiver.getChannelValue(_flight_mode_receiver_channel_number);
    int throttle_pwm_signal = _receiver.getChannelValue(_throttle_receiver_channel_number);
    bool throttle_is_zero = throttle_pwm_signal >= PWM_SIGNAL_MINIMUM &&
                            throttle_pwm_signal <= PWM_SIGNAL_ENABLE_THRESHOLD;

    if (flight_mode_pwm_signal >= PWM_SIGNAL_MINIMUM && flight_mode_pwm_signal < PWM_SIGNAL_MID_LOW)
    {
        drone->disableMotors();

        const auto none_flight_mode = FlightMode();
        drone->activateFlightMode(none_flight_mode);
    }
    else if (flight_mode_pwm_signal >= PWM_SIGNAL_MID_LOW && flight_mode_pwm_signal < PWM_SIGNAL_MID_HIGH && drone->getFlightMode().type() != acro)
    {
        const auto acro_local = FLightModeAcroLocal();
        drone->activateFlightMode(acro_local);

        if (throttle_is_zero)
        {
            drone->enableMotors();
        }
        else
        {
            drone->disableMotors();
        }
    }
    else if (flight_mode_pwm_signal >= PWM_SIGNAL_MID_HIGH && drone->getFlightMode().type() != auto_level)
    {
        const auto auto_level_local = FlightModeAutoLevelLocal();
        drone->activateFlightMode(auto_level_local);

        if (throttle_is_zero)
        {
            drone->enableMotors();
        }
        else
        {
            drone->disableMotors();
        }
    }
}

void PwmReceiverController::setup()
{
    _receiver.setup();
}
