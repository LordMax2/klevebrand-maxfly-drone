#include "drone_pwm_receiver.h"

#include "flight_mode_acro_local.h"
#include "flight_mode_auto_level_local.h"

void DronePwmReceiver::setThrottleYawPitchRoll(KlevebrandMaxFlyDrone *drone)
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

void DronePwmReceiver::setFlightMode(KlevebrandMaxFlyDrone *drone)
{
    int flight_mode_pwm_signal = _receiver.getChannelValue(_flight_mode_receiver_channel_number);

    if (flight_mode_pwm_signal >= 1000 && flight_mode_pwm_signal < 1250)
    {
        drone->disableMotors();
    }
    else if (flight_mode_pwm_signal >= 1250 && flight_mode_pwm_signal < 1750)
    {
        drone->enableMotors();
        const auto acro_local = FLightModeAcroLocal();
        drone->activateFlightMode(acro_local);
    }
    else if (flight_mode_pwm_signal >= 1750)
    {
        drone->enableMotors();
        const auto auto_level_local = FlightModeAutoLevelLocal();
        drone->activateFlightMode(auto_level_local);
    }
}

void DronePwmReceiver::setup()
{
    _receiver.setup();
}
