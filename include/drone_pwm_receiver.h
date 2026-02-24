#ifndef DRONE_PWM_RECEIVER_H
#define DRONE_PWM_RECEIVER_H

#include "pwm_receiver.h"
#include "klevebrand_maxfly_drone.h"

class DronePwmReceiver
{
public:
    DronePwmReceiver(
        int throttle_receiver_channel_number,
        int yaw_receiver_channel_number,
        int pitch_receiver_channel_number,
        int roll_receiver_channel_number,
        int flight_mode_receiver_channel_number
    )
    {
        /*
         * Map the receiver channel numbers
         */
        this->_throttle_receiver_channel_number = throttle_receiver_channel_number;
        this->_yaw_receiver_channel_number = yaw_receiver_channel_number;
        this->_pitch_receiver_channel_number = pitch_receiver_channel_number;
        this->_roll_receiver_channel_number = roll_receiver_channel_number;
        this->_flight_mode_receiver_channel_number = flight_mode_receiver_channel_number;
    };

    void setup();
    void setThrottleYawPitchRoll(KlevebrandMaxFlyDrone *drone);
    void setFlightMode(KlevebrandMaxFlyDrone *drone);

private:
    PwmReceiver _receiver;

    int _throttle_receiver_channel_number;
    int _yaw_receiver_channel_number;
    int _pitch_receiver_channel_number;
    int _roll_receiver_channel_number;
    int _flight_mode_receiver_channel_number;

    float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
};

#endif // DRONE_PWM_RECEIVER_H