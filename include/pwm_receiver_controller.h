#ifndef DRONE_PWM_RECEIVER_H
#define DRONE_PWM_RECEIVER_H

#include <Arduino.h>
#include <PinChangeInterrupt.h>

#include "klevebrand_maxfly_drone.h"

#define CHANNEL_COUNT 8

class PwmReceiverController
{
public:
    PwmReceiverController(
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
    bool hasValidSignal();
    bool wantsControl();
    void apply(KlevebrandMaxFlyDrone *drone);

private:
    int _throttle_receiver_channel_number;
    int _yaw_receiver_channel_number;
    int _pitch_receiver_channel_number;
    int _roll_receiver_channel_number;
    int _flight_mode_receiver_channel_number;

    void setThrottleYawPitchRoll(KlevebrandMaxFlyDrone *drone);
    void setFlightMode(KlevebrandMaxFlyDrone *drone);

    int getChannelValue(int channel_number);

    static void recordPinChangePulseWidth(int channel_number);
    static void recordPinChangePulseWidthChannel1() { recordPinChangePulseWidth(1); };
    static void recordPinChangePulseWidthChannel2() { recordPinChangePulseWidth(2); };
    static void recordPinChangePulseWidthChannel3() { recordPinChangePulseWidth(3); };
    static void recordPinChangePulseWidthChannel4() { recordPinChangePulseWidth(4); };
    static void recordPinChangePulseWidthChannel5() { recordPinChangePulseWidth(5); };
    static void recordPinChangePulseWidthChannel6() { recordPinChangePulseWidth(6); };
    static void recordPinChangePulseWidthChannel7() { recordPinChangePulseWidth(7); };
    static void recordPinChangePulseWidthChannel8() { recordPinChangePulseWidth(8); };

    static volatile int _channel_number_to_gpio_map_array[CHANNEL_COUNT];
    static volatile unsigned long _pulse_start_micros[CHANNEL_COUNT];
    static volatile int _pulse_widths[CHANNEL_COUNT];

    float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
};

#endif // DRONE_PWM_RECEIVER_H
