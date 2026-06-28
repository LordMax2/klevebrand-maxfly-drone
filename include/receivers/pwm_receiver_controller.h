#ifndef DRONE_PWM_RECEIVER_H
#define DRONE_PWM_RECEIVER_H

#include "../klevebrand_maxfly_drone.h"

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

    static void setup();
    bool hasValidSignal() const;
    bool wantsControl() const;
    void apply(KlevebrandMaxFlyDrone *drone) const;

private:
    int _throttle_receiver_channel_number;
    int _yaw_receiver_channel_number;
    int _pitch_receiver_channel_number;
    int _roll_receiver_channel_number;
    int _flight_mode_receiver_channel_number;

    void setThrottleYawPitchRoll(KlevebrandMaxFlyDrone *drone) const;
    void setFlightMode(KlevebrandMaxFlyDrone *drone) const;

    static int getChannelValue(int channel_number);

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

    static float normalizeChannel(int channel_number);

    static float applyExpo(float input, float expo);
    static float applySlew(float current, float target, float dt);
};

#endif // DRONE_PWM_RECEIVER_H
