#pragma once

#include "../klevebrand_maxfly_drone.h"
#include "pwm_control_modes/pwm_receiver_control_mode_acro.h"
#include "pwm_control_modes/pwm_receiver_control_mode_auto_level.h"
#include "pwm_control_modes/pwm_receiver_control_mode_none.h"

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
        this->_throttle_receiver_channel_number = throttle_receiver_channel_number;
        this->_yaw_receiver_channel_number = yaw_receiver_channel_number;
        this->_pitch_receiver_channel_number = pitch_receiver_channel_number;
        this->_roll_receiver_channel_number = roll_receiver_channel_number;
        this->_flight_mode_receiver_channel_number = flight_mode_receiver_channel_number;

        _control_modes[0] = &_none_control_mode;
        _control_modes[1] = &_acro_control_mode;
        _control_modes[2] = &_auto_level_control_mode;
    };

    static void setup();
    static bool wantsControl();
    void apply(KlevebrandMaxFlyDrone *drone) const;

private:
    int _throttle_receiver_channel_number;
    int _yaw_receiver_channel_number;
    int _pitch_receiver_channel_number;
    int _roll_receiver_channel_number;
    int _flight_mode_receiver_channel_number;

    PwmReceiverControlModeNone _none_control_mode;
    PwmReceiverControlModeAcro _acro_control_mode;
    PwmReceiverControlModeAutoLevel _auto_level_control_mode;
    const BasePwmReceiverControlMode* _control_modes[3]{};

    static void setFlightMode(KlevebrandMaxFlyDrone *drone, int flight_mode_pwm);

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

    static int _channel_number_to_gpio_map_array[CHANNEL_COUNT];
    static unsigned long _pulse_start_micros[CHANNEL_COUNT];
    static int _pulse_widths[CHANNEL_COUNT];
};
