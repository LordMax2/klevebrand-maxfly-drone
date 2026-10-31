#pragma once

#include "../klevebrand_maxfly_drone.h"
#include "pwm_control_modes/pwm_receiver_control_mode_acro.h"
#include "pwm_control_modes/pwm_receiver_control_mode_auto_level.h"
#include "pwm_control_modes/pwm_receiver_control_mode_none.h"

static constexpr int CHANNEL_COUNT = 8;

template <ConceptPwmReceiverControlMode ...Modes>
class PwmReceiverController : Modes...
{
public:
    PwmReceiverController(
        int throttle_receiver_channel_number,
        int yaw_receiver_channel_number,
        int pitch_receiver_channel_number,
        int roll_receiver_channel_number,
        int flight_mode_receiver_channel_number,
        int autopilot_receiver_channel_number
    )
    {
        this->_throttle_receiver_channel_number = throttle_receiver_channel_number;
        this->_yaw_receiver_channel_number = yaw_receiver_channel_number;
        this->_pitch_receiver_channel_number = pitch_receiver_channel_number;
        this->_roll_receiver_channel_number = roll_receiver_channel_number;
        this->_flight_mode_receiver_channel_number = flight_mode_receiver_channel_number;
        this->_autopilot_receiver_channel_number = autopilot_receiver_channel_number;
    };

    static void setup();
    bool wantsControl() const;
    void applyAutopilot(KlevebrandMaxFlyDrone* drone);
    void apply(KlevebrandMaxFlyDrone* drone);

private:
    int _throttle_receiver_channel_number;
    int _yaw_receiver_channel_number;
    int _pitch_receiver_channel_number;
    int _roll_receiver_channel_number;
    int _flight_mode_receiver_channel_number;
    int _autopilot_receiver_channel_number;

    PwmReceiverControlModeNone _none_control_mode;
    PwmReceiverControlModeAcro _acro_control_mode;
    PwmReceiverControlModeAutoLevel _auto_level_control_mode;

    template<ConceptPwmReceiverControlMode Mode>
    bool tryApplyMode(KlevebrandMaxFlyDrone* drone, int throttle_pwm, int yaw_pwm, int pitch_pwm, int roll_pwm);

    static void setFlightMode(KlevebrandMaxFlyDrone* drone, int flight_mode_pwm);

    static int getChannelValue(int channel_number);

    static void recordPulseEdges();

    // Channels 1-8 come in on pads GPIO_B1_04 to GPIO_B1_11, which are GPIO2 bits 20-27.
    static constexpr int first_channel_gpio_bit = 20;
    static constexpr uint32_t channel_gpio_mask = 0xFFUL << first_channel_gpio_bit;

    // Ahead of the serial ports (64) and everything else (128), behind systick (32), which micros() relies on.
    static constexpr uint8_t pulse_interrupt_priority = 48;

    inline static volatile unsigned long _pulse_start_micros[CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
    inline static volatile int _pulse_widths[CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
};

#include "pwm_receiver_controller.ipp"
