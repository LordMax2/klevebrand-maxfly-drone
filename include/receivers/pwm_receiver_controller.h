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
    /*
     * Channels 1-8 come in on eight consecutive pads from GPIO_B1_<first_channel_pad>. They must lie within
     * GPIO_B1_00-15 (GPIO2 bits 16-31), whose interrupt the Teensy core never uses.
     */
    PwmReceiverController(
        uint8_t first_channel_pad,
        int throttle_receiver_channel_number,
        int yaw_receiver_channel_number,
        int pitch_receiver_channel_number,
        int roll_receiver_channel_number,
        int flight_mode_receiver_channel_number,
        int autopilot_receiver_channel_number
    ) : _first_channel_pad(first_channel_pad)
    {
        this->_throttle_receiver_channel_number = throttle_receiver_channel_number;
        this->_yaw_receiver_channel_number = yaw_receiver_channel_number;
        this->_pitch_receiver_channel_number = pitch_receiver_channel_number;
        this->_roll_receiver_channel_number = roll_receiver_channel_number;
        this->_flight_mode_receiver_channel_number = flight_mode_receiver_channel_number;
        this->_autopilot_receiver_channel_number = autopilot_receiver_channel_number;
    };

    void setup();
    bool wantsControl() const;
    void applyAutopilot(KlevebrandMaxFlyDrone* drone);
    void apply(KlevebrandMaxFlyDrone* drone);

private:
    const uint8_t _first_channel_pad;
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

    int getChannelValue(int channel_number) const;

    int firstChannelGpioBit() const;

    uint32_t channelGpioMask() const;

    static void onPulseEdgeInterrupt();

    void recordPulseEdges();

    // GPIO_B1_00 is GPIO2 bit 16, so channel 1 can start at GPIO_B1_00 up to GPIO_B1_08.
    static constexpr int first_b1_pad_gpio_bit = 16;
    static constexpr uint8_t last_first_channel_pad = 16 - CHANNEL_COUNT;

    // Ahead of the serial ports (64) and everything else (128), behind systick (32), which micros() relies on.
    static constexpr uint8_t pulse_interrupt_priority = 48;

    // The interrupt vector is a plain function, so it reaches the receiver through this. There is one receiver.
    inline static PwmReceiverController *_interrupt_receiver = nullptr;

    volatile unsigned long _pulse_start_micros[CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
    volatile int _pulse_widths[CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
};

#include "pwm_receiver_controller.ipp"
