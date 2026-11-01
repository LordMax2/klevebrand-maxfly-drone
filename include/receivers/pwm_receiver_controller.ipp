#include "drone_components/flight_mode_acro_local.h"
#include "drone_components/flight_mode_auto_level_local.h"
#include "drone_components/flight_mode_none_local.h"
#include "imxrt_gpio_pad.h"

constexpr int PWM_SIGNAL_MINIMUM = 1000;
constexpr int PWM_SIGNAL_MID_LOW = 1250;
constexpr int PWM_SIGNAL_MID_HIGH = 1750;
constexpr int PWM_AUTOPILOT_ON_THRESHOLD = 1500;

static auto none_flight_mode = FlightModeNoneLocal();
static auto acro_local = FlightModeAcroLocal();
static auto auto_level_local = FlightModeAutoLevelLocal();

template <ConceptPwmReceiverControlMode ...Modes>
bool PwmReceiverController<Modes...>::wantsControl() const
{
    return getChannelValue(_autopilot_receiver_channel_number) < PWM_AUTOPILOT_ON_THRESHOLD;
}

template <ConceptPwmReceiverControlMode ...Modes>
void PwmReceiverController<Modes...>::applyAutopilot(KlevebrandMaxFlyDrone* drone)
{
    if (drone == nullptr)
    {
        return;
    }

    if (wantsControl())
    {
        if (drone->isAutopilotEnabled())
        {
            drone->disableAutopilot();
        }

        return;
    }

    if (drone->isGpsReady() && !drone->isAutopilotEnabled())
    {
        drone->enableAutopilot();
    }
}

template <ConceptPwmReceiverControlMode ...Modes>
void PwmReceiverController<Modes...>::apply(KlevebrandMaxFlyDrone* drone)
{
    const int throttle_pwm = getChannelValue(_throttle_receiver_channel_number);
    const int yaw_pwm = getChannelValue(_yaw_receiver_channel_number);
    const int pitch_pwm = getChannelValue(_pitch_receiver_channel_number);
    const int roll_pwm = getChannelValue(_roll_receiver_channel_number);
    const int flight_mode_pwm = getChannelValue(_flight_mode_receiver_channel_number);

    setFlightMode(drone, flight_mode_pwm);

    (void)((tryApplyMode<Modes>(drone, throttle_pwm, yaw_pwm, pitch_pwm, roll_pwm) || ...));
}

template <ConceptPwmReceiverControlMode ... Modes>
template <ConceptPwmReceiverControlMode Mode>
bool PwmReceiverController<Modes...>::tryApplyMode(KlevebrandMaxFlyDrone* drone, int throttle_pwm, int yaw_pwm, int pitch_pwm, int roll_pwm)
{
    Mode& mode = static_cast<Mode&>(*this);

    if (drone->getControlMode() == mode.controlModeType())
    {
        mode.applyThrottleYawPitchRoll(drone, throttle_pwm, yaw_pwm, pitch_pwm, roll_pwm);
        return true;
    }

    return false;
}

template <ConceptPwmReceiverControlMode ...Modes>
void PwmReceiverController<Modes...>::setFlightMode(KlevebrandMaxFlyDrone* drone, const int flight_mode_pwm)
{
    if (flight_mode_pwm >= PWM_SIGNAL_MINIMUM && flight_mode_pwm < PWM_SIGNAL_MID_LOW)
    {
        drone->disableMotors();

        drone->activateControlMode(&none_flight_mode);
    }
    else if (flight_mode_pwm >= PWM_SIGNAL_MID_LOW && flight_mode_pwm < PWM_SIGNAL_MID_HIGH && drone->
        getControlMode() != acro)
    {
        drone->activateControlMode(&acro_local);

        drone->enableMotors();
    }
    else if (flight_mode_pwm >= PWM_SIGNAL_MID_HIGH && drone->getControlMode() != auto_level)
    {
        drone->activateControlMode(&auto_level_local);

        drone->enableMotors();
    }
}

template <ConceptPwmReceiverControlMode ...Modes>
void PwmReceiverController<Modes...>::setup()
{
    if (_first_channel_pad > last_first_channel_pad)
    {
        Serial.println("RECEIVER CHANNELS MUST BE ON PADS GPIO_B1_00 TO GPIO_B1_15.");

        return;
    }

    // The Teensy core has no pin numbers for these pads. Their mux and pad registers are consecutive.
    volatile uint32_t *const first_mux_register = &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_00 + _first_channel_pad;
    volatile uint32_t *const first_pad_register = &IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_00 + _first_channel_pad;
    const uint32_t channel_gpio_mask = channelGpioMask();

    CCM_CCGR0 = CCM_CCGR0 | CCM_CCGR0_GPIO2(CCM_CCGR_ON);

    GPIO2_IMR = GPIO2_IMR & ~channel_gpio_mask;
    GPIO2_GDIR = GPIO2_GDIR & ~channel_gpio_mask;

    for (int channel_index = 0; channel_index < CHANNEL_COUNT; channel_index++)
    {
        first_pad_register[channel_index] = IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(0) | IOMUXC_PAD_HYS;
        first_mux_register[channel_index] = ImxrtGpioPad::gpio_mux_mode;
    }

    // The core routes every pad to the fast GPIO7 at startup. These eight go back to GPIO2, whose interrupt for
    // bits 16-31 the core never uses.
    IOMUXC_GPR_GPR27 = IOMUXC_GPR_GPR27 & ~channel_gpio_mask;

    GPIO2_EDGE_SEL = GPIO2_EDGE_SEL | channel_gpio_mask;
    GPIO2_ISR = channel_gpio_mask;

    _interrupt_receiver = this;

    GPIO2_IMR = GPIO2_IMR | channel_gpio_mask;

    attachInterruptVector(IRQ_GPIO2_16_31, onPulseEdgeInterrupt);
    NVIC_SET_PRIORITY(IRQ_GPIO2_16_31, pulse_interrupt_priority);
    NVIC_ENABLE_IRQ(IRQ_GPIO2_16_31);
}

template <ConceptPwmReceiverControlMode ...Modes>
int PwmReceiverController<Modes...>::firstChannelGpioBit() const
{
    return first_b1_pad_gpio_bit + _first_channel_pad;
}

template <ConceptPwmReceiverControlMode ...Modes>
uint32_t PwmReceiverController<Modes...>::channelGpioMask() const
{
    return ((1UL << CHANNEL_COUNT) - 1) << firstChannelGpioBit();
}

template <ConceptPwmReceiverControlMode ...Modes>
void PwmReceiverController<Modes...>::onPulseEdgeInterrupt()
{
    _interrupt_receiver->recordPulseEdges();
}

template <ConceptPwmReceiverControlMode ...Modes>
void PwmReceiverController<Modes...>::recordPulseEdges()
{
    const int first_channel_gpio_bit = firstChannelGpioBit();
    const uint32_t edges = GPIO2_ISR & channelGpioMask();
    const uint32_t levels = GPIO2_PSR;
    const unsigned long timestamp_microseconds = micros();

    GPIO2_ISR = edges;

    for (int channel_index = 0; channel_index < CHANNEL_COUNT; channel_index++)
    {
        const uint32_t channel_bit = 1UL << (first_channel_gpio_bit + channel_index);

        if ((edges & channel_bit) == 0)
        {
            continue;
        }

        if ((levels & channel_bit) != 0)
        {
            _pulse_start_micros[channel_index] = timestamp_microseconds;
        }
        else if (_pulse_start_micros[channel_index] != 0)
        {
            const unsigned long pulse_start_microseconds = _pulse_start_micros[channel_index];

            _pulse_widths[channel_index] = static_cast<int>(timestamp_microseconds - pulse_start_microseconds);
            _pulse_start_micros[channel_index] = 0;
        }
    }

    // Let the flag clear reach the GPIO block before returning, or the interrupt fires again straight away.
    asm volatile("dsb");
}

template <ConceptPwmReceiverControlMode ...Modes>
int PwmReceiverController<Modes...>::getChannelValue(const int channel_number) const
{
    if (channel_number < 1 || channel_number > CHANNEL_COUNT)
    {
        return -1;
    }

    const int pulse_width_microseconds = _pulse_widths[channel_number - 1];

    return constrain(pulse_width_microseconds, 1000, 2000);
}
