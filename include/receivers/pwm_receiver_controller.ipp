#include "drone_components/flight_mode_acro_local.h"
#include "drone_components/flight_mode_auto_level_local.h"
#include "drone_components/flight_mode_none_local.h"
#include <PinChangeInterrupt.h>

constexpr int PWM_SIGNAL_MINIMUM = 1000;
constexpr int PWM_SIGNAL_MID_LOW = 1250;
constexpr int PWM_SIGNAL_MID_HIGH = 1750;

template <ConceptPwmReceiverControlMode ...Modes>
int PwmReceiverController<Modes...>::_channel_number_to_gpio_map_array[CHANNEL_COUNT] = {
    A8, A9, A10, A11, A12, A13, A14, A15
};

static auto none_flight_mode = FlightModeNoneLocal();
static auto acro_local = FlightModeAcroLocal();
static auto auto_level_local = FlightModeAutoLevelLocal();

template <ConceptPwmReceiverControlMode ...Modes>
bool PwmReceiverController<Modes...>::wantsControl()
{
    return true;
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
    if (_channel_number_to_gpio_map_array[0] != -1)
    {
        pinMode(_channel_number_to_gpio_map_array[0], INPUT);
        attachPCINT(digitalPinToPCINT(_channel_number_to_gpio_map_array[0]), recordPinChangePulseWidthChannel1, CHANGE);
    }

    if (_channel_number_to_gpio_map_array[1] != -1)
    {
        pinMode(_channel_number_to_gpio_map_array[1], INPUT);
        attachPCINT(digitalPinToPCINT(_channel_number_to_gpio_map_array[1]), recordPinChangePulseWidthChannel2, CHANGE);
    }

    if (_channel_number_to_gpio_map_array[2] != -1)
    {
        pinMode(_channel_number_to_gpio_map_array[2], INPUT);
        attachPCINT(digitalPinToPCINT(_channel_number_to_gpio_map_array[2]), recordPinChangePulseWidthChannel3, CHANGE);
    }

    if (_channel_number_to_gpio_map_array[3] != -1)
    {
        pinMode(_channel_number_to_gpio_map_array[3], INPUT);
        attachPCINT(digitalPinToPCINT(_channel_number_to_gpio_map_array[3]), recordPinChangePulseWidthChannel4, CHANGE);
    }

    if (_channel_number_to_gpio_map_array[4] != -1)
    {
        pinMode(_channel_number_to_gpio_map_array[4], INPUT);
        attachPCINT(digitalPinToPCINT(_channel_number_to_gpio_map_array[4]), recordPinChangePulseWidthChannel5, CHANGE);
    }

    if (_channel_number_to_gpio_map_array[5] != -1)
    {
        pinMode(_channel_number_to_gpio_map_array[5], INPUT);
        attachPCINT(digitalPinToPCINT(_channel_number_to_gpio_map_array[5]), recordPinChangePulseWidthChannel6, CHANGE);
    }

    if (_channel_number_to_gpio_map_array[6] != -1)
    {
        pinMode(_channel_number_to_gpio_map_array[6], INPUT);
        attachPCINT(digitalPinToPCINT(_channel_number_to_gpio_map_array[6]), recordPinChangePulseWidthChannel7, CHANGE);
    }

    if (_channel_number_to_gpio_map_array[7] != -1)
    {
        pinMode(_channel_number_to_gpio_map_array[7], INPUT);
        attachPCINT(digitalPinToPCINT(_channel_number_to_gpio_map_array[7]), recordPinChangePulseWidthChannel8, CHANGE);
    }
}

template <ConceptPwmReceiverControlMode ...Modes>
void PwmReceiverController<Modes...>::recordPinChangePulseWidth(const int channel_number)
{
    const int channelNumberIndex = channel_number - 1;
    const int pinState = digitalRead(_channel_number_to_gpio_map_array[channelNumberIndex]);
    const unsigned long timestamp_microseconds = micros();

    if (pinState == HIGH)
    {
        _pulse_start_micros[channelNumberIndex] = timestamp_microseconds;

        return;
    }

    if (_pulse_start_micros[channelNumberIndex] != 0)
    {
        _pulse_widths[channelNumberIndex] = static_cast<int>(timestamp_microseconds - _pulse_start_micros[
            channelNumberIndex]);
        _pulse_start_micros[channelNumberIndex] = 0;

        return;
    }
}

template <ConceptPwmReceiverControlMode ...Modes>
int PwmReceiverController<Modes...>::getChannelValue(const int channel_number)
{
    if (channel_number < 1 || channel_number > CHANNEL_COUNT)
    {
        return -1;
    }

    return constrain(_pulse_widths[channel_number - 1], 1000, 2000);
}
