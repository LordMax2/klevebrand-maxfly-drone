#include "receivers/pwm_receiver_controller.h"

#include "drone_components/flight_mode_acro_local.h"
#include "drone_components/flight_mode_auto_level_local.h"
#include <PinChangeInterrupt.h>

constexpr int PWM_SIGNAL_MINIMUM = 1000;
constexpr int PWM_SIGNAL_MID_LOW = 1250;
constexpr int PWM_SIGNAL_MID_HIGH = 1750;
constexpr int PWM_SIGNAL_ENABLE_THRESHOLD = 1050;
constexpr int PWM_DEADZONE = 20;

constexpr float ACRO_EXPONENTIAL_INCREASE_RATE = 0.5f;
constexpr float MAX_ACRO_RATE_PITCH_ROLL = 300.0f;
constexpr float MAX_ACRO_RATE_YAW = 300.0f;

constexpr float YAW_STEP_EXPONENTIAL_INCREASE_RATE = 0.5f;
constexpr float MAX_YAW_STEP_DEGREES = 5.0f;

static constexpr float MAX_RATE_DEG_PER_SEC = 135.0f;

volatile int PwmReceiverController::_channel_number_to_gpio_map_array[CHANNEL_COUNT] = {
    A8, A9, A10, A11, A12, A13, A14, A15
};
volatile unsigned long PwmReceiverController::_pulse_start_micros[CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile int PwmReceiverController::_pulse_widths[CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};

bool PwmReceiverController::hasValidSignal() const
{
    return getChannelValue(_throttle_receiver_channel_number) >= 1000 &&
        getChannelValue(_yaw_receiver_channel_number) >= 1000 &&
        getChannelValue(_pitch_receiver_channel_number) >= 1000 &&
        getChannelValue(_roll_receiver_channel_number) >= 1000 &&
        getChannelValue(_flight_mode_receiver_channel_number) >= 1000;
}

bool PwmReceiverController::wantsControl() const
{
    return hasValidSignal();
}

void PwmReceiverController::apply(KlevebrandMaxFlyDrone* drone) const
{
    setThrottleYawPitchRoll(drone);
    setFlightMode(drone);
}

static unsigned long last_yaw_set_timestamp_milliseconds = 0;

static float getSlewTimeDelta()
{
    static unsigned long last_slew_timestamp_microseconds = 0;

    const unsigned long timestamp_microseconds = micros();
    if (last_slew_timestamp_microseconds == 0)
        last_slew_timestamp_microseconds = timestamp_microseconds;

    float dt = (timestamp_microseconds - last_slew_timestamp_microseconds) / 1000000.0f;
    last_slew_timestamp_microseconds = timestamp_microseconds;
    if (dt > 0.1f)
        dt = 0.1f;

    return dt;
}

void PwmReceiverController::setThrottleYawPitchRoll(KlevebrandMaxFlyDrone* drone) const
{
    const float throttle_value = map(getChannelValue(_throttle_receiver_channel_number), 1000, 2000, THROTTLE_MINIMUM,
                               THROTTLE_MAXIMUM);
    drone->setThrottle(throttle_value);

    if (drone->getControlModeType() == acro)
    {
        const float roll = applyExpo(normalizeChannel(_roll_receiver_channel_number), ACRO_EXPONENTIAL_INCREASE_RATE);
        const float pitch = applyExpo(normalizeChannel(_pitch_receiver_channel_number), ACRO_EXPONENTIAL_INCREASE_RATE);
        const float yaw = applyExpo(normalizeChannel(_yaw_receiver_channel_number), ACRO_EXPONENTIAL_INCREASE_RATE);

        drone->setDesiredRollAngle(-(roll * MAX_ACRO_RATE_PITCH_ROLL));
        drone->setDesiredPitchAngle(-(pitch * MAX_ACRO_RATE_PITCH_ROLL));
        drone->setDesiredYawAngle(-(yaw * MAX_ACRO_RATE_YAW));

        return;
    }

    if (drone->getControlModeType() == auto_level)
    {
        const float yaw_stick = normalizeChannel(_yaw_receiver_channel_number);
        const float yaw_increment = -applyExpo(yaw_stick, YAW_STEP_EXPONENTIAL_INCREASE_RATE) * MAX_YAW_STEP_DEGREES;

        const auto timestamp_milliseconds = millis();

        if (timestamp_milliseconds - last_yaw_set_timestamp_milliseconds > 50)
        {
            const float new_desired_yaw_angle = constrain(drone->getDesiredYawAngle() + yaw_increment, -180.0f, 180.0f);
            drone->setDesiredYawAngle(new_desired_yaw_angle);

            last_yaw_set_timestamp_milliseconds = timestamp_milliseconds;
        }

        const float dt = getSlewTimeDelta();

        const float desired_pitch_angle = map(getChannelValue(_pitch_receiver_channel_number), 1000, 2000, 60, -60);
        const float current_pitch_angle = drone->getDesiredPitchAngle();
        const float slew_limited_desired_pitch = applySlew(current_pitch_angle, desired_pitch_angle, dt);
        drone->setDesiredPitchAngle(slew_limited_desired_pitch);

        const float desired_roll_angle = map(getChannelValue(_roll_receiver_channel_number), 1000, 2000, 60, -60);
        const float current_roll_angle = drone->getDesiredRollAngle();
        const float slew_limited_desired_roll = applySlew(current_roll_angle, desired_roll_angle, dt);
        drone->setDesiredRollAngle(slew_limited_desired_roll);

        return;
    }
}

void PwmReceiverController::setFlightMode(KlevebrandMaxFlyDrone* drone) const
{
    int flight_mode_pwm_signal = getChannelValue(_flight_mode_receiver_channel_number);

    if (flight_mode_pwm_signal >= PWM_SIGNAL_MINIMUM && flight_mode_pwm_signal < PWM_SIGNAL_MID_LOW)
    {
        drone->disableMotors();

        static auto none_flight_mode = BaseControlMode();
        drone->activateControlMode(&none_flight_mode);
    }
    else if (flight_mode_pwm_signal >= PWM_SIGNAL_MID_LOW && flight_mode_pwm_signal < PWM_SIGNAL_MID_HIGH && drone->
        getControlModeType() != acro)
    {
        static auto acro_local = FLightModeAcroLocal();
        drone->activateControlMode(&acro_local);

        drone->enableMotors();
    }
    else if (flight_mode_pwm_signal >= PWM_SIGNAL_MID_HIGH && drone->getControlModeType() != auto_level)
    {
        static auto auto_level_local = FlightModeAutoLevelLocal();
        drone->activateControlMode(&auto_level_local);

        drone->enableMotors();
    }
}

void PwmReceiverController::setup()
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

void PwmReceiverController::recordPinChangePulseWidth(const int channelNumber)
{
    const int channelNumberIndex = channelNumber - 1;
    const int pinState = digitalRead(_channel_number_to_gpio_map_array[channelNumberIndex]);
    const unsigned long timestamp_microseconds = micros();

    if (pinState == HIGH)
    {
        _pulse_start_micros[channelNumberIndex] = timestamp_microseconds;

        return;
    }

    if (_pulse_start_micros[channelNumberIndex] != 0)
    {
        _pulse_widths[channelNumberIndex] = static_cast<int>(timestamp_microseconds - _pulse_start_micros[channelNumberIndex]);
        _pulse_start_micros[channelNumberIndex] = 0;

        return;
    }
}

int PwmReceiverController::getChannelValue(const int channel_number)
{
    if (channel_number < 1 || channel_number > CHANNEL_COUNT)
    {
        return -1;
    }

    return constrain(_pulse_widths[channel_number - 1], 1000, 2000);
}

float PwmReceiverController::normalizeChannel(const int channel_number)
{
    const int centered = getChannelValue(channel_number) - 1500;

    if (abs(centered) < PWM_DEADZONE) return 0.0f;

    return constrain(centered / 500.0f, -1.0f, 1.0f);
}

float PwmReceiverController::applyExpo(const float input, const float expo)
{
    const float sign = (input >= 0.0f) ? 1.0f : -1.0f;

    const float abs_input = abs(input);

    return sign * (abs_input * abs_input * abs_input * expo + abs_input * (1.0f - expo));
}

float PwmReceiverController::applySlew(const float current, const float target, const float dt)
{
    const float error = target - current;

    const auto max_step = MAX_RATE_DEG_PER_SEC * dt;

    const float limited_change = constrain(error, -max_step, max_step);

    return current + limited_change;
}
