#include "pwm_receiver_controller.h"

#include "flight_mode_acro_local.h"
#include "flight_mode_auto_level_local.h"

constexpr int PWM_SIGNAL_MINIMUM = 1000;
constexpr int PWM_SIGNAL_MID_LOW = 1250;
constexpr int PWM_SIGNAL_MID_HIGH = 1750;
constexpr int PWM_SIGNAL_ENABLE_THRESHOLD = 1050;
constexpr int PWM_DEADZONE = 20;

constexpr float ACRO_EXPONENTIAL_INCREASE_RATE = 0.5f;
constexpr float MAX_ACRO_RATE_PITCH_ROLL = 300.0f;
constexpr float MAX_ACRO_RATE_YAW = 300.0f;

volatile int PwmReceiverController::_channel_number_to_gpio_map_array[CHANNEL_COUNT] = {
    A8, A9, A10, A11, A12, A13, A14, A15
};
volatile unsigned long PwmReceiverController::_pulse_start_micros[CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile int PwmReceiverController::_pulse_widths[CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};

bool PwmReceiverController::hasValidSignal()
{
    return getChannelValue(_throttle_receiver_channel_number) >= 1000 &&
        getChannelValue(_yaw_receiver_channel_number) >= 1000 &&
        getChannelValue(_pitch_receiver_channel_number) >= 1000 &&
        getChannelValue(_roll_receiver_channel_number) >= 1000 &&
        getChannelValue(_flight_mode_receiver_channel_number) >= 1000;
}

bool PwmReceiverController::wantsControl()
{
    return hasValidSignal();
}

void PwmReceiverController::apply(KlevebrandMaxFlyDrone* drone)
{
    setThrottleYawPitchRoll(drone);
    setFlightMode(drone);
}

static unsigned long last_yaw_set_timestamp_millis = 0;

void PwmReceiverController::setThrottleYawPitchRoll(KlevebrandMaxFlyDrone* drone)
{
    float throttle_value = map(getChannelValue(_throttle_receiver_channel_number), 1000, 2000, THROTTLE_MINIMUM,
                               THROTTLE_MAXIMUM);
    drone->setThrottle(throttle_value);

    if (drone->getFlightMode()->type() == acro)
    {
        float roll = applyExpo(normalizeChannel(_roll_receiver_channel_number), ACRO_EXPONENTIAL_INCREASE_RATE);
        float pitch = applyExpo(normalizeChannel(_pitch_receiver_channel_number), ACRO_EXPONENTIAL_INCREASE_RATE);
        float yaw = applyExpo(normalizeChannel(_yaw_receiver_channel_number), ACRO_EXPONENTIAL_INCREASE_RATE);

        drone->setDesiredRollAngle(-(roll * MAX_ACRO_RATE_PITCH_ROLL));
        drone->setDesiredPitchAngle(-(pitch * MAX_ACRO_RATE_PITCH_ROLL));
        drone->setDesiredYawAngle(-(yaw * MAX_ACRO_RATE_YAW));

        return;
    }

    if (drone->getFlightMode()->type() == auto_level)
    {
        float desired_yaw_angle = map(getChannelValue(_yaw_receiver_channel_number), 1000, 2000, 5, -5);
        if (desired_yaw_angle < 5 && desired_yaw_angle > -5)
        {
            desired_yaw_angle = 0;
        }
        else
        {
            last_yaw_set_timestamp_millis = millis();
        }

        if (millis() - last_yaw_set_timestamp_millis > 10)
        {
            const float new_desired_yaw_angle = drone->getDesiredYawAngle() + desired_yaw_angle;
            drone->setDesiredYawAngle(new_desired_yaw_angle);
        }

        float desired_pitch_angle = map(getChannelValue(_pitch_receiver_channel_number), 1000, 2000, -60, 60);
        drone->setDesiredPitchAngle(desired_pitch_angle);

        float desired_roll_angle = map(getChannelValue(_roll_receiver_channel_number), 1000, 2000, -60, 60);
        drone->setDesiredRollAngle(desired_roll_angle);

        return;
    }
}

void PwmReceiverController::setFlightMode(KlevebrandMaxFlyDrone* drone)
{
    int flight_mode_pwm_signal = getChannelValue(_flight_mode_receiver_channel_number);

    if (flight_mode_pwm_signal >= PWM_SIGNAL_MINIMUM && flight_mode_pwm_signal < PWM_SIGNAL_MID_LOW)
    {
        drone->disableMotors();

        static auto none_flight_mode = BaseFlightMode();
        drone->activateFlightMode(&none_flight_mode);
    }
    else if (flight_mode_pwm_signal >= PWM_SIGNAL_MID_LOW && flight_mode_pwm_signal < PWM_SIGNAL_MID_HIGH && drone->
        getFlightMode()->type() != acro)
    {
        static auto acro_local = FLightModeAcroLocal();
        drone->activateFlightMode(&acro_local);

        drone->enableMotors();
    }
    else if (flight_mode_pwm_signal >= PWM_SIGNAL_MID_HIGH && drone->getFlightMode()->type() != auto_level)
    {
        static auto auto_level_local = FlightModeAutoLevelLocal();
        drone->activateFlightMode(&auto_level_local);

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

void PwmReceiverController::recordPinChangePulseWidth(int channelNumber)
{
    int channelNumberIndex = channelNumber - 1;
    int pinState = digitalRead(_channel_number_to_gpio_map_array[channelNumberIndex]);

    if (pinState == HIGH)
    {
        _pulse_start_micros[channelNumberIndex] = micros();

        return;
    }

    if (_pulse_start_micros[channelNumberIndex] != 0)
    {
        _pulse_widths[channelNumberIndex] = (int)(micros() - _pulse_start_micros[channelNumberIndex]);
        _pulse_start_micros[channelNumberIndex] = 0;

        return;
    }
}

int PwmReceiverController::getChannelValue(int channelNumber)
{
    if (channelNumber < 1 || channelNumber > CHANNEL_COUNT)
    {
        return -1;
    }

    return constrain(_pulse_widths[channelNumber - 1], 1000, 2000);
}

float PwmReceiverController::normalizeChannel(int channel_number)
{
    const int centered = getChannelValue(channel_number) - 1500;

    if (abs(centered) < PWM_DEADZONE) return 0.0f;

    return constrain(centered / 500.0f, -1.0f, 1.0f);
}

float PwmReceiverController::applyExpo(float input, float expo)
{
    const float sign = (input >= 0.0f) ? 1.0f : -1.0f;

    const float abs_input = abs(input);

    return sign * (abs_input * abs_input * abs_input * expo + abs_input * (1.0f - expo));
}
