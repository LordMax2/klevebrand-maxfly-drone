#pragma once

#include "imxrt_gpio_pad.h"

/*
 * A status LED and an error LED: the error LED is lit while the drone starts up (sensors, GPS fix, modem), and the
 * status LED takes over once it is ready.
 */
class StatusLeds
{
public:
    StatusLeds(const ImxrtGpioPad &status_led, const ImxrtGpioPad &error_led)
        : _status_led(status_led), _error_led(error_led)
    {
    }

    void showStarting() const
    {
        _status_led.setModeOutput(false);
        _error_led.setModeOutput(true);
    }

    void showReady() const
    {
        _error_led.write(false);
        _status_led.write(true);
    }

private:
    const ImxrtGpioPad _status_led;
    const ImxrtGpioPad _error_led;
};
