#pragma once

#include "imxrt_gpio_pad.h"

/*
 * The status LED (GPIO_EMC_20) and error LED (GPIO_EMC_21): the error LED is lit while the drone starts up
 * (sensors, GPS fix, modem), and the status LED takes over once it is ready.
 */
class StatusLeds
{
public:
    static void showStarting()
    {
        statusLed().setModeOutput(false);
        errorLed().setModeOutput(true);
    }

    static void showReady()
    {
        errorLed().write(false);
        statusLed().write(true);
    }

private:
    static ImxrtGpioPad statusLed()
    {
        return ImxrtGpioPad::emc(20);
    }

    static ImxrtGpioPad errorLed()
    {
        return ImxrtGpioPad::emc(21);
    }
};
