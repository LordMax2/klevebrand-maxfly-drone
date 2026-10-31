#pragma once

#include <Arduino.h>

/*
 * A GPIO on an i.MX RT1062 pad that has no Teensy pin number, driven through the same registers the core uses.
 */
class ImxrtGpioPad
{
public:
    // ALT5 is the GPIO function on every pad. SION keeps the input path on, like the core's pinMode().
    static constexpr uint32_t gpio_mux_mode = 5 | 0x10;

    // Pad GPIO_EMC_<pad_number>. GPIO_EMC_00-31 are GPIO4 bits 0-31 and GPIO_EMC_32-41 are GPIO3 bits 18-27; the
    // Teensy startup code routes both to their fast aliases, GPIO9 and GPIO8. The pads' registers are consecutive.
    static ImxrtGpioPad emc(const uint8_t pad_number)
    {
        volatile uint32_t &mux_register = (&IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_00)[pad_number];
        volatile uint32_t &pad_register = (&IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_00)[pad_number];

        if (pad_number < 32)
        {
            return {mux_register, pad_register, IMXRT_GPIO9, pad_number};
        }

        return {mux_register, pad_register, IMXRT_GPIO8, static_cast<uint8_t>(pad_number - 14)};
    }

    // Drive the pad, latching the level first so the line never glitches to the other one.
    void setModeOutput(const bool initial_level) const
    {
        write(initial_level);

        _gpio.GDIR = _gpio.GDIR | _mask;
        _pad_register = IOMUXC_PAD_DSE(7);
        _mux_register = gpio_mux_mode;
    }

    void write(const bool level) const
    {
        if (level)
        {
            _gpio.DR_SET = _mask;
        }
        else
        {
            _gpio.DR_CLEAR = _mask;
        }
    }

private:
    volatile uint32_t &_mux_register;
    volatile uint32_t &_pad_register;
    IMXRT_GPIO_t &_gpio;
    const uint32_t _mask;

    ImxrtGpioPad(volatile uint32_t &mux_register, volatile uint32_t &pad_register, IMXRT_GPIO_t &gpio,
                 const uint8_t bit)
        : _mux_register(mux_register), _pad_register(pad_register), _gpio(gpio), _mask(1UL << bit)
    {
    }
};
