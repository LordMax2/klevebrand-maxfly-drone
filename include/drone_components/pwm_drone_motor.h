#pragma once

#include "Arduino.h"

/*
 * ESC signal (1000-2000 µs pulses in a 400 Hz frame) from a FlexPWM pin through Teensy's analogWrite, so the timing
 * comes from the hardware and no interrupt can make it jitter.
 */
class PwmDroneMotor
{
public:
    static constexpr float frame_frequency_hz = 400.0f;

    PwmDroneMotor() = default;

    void setup(int pin)
    {
        _pin = pin;
        _is_configured = true;

        analogWriteResolution(write_resolution_bits);
        analogWriteFrequency(_pin, frame_frequency_hz);

        attach();
    }

    void setSpeed(float percentage)
    {
        if (!_is_attached)
        {
            return;
        }

        constexpr float pulse_range_microseconds = maximum_pulse_microseconds - minimum_pulse_microseconds;

        writePulseMicroseconds(minimum_pulse_microseconds + percentage / 100.0f * pulse_range_microseconds);
    }

    void attach()
    {
        if (!_is_configured || _is_attached)
        {
            return;
        }

        writePulseMicroseconds(minimum_pulse_microseconds);
        _is_attached = true;
    }

    void detach()
    {
        if (!_is_attached)
        {
            return;
        }

        // No pulses at all, so the ESC sees a lost signal rather than a throttle.
        analogWrite(_pin, 0);
        _is_attached = false;
    }

    bool isAttached() const
    {
        return _is_attached;
    }

private:
    static constexpr int write_resolution_bits = 15;
    static constexpr float minimum_pulse_microseconds = 1000.0f;
    static constexpr float maximum_pulse_microseconds = 2000.0f;

    int _pin = 0;
    bool _is_configured = false;
    bool _is_attached = false;

    void writePulseMicroseconds(float pulse_microseconds) const
    {
        constexpr float frame_microseconds = 1000000.0f / frame_frequency_hz;
        constexpr float full_scale_value = 1 << write_resolution_bits;

        const float constrained_pulse_microseconds =
            constrain(pulse_microseconds, minimum_pulse_microseconds, maximum_pulse_microseconds);
        const float duty_value = constrained_pulse_microseconds / frame_microseconds * full_scale_value;

        analogWrite(_pin, static_cast<int>(duty_value + 0.5f));
    }
};
