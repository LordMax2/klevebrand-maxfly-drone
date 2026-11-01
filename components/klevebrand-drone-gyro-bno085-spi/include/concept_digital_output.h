#pragma once

/*
 * An output line the gyro drives itself, like its reset line: an Arduino pin, or a pad that has no pin number.
 */
template <class T>
concept DigitalOutputConcept = requires(const T line, const bool level)
{
    line.setModeOutput(level);

    line.write(level);
};
