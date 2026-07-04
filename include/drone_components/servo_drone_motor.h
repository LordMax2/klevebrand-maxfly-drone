#pragma once

#include "Arduino.h"
#include <Servo.h>
#include "concept_drone_motor.h"

class ServoDroneMotor
{
public:
    ServoDroneMotor() {}

    void setup(int pin)
    {
        _pin = pin;
        _is_configured = true;
        attach();
    }

    void setSpeed(float percentage)
    {
        if (!_is_attached)
        {
            return;
        }

        int microseconds = map(percentage, 0, 100, 1000, 2000);

        microseconds = constrain(microseconds, 1000, 2000);

        _motor.writeMicroseconds(microseconds);
    }

    void attach()
    {
        if (!_is_configured || _is_attached)
        {
            return;
        }

        _motor.attach(_pin);
        _is_attached = true;
    }

    void detach()
    {
        if (!_is_attached)
        {
            return;
        }

        _motor.detach();
        _is_attached = false;
    }

    bool isAttached() const
    {
        return _is_attached;
    }

    static_assert(DroneMotorConcept<ServoDroneMotor>, "ServoDroneMotor does not implement DroneMotorConcept");

private:
    Servo _motor;
    int _pin = 0;
    bool _is_configured = false;
    bool _is_attached = false;
};
