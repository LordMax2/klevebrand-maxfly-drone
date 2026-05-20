#ifndef SERVO_DRONE_MOTOR_H
#define SERVO_DRONE_MOTOR_H

#include "Arduino.h"
#include "base_drone_motor.h"
#include <Servo.h>

class ServoDroneMotor : public BaseDroneMotor
{
public:
    ServoDroneMotor() : BaseDroneMotor() {}

    void setup(int pin)
    {
        _pin = pin;
        _is_configured = true;
        attach();
    }

    void setSpeed(float percentage) override
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

private:
    Servo _motor;
    int _pin = 0;
    bool _is_configured = false;
    bool _is_attached = false;
};

#endif // SERVO_DRONE_MOTOR_H
