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
        _motor.attach(pin);
    }

    void setSpeed(float percentage) override
    {
        int microseconds = map(percentage, 0, 100, 1000, 2000);

        microseconds = constrain(microseconds, 1000, 2000);

        _motor.writeMicroseconds(microseconds);
    }
private:
    Servo _motor;
};

#endif // SERVO_DRONE_MOTOR_H