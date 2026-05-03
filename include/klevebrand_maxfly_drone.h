#ifndef KLEVEBRAND_MAXFLY_DRONE_H
#define KLEVEBRAND_MAXFLY_DRONE_H

#include <Arduino.h>
#include "template_drone.h"
#include "servo_drone_motor.h"
#include "quadcopter_pid.h"
#include "bno08x_drone_gyro.h"
#include "hardware_processor_arduino.h"

class KlevebrandMaxFlyDrone : public TemplateDrone<QuadcopterPid> {
private:
    ServoDroneMotor *_motors;
    static constexpr int motor_pin_count = 4;

    int _motor_pins[motor_pin_count]{};

    ServoDroneMotor &motorLeftFront() const;

    ServoDroneMotor &motorRightFront() const;

    ServoDroneMotor &motorLeftBack() const;

    ServoDroneMotor &motorRightBack() const;

    Bno08xDroneGyro _gyro;
    BasePidRepository _pid_repository;
    BaseDronePosition _position;
    HardwareProcessorArduino _processor;

    void printThrottle(float delta_time_seconds);

    void attachMotors() const;

    void detachMotors() const;

public:
    KlevebrandMaxFlyDrone(ServoDroneMotor *motors, const int motor_pins[motor_pin_count]);

    void setup() override;

    bool run() override;

    void runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw, float delta_time_seconds) override;

    void enableMotors() override;

    void disableMotors() override;

    void setupMotors() override;

    void stopMotors() override;
};

#endif
