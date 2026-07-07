#pragma once

#include "drone_components/servo_drone_motor.h"
#include "template_drone.h"
#include "drone_components/quadcopter_pid.h"
#include "bno08x_drone_gyro.h"
#include "hardware_processor_arduino.h"
#include "quadcopter_position.h"

using MaxFlyGyro = Bno08xDroneGyro;
using MaxFlyProcessor = HardwareProcessorArduino;
using MaxFlyPosition = QuadcopterPosition<MaxFlyGyro>;
using MaxFlyPid = QuadcopterPid;
using MaxFlyDroneBase = TemplateDrone<MaxFlyPid, MaxFlyPosition, MaxFlyGyro, MaxFlyProcessor>;

class KlevebrandMaxFlyDrone : public MaxFlyDroneBase {
    ServoDroneMotor *_motors;
    static constexpr int motor_pin_count = 4;
    int _motor_pins[motor_pin_count]{};

    ServoDroneMotor &motorLeftFront() const;
    ServoDroneMotor &motorRightFront() const;
    ServoDroneMotor &motorLeftBack() const;
    ServoDroneMotor &motorRightBack() const;

    void printThrottle(float delta_time_seconds);

    void attachMotors() const;

    void detachMotors() const;

public:
    static constexpr int gyro_reset_pin = 10;

    KlevebrandMaxFlyDrone(ServoDroneMotor *motors, const int motor_pins[motor_pin_count]);
    ~KlevebrandMaxFlyDrone() override;

    void setup();

    bool run();

    void runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw, float delta_time_seconds);

    void enableMotors();

    void disableMotors();

    void setupMotors();

    void stopMotors();
};
