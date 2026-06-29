#pragma once

#include "base_autopilot.h"
#include "template_drone.h"
#include "drone_components/servo_drone_motor.h"
#include "drone_components/quadcopter_pid.h"
#include "bno08x_drone_gyro.h"
#include "hardware_processor_arduino.h"
#include "quadcopter_position.h"

class KlevebrandMaxFlyDrone : public TemplateDrone<QuadcopterPid> {
    ServoDroneMotor *_motors;
    static constexpr int motor_pin_count = 4;
    int _motor_pins[motor_pin_count]{};

    ServoDroneMotor &motorLeftFront() const;
    ServoDroneMotor &motorRightFront() const;
    ServoDroneMotor &motorLeftBack() const;
    ServoDroneMotor &motorRightBack() const;

    Bno08xDroneGyro _gyro;
    BasePidRepository _pid_repository;
    QuadcopterPosition _position;
    HardwareProcessorArduino _processor;
    BaseAutopilot *_autopilot;

    bool _autopilot_enabled = false;

    void printThrottle(float delta_time_seconds);

    void attachMotors() const;

    void detachMotors() const;

public:
    KlevebrandMaxFlyDrone(ServoDroneMotor *motors, const int motor_pins[motor_pin_count]);
    ~KlevebrandMaxFlyDrone() override;

    void enableAutopilot();

    void disableAutopilot();

    bool isAutopilotEnabled() const;

    void setup() override;

    bool run() override;

    void runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw, float delta_time_seconds) override;

    void enableMotors() override;

    void disableMotors() override;

    void setupMotors() override;

    void stopMotors() override;
};
