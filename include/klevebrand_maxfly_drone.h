#pragma once

#include "drone_components/servo_drone_motor.h"
#include "template_drone.h"
#include "drone_components/quadcopter_pid.h"
#include "drone_components/maxfly_position.h"
#include "bno08x_drone_gyro.h"
#include "hardware_processor_arduino.h"
#include "autopilot/autopilot_tilt.h"

using MaxFlyGyro = Bno08xDroneGyro;
using MaxFlyProcessor = HardwareProcessorArduino;
using MaxFlyPosition = MaxFlyPositionType<MaxFlyGyro>;
using MaxFlyPid = QuadcopterPid;
using MaxFlyDroneBase = TemplateDrone<MaxFlyPid, MaxFlyPosition, MaxFlyGyro, MaxFlyProcessor>;

class KlevebrandMaxFlyDrone : public MaxFlyDroneBase {
    ServoDroneMotor *_motors;
    static constexpr int motor_pin_count = 4;
    int _motor_pins[motor_pin_count]{};

    AutopilotTilt<MaxFlyPid, MaxFlyPosition, MaxFlyGyro, MaxFlyProcessor> _autopilot;
    bool _is_autopilot_enabled = false;

    static constexpr float AUTOPILOT_TEST_LATITUDE = 59.8586f;
    static constexpr float AUTOPILOT_TEST_LONGITUDE = 17.6389f;
    static constexpr float AUTOPILOT_TEST_ALTITUDE_METERS = 10.0f;

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

    void enableMotors() override;

    void disableMotors() override;

    void setupMotors();

    void stopMotors();

    bool isAutopilotEnabled() const;

    void enableAutopilot();

    void disableAutopilot();
};
