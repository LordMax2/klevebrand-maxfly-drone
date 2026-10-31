#pragma once

#include "drone_components/pwm_drone_motor.h"
#include "template_drone.h"
#include "drone_components/quadcopter_pid.h"
#include "drone_components/maxfly_position.h"
#include "drone_components/bno08x_spi_drone_gyro.h"
#include "hardware_processor_arduino.h"
#include "autopilot/autopilot_tilt.h"

using MaxFlyGyro = Bno08xSpiDroneGyro;
using MaxFlyProcessor = HardwareProcessorArduino;
using MaxFlyPosition = MaxFlyPositionType<MaxFlyGyro>;
using MaxFlyPid = QuadcopterPid;
using MaxFlyDroneBase = TemplateDrone<MaxFlyPid, MaxFlyPosition, MaxFlyGyro, MaxFlyProcessor>;

class KlevebrandMaxFlyDrone : public MaxFlyDroneBase {
    PwmDroneMotor *_motors;
    static constexpr int motor_pin_count = 4;
    int _motor_pins[motor_pin_count]{};

    AutopilotTilt<MaxFlyPid, MaxFlyPosition, MaxFlyGyro, MaxFlyProcessor> _autopilot;
    bool _is_autopilot_enabled = false;
    float _autopilot_target_latitude = 0.0f;
    float _autopilot_target_longitude = 0.0f;
    float _autopilot_target_altitude = 0.0f;

    PwmDroneMotor &motorLeftFront() const;
    PwmDroneMotor &motorRightFront() const;
    PwmDroneMotor &motorLeftBack() const;
    PwmDroneMotor &motorRightBack() const;

    void printThrottle(float delta_time_seconds);

    void attachMotors() const;

    void detachMotors() const;

public:
    // The BNO085 reset line: pad GPIO_EMC_10, which has no Teensy pin number.
    static constexpr int gyro_reset_pad = 10;

    KlevebrandMaxFlyDrone(PwmDroneMotor *motors, const int motor_pins[motor_pin_count]);
    ~KlevebrandMaxFlyDrone() override;

    void setup();

    bool run();

    void runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw, float delta_time_seconds);

    void enableMotors() override;

    void disableMotors() override;

    void setupMotors();

    void stopMotors();

    bool isAutopilotEnabled() const;

    bool isGpsReady() const;

    void enableAutopilot();

    void disableAutopilot();

    void setAutopilotHorizontalTarget(float latitude, float longitude);
};
