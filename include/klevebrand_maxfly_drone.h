#pragma once

#include "drone_components/pwm_drone_motor.h"
#include "template_drone.h"
#include "drone_components/quadcopter_pid.h"
#include "bno08x_spi_drone_gyro.h"
#include "drone_bmp581_neom9n_position.h"
#include "hardware_processor_arduino.h"
#include "imxrt_gpio_pad.h"
#include "autopilot/autopilot_tilt.h"

using MaxFlyGyro = Bno08xSpiDroneGyro<ImxrtGpioPad>;
using MaxFlyProcessor = HardwareProcessorArduino;
using MaxFlyPosition = DroneBmp581Neom9nPosition<MaxFlyGyro>;
using MaxFlyPid = QuadcopterPid;
using MaxFlyDroneBase = TemplateDrone<MaxFlyPid, MaxFlyPosition, MaxFlyGyro, MaxFlyProcessor>;

class KlevebrandMaxFlyDrone : public MaxFlyDroneBase {
    PwmDroneMotor *_motors;
    static constexpr int motor_pin_count = 4;
    int _motor_pins[motor_pin_count]{};
    const ImxrtGpioPad _output_enable_line;

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
    /*
     * The gyro, the position and the motors are constructed by the application with the board's wiring.
     * output_enable_line is OUT_EN_N of the output buffers, which pass the motor signals only while it is low (a
     * pull-up keeps them off at boot).
     */
    KlevebrandMaxFlyDrone(MaxFlyGyro &drone_gyro, MaxFlyPosition &drone_position, PwmDroneMotor *motors,
                          const int motor_pins[motor_pin_count], const ImxrtGpioPad &output_enable_line);
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
