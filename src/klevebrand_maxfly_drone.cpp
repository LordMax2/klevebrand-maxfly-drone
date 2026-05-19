#include "klevebrand_maxfly_drone.h"

KlevebrandMaxFlyDrone::KlevebrandMaxFlyDrone(ServoDroneMotor *motors, const int motor_pins[motor_pin_count])
    : TemplateDrone(500, 200, 10000, &_processor, &_gyro, &_pid_repository, &_position), _motors(motors),
      _gyro(10), _position(&_gyro) {
    for (int i = 0; i < motor_pin_count; i++) {
        _motor_pins[i] = motor_pins[i];
    }
}

ServoDroneMotor &KlevebrandMaxFlyDrone::motorLeftFront() const {
    return _motors[0];
}

ServoDroneMotor &KlevebrandMaxFlyDrone::motorRightFront() const {
    return _motors[1];
}

ServoDroneMotor &KlevebrandMaxFlyDrone::motorLeftBack() const {
    return _motors[2];
}

ServoDroneMotor &KlevebrandMaxFlyDrone::motorRightBack() const {
    return _motors[3];
}

void KlevebrandMaxFlyDrone::setup() {
    _processor.setup();

    _processor.print("STARTING DRONE...");

    _position.setup();

    _gyro.setup();

    pid_repository->setup();

    setupMotors();

    static auto none_flight_mode = BaseControlMode();
    activateControlMode(&none_flight_mode);

    processor->print("DRONE STARTED!");
}

static unsigned long last_run_start_micros_timestamp = 0;
static unsigned long last_gyro_fetch_duration = 0;

bool KlevebrandMaxFlyDrone::run() {
    if (delayToKeepFeedbackLoopHz(last_run_start_micros_timestamp - last_gyro_fetch_duration) > 0) {
        _position.run(false);

        return false;
    }

    const unsigned long current_time = timestampMicroseconds();
    const unsigned long delta_time = current_time - last_run_start_micros_timestamp;
    const float delta_time_seconds = delta_time / 1000000.0f;
    last_run_start_micros_timestamp = current_time;

    updateGyro();

    last_gyro_fetch_duration = timestampMicroseconds() - current_time;

    _position.run(true);

    if (hasLostConnection() && false) {
        // If connection is dead, stop the drone
        disableMotors();

        // Serial.println("LOST CONNECTION");
    } else if (!isMotorsEnabled() && false) {
        // If the motors are diabled, stop the drone
        disableMotors();

        // Serial.println("MOTORS DISABLED");
    } else {
        const float gyro_roll = getRoll();
        const float gyro_pitch = getPitch();
        float gyro_yaw = getYaw();

        // The IMU on the drone seems to be oriented in the wrong direction, where backwards points to north, flip it 180 deg
        if (getControlModeType() == auto_level) {
            float gyro_yaw_rotated_180 = gyro_yaw + 180;
            if (gyro_yaw_rotated_180 >= 180) gyro_yaw_rotated_180 -= 360;

            gyro_yaw = gyro_yaw_rotated_180 * -1;
        }

        // Increment the integral part of the PID loop
        if (getThrottle() < PID_THROTTLE_THRESHOLD) {
            resetPid();
        } else if
        (
            getControlModeType() != acro ||
            (
                getDesiredYawAngle() < PID_ACRO_MAX_ANGLE_RATE_INTEGRAL_THRESHOLD &&
                getDesiredPitchAngle() < PID_ACRO_MAX_ANGLE_RATE_INTEGRAL_THRESHOLD &&
                getDesiredRollAngle() < PID_ACRO_MAX_ANGLE_RATE_INTEGRAL_THRESHOLD
            )
        ) {
            calculatePidIntegral(gyro_roll, gyro_pitch, gyro_yaw, delta_time_seconds);
        }

        // To debug stuff
        // print();
        // printConstants();
        // printThrottle(delta_time_seconds);
        // printGyro();

        // Run the motors with the calculated PID throttle
        runMotors(gyro_roll, gyro_pitch, gyro_yaw, delta_time_seconds);

        savePidErrors(gyro_roll, gyro_pitch, gyro_yaw);

        // persistPidConstants();
    }

    return true;
}

void KlevebrandMaxFlyDrone::setupMotors() {
    for (int i = 0; i < motor_pin_count; i++) {
        _motors[i].setup(_motor_pins[i]);
    }

    stopMotors();

    delay(1000);
}

void KlevebrandMaxFlyDrone::stopMotors() {
    motorLeftFront().setSpeed(0);
    motorRightFront().setSpeed(0);
    motorLeftBack().setSpeed(0);
    motorRightBack().setSpeed(0);
}

void KlevebrandMaxFlyDrone::runMotors(const float gyro_roll, const float gyro_pitch, const float gyro_yaw,
                                      const float delta_time_seconds) {
    const QuadcopterPidMotorThrottle_t throttle = pid.getPidMotorThrottle(
        getThrottle(), gyro_roll, getDesiredRollAngle(), gyro_pitch, getDesiredPitchAngle(), gyro_yaw,
        getDesiredYawAngle(), delta_time_seconds);

    motorLeftFront().setSpeed(throttle.pid_throttle_lf);
    motorRightFront().setSpeed(throttle.pid_throttle_rf);
    motorLeftBack().setSpeed(throttle.pid_throttle_lb);
    motorRightBack().setSpeed(throttle.pid_throttle_rb);
}

void KlevebrandMaxFlyDrone::attachMotors() const {
    motorLeftFront().attach();
    motorRightFront().attach();
    motorLeftBack().attach();
    motorRightBack().attach();
}

void KlevebrandMaxFlyDrone::detachMotors() const {
    motorLeftFront().detach();
    motorRightFront().detach();
    motorLeftBack().detach();
    motorRightBack().detach();
}

void KlevebrandMaxFlyDrone::enableMotors() {
    attachMotors();
    BaseDrone::enableMotors();
}

void KlevebrandMaxFlyDrone::disableMotors() {
    Serial.println("DISABLING MOTORS");
    setThrottle(0);
    stopMotors();
    resetPid();
    BaseDrone::disableMotors();
    detachMotors();
}

void KlevebrandMaxFlyDrone::printThrottle(const float delta_time_seconds) {
    const QuadcopterPidMotorThrottle_t throttle = pid.getPidMotorThrottle(
        getThrottle(), getRoll(), getDesiredRollAngle(), getPitch(), getDesiredPitchAngle(), getYaw(),
        getDesiredYawAngle(), delta_time_seconds);

    Serial.print(throttle.pid_throttle_lf);
    Serial.print("    ");
    Serial.println(throttle.pid_throttle_rf);
    Serial.print(throttle.pid_throttle_lb);
    Serial.print("    ");
    Serial.println(throttle.pid_throttle_rb);
    Serial.println("-----------------------------------------");
}
