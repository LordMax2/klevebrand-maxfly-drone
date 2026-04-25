#include "klevebrand_maxfly_drone.h"

KlevebrandMaxFlyDrone::KlevebrandMaxFlyDrone(ServoDroneMotor *motors, const int motor_pins[motor_pin_count])
    : TemplateDrone<QuadcopterPid>(500, 200, 10000, &_processor, &_gyro, &_pid_repository, &_position), _motors(motors), _gyro(10) {
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

    _gyro.setup();

    pid_repository->setup();

    setupMotors();

    //setFlightModeAcro();
    setFlightModeAutoLevel();

    processor->print("DRONE STARTED!");
}

static unsigned long last_run_start_micros_timestamp = 0;
static unsigned long last_gyro_fetch_duration = 0;

bool KlevebrandMaxFlyDrone::run() {
    if (delayToKeepFeedbackLoopHz(last_run_start_micros_timestamp - last_gyro_fetch_duration) > 0) {
        return false;
    }

    const unsigned long current_time = _processor.microsecondsTimestamp();
    const unsigned long delta_time = current_time - last_run_start_micros_timestamp;
    const unsigned long delta_time_seconds = delta_time / 1000000.0f;
    last_run_start_micros_timestamp = current_time;

    // Get the latest data from the gyroscope
    const unsigned long gyro_fetch_start_timestamp = _processor.microsecondsTimestamp();

    updateGyro();

    last_gyro_fetch_duration = _processor.microsecondsTimestamp() - gyro_fetch_start_timestamp;

    const unsigned long now_millis = _processor.millisecondsTimestamp();

    if (hasLostConnection()) {
        // If connection is dead, stop the drone
        disableMotors();

        // Serial.println("LOST CONNECTION");
    } else if (!isMotorsEnabled() && false) {
        // If the motors are diabled, stop the drone
        disableMotors();

        // Serial.println("MOTORS DISABLED");
    } else {
        // Increment the integral part of the PID loop
        if (getThrottle() > PID_THROTTLE_THRESHOLD) {
            calculatePidIntegral(_gyro.roll(), _gyro.pitch(), _gyro.yaw(), delta_time_seconds);
        } else {
            resetPid();
        }

        // To debug stuff
        // print();
        // printConstants();
        // printThrottle(delta_time_seconds);
        // printGyro();

        // Run the motors with the calculated PID throttle
        runMotors(_gyro.roll(), _gyro.pitch(), _gyro.yaw(), delta_time_seconds);

        savePidErrors(_gyro.roll(), _gyro.pitch(), _gyro.yaw());

        persistPidConstants();
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

void KlevebrandMaxFlyDrone::runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw, float delta_time_seconds) {
    const float pid_throttle_lf = pid.pidThrottleLF(getThrottle(), gyro_roll, getDesiredRollAngle(), gyro_pitch, getDesiredPitchAngle(), gyro_yaw, getDesiredYawAngle(), delta_time_seconds);
    const float pid_throttle_rf = pid.pidThrottleRF(getThrottle(), gyro_roll, getDesiredRollAngle(), gyro_pitch, getDesiredPitchAngle(), gyro_yaw, getDesiredYawAngle(), delta_time_seconds);
    const float pid_throttle_lb = pid.pidThrottleLB(getThrottle(), gyro_roll, getDesiredRollAngle(), gyro_pitch, getDesiredPitchAngle(), gyro_yaw, getDesiredYawAngle(), delta_time_seconds);
    const float pid_throttle_rb = pid.pidThrottleRB(getThrottle(), gyro_roll, getDesiredRollAngle(), gyro_pitch, getDesiredPitchAngle(), gyro_yaw, getDesiredYawAngle(), delta_time_seconds);

    const float max_motor_pid_throttle = max(max(pid_throttle_lf, pid_throttle_rf), max(pid_throttle_lb, pid_throttle_rb));

    float throttle_reduction = 0.0f;
    if (max_motor_pid_throttle > 100.0f) {
        throttle_reduction = max_motor_pid_throttle - 100.0f;
    }

    motorLeftFront().setSpeed(max(0.0f, pid_throttle_lf - throttle_reduction));
    motorRightFront().setSpeed(max(0.0f, pid_throttle_rf - throttle_reduction));
    motorLeftBack().setSpeed(max(0.0f, pid_throttle_lb - throttle_reduction));
    motorRightBack().setSpeed(max(0.0f, pid_throttle_rb - throttle_reduction));
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
    stopMotors();
    BaseDrone::enableMotors();
}

void KlevebrandMaxFlyDrone::disableMotors() {
    setThrottle(0);
    stopMotors();
    resetPid();
    BaseDrone::disableMotors();
    detachMotors();
}

void KlevebrandMaxFlyDrone::printThrottle(float delta_time_seconds) {
    Serial.print(pid.pidThrottleLF(getThrottle(), _gyro.roll(), getDesiredRollAngle(), _gyro.pitch(),
                                   getDesiredPitchAngle(), _gyro.yaw(), getDesiredYawAngle(), delta_time_seconds));
    Serial.print("    ");
    Serial.println(pid.pidThrottleRF(getThrottle(), _gyro.roll(), getDesiredRollAngle(), _gyro.pitch(),
                                     getDesiredPitchAngle(), _gyro.yaw(), getDesiredYawAngle(), delta_time_seconds));
    Serial.print(pid.pidThrottleLB(getThrottle(), _gyro.roll(), getDesiredRollAngle(), _gyro.pitch(),
                                   getDesiredPitchAngle(), _gyro.yaw(), getDesiredYawAngle(), delta_time_seconds));
    Serial.print("    ");
    Serial.println(pid.pidThrottleRB(getThrottle(), _gyro.roll(), getDesiredRollAngle(), _gyro.pitch(),
                                     getDesiredPitchAngle(), _gyro.yaw(), getDesiredYawAngle(), delta_time_seconds));
    Serial.println("-----------------------------------------");
}
