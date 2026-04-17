#include "klevebrand_maxfly_drone.h"

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
static unsigned long last_pid_optimizer_run = 0;

bool KlevebrandMaxFlyDrone::run() {
    if (delayToKeepFeedbackLoopHz(last_run_start_micros_timestamp - last_gyro_fetch_duration) > 0) {
        return false;
    }

    const unsigned long now_micros = _processor.microsecondsTimestamp();

    last_run_start_micros_timestamp = now_micros;

    // Get the latest data from the gyroscope
    unsigned long gyro_fetch_start_timestamp = now_micros;

    updateGyro();

    last_gyro_fetch_duration = _processor.microsecondsTimestamp() - gyro_fetch_start_timestamp;

    const unsigned long now_millis = _processor.millisecondsTimestamp();

    if (hasLostConnection()) {
        // If connection is dead, stop the drone
        disableMotors();

        // Serial.println("LOST CONNECTION");
    } else if (!isMotorsEnabled()) {
        // If the motors are diabled, stop the drone
        disableMotors();

        // Serial.println("MOTORS DISABLED");
    } else {
        // Increment the integral part of the PID loop
        if (getThrottle() > PID_THROTTLE_THRESHOLD) {
            calculatePidIntegral(_gyro.roll(), _gyro.pitch(), _gyro.yaw());

            if (true) {
                runPidOptimizer(now_millis);
            }
        } else {
            resetPid();
        }

        // To debug stuff
        // print();
        // printConstants();
        // printThrottle();
        // printGyro();

        // Run the motors with the calculated PID throttle
        runMotors(_gyro.roll(), _gyro.pitch(), _gyro.yaw());

        savePidErrors(_gyro.roll(), _gyro.pitch(), _gyro.yaw());

        persistPidConstants();
    }

    return true;
}

void KlevebrandMaxFlyDrone::runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw) {
    motorLeftFront().setSpeed(pid.pidThrottleLF(getThrottle(), gyro_roll, getDesiredRollAngle(), gyro_pitch,
                                                getDesiredPitchAngle(), gyro_yaw, getDesiredYawAngle()));
    motorRightFront().setSpeed(pid.pidThrottleRF(getThrottle(), gyro_roll, getDesiredRollAngle(), gyro_pitch,
                                                 getDesiredPitchAngle(), gyro_yaw, getDesiredYawAngle()));
    motorLeftBack().setSpeed(pid.pidThrottleLB(getThrottle(), gyro_roll, getDesiredRollAngle(), gyro_pitch,
                                               getDesiredPitchAngle(), gyro_yaw, getDesiredYawAngle()));
    motorRightBack().setSpeed(pid.pidThrottleRB(getThrottle(), gyro_roll, getDesiredRollAngle(), gyro_pitch,
                                                getDesiredPitchAngle(), gyro_yaw, getDesiredYawAngle()));
}

void KlevebrandMaxFlyDrone::attachMotors() {
    motorLeftFront().attach();
    motorRightFront().attach();
    motorLeftBack().attach();
    motorRightBack().attach();
}

void KlevebrandMaxFlyDrone::detachMotors() {
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

void KlevebrandMaxFlyDrone::printThrottle() {
    Serial.print(pid.pidThrottleLF(getThrottle(), _gyro.roll(), getDesiredRollAngle(), _gyro.pitch(),
                                   getDesiredPitchAngle(), _gyro.yaw(), getDesiredYawAngle()));
    Serial.print("    ");
    Serial.println(pid.pidThrottleRF(getThrottle(), _gyro.roll(), getDesiredRollAngle(), _gyro.pitch(),
                                     getDesiredPitchAngle(), _gyro.yaw(), getDesiredYawAngle()));
    Serial.print(pid.pidThrottleLB(getThrottle(), _gyro.roll(), getDesiredRollAngle(), _gyro.pitch(),
                                   getDesiredPitchAngle(), _gyro.yaw(), getDesiredYawAngle()));
    Serial.print("    ");
    Serial.println(pid.pidThrottleRB(getThrottle(), _gyro.roll(), getDesiredRollAngle(), _gyro.pitch(),
                                     getDesiredPitchAngle(), _gyro.yaw(), getDesiredYawAngle()));
    Serial.println("-----------------------------------------");
}
