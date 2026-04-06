#include "klevebrand_maxfly_drone.h"

void KlevebrandMaxFlyDrone::setup()
{
    _processor.setup();

    _processor.print("STARTING DRONE...");

    _gyro.setup();

    pid_repository->setup();

    setupMotors();

    //setFlightModeAcro();
    setFlightModeAutoLevel();

    processor->print("DRONE STARTED!");
}

static long last_run_start_micros_timestamp = 0;
static long last_gyro_fetch_duration = 0;

bool KlevebrandMaxFlyDrone::run()
{
    if (delayToKeepFeedbackLoopHz(last_run_start_micros_timestamp - last_gyro_fetch_duration) > 0)
    {
        return false;
    }

    last_run_start_micros_timestamp = _processor.microsecondsTimestamp();

    // Get the latest data from the gyroscope
    long gyro_fetch_start_timestamp = _processor.microsecondsTimestamp();

    updateGyro();

    last_gyro_fetch_duration = _processor.microsecondsTimestamp() - gyro_fetch_start_timestamp;


    if (hasLostConnection())
    {
        // If connection is dead, stop the drone
        resetPid();
        stopMotors();

        // Serial.println("LOST CONNECTION");
    }
    else if (!isMotorsEnabled())
    {
        // If the motors are diabled, stop the drone
        resetPid();
        stopMotors();

        // Serial.println("MOTORS DISABLED");
    }
    else
    {
        // Increment the integral part of the PID loop
        if (getThrottle() > PID_THROTTLE_THRESHOLD)
        {
            calculatePidIntegral(_gyro.roll(), _gyro.pitch(), _gyro.yaw());
        }
        else
        {
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

void KlevebrandMaxFlyDrone::runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw)
{
    motorLeftFront().setSpeed(pid.pidThrottleLF(getThrottle(), gyro_roll, getDesiredRollAngle(), gyro_pitch, getDesiredPitchAngle(), gyro_yaw, getDesiredYawAngle()));
    motorRightFront().setSpeed(pid.pidThrottleRF(getThrottle(), gyro_roll, getDesiredRollAngle(), gyro_pitch, getDesiredPitchAngle(), gyro_yaw, getDesiredYawAngle()));
    motorLeftBack().setSpeed(pid.pidThrottleLB(getThrottle(), gyro_roll, getDesiredRollAngle(), gyro_pitch, getDesiredPitchAngle(), gyro_yaw, getDesiredYawAngle()));
    motorRightBack().setSpeed(pid.pidThrottleRB(getThrottle(), gyro_roll, getDesiredRollAngle(), gyro_pitch, getDesiredPitchAngle(), gyro_yaw, getDesiredYawAngle()));
}

void KlevebrandMaxFlyDrone::printThrottle()
{
    Serial.print(pid.pidThrottleLF(getThrottle(), _gyro.roll(), getDesiredRollAngle(), _gyro.pitch(), getDesiredPitchAngle(), _gyro.yaw(), getDesiredYawAngle()));
    Serial.print("    ");
    Serial.println(pid.pidThrottleRF(getThrottle(), _gyro.roll(), getDesiredRollAngle(), _gyro.pitch(), getDesiredPitchAngle(), _gyro.yaw(), getDesiredYawAngle()));
    Serial.print(pid.pidThrottleLB(getThrottle(), _gyro.roll(), getDesiredRollAngle(), _gyro.pitch(), getDesiredPitchAngle(), _gyro.yaw(), getDesiredYawAngle()));
    Serial.print("    ");
    Serial.println(pid.pidThrottleRB(getThrottle(), _gyro.roll(), getDesiredRollAngle(), _gyro.pitch(), getDesiredPitchAngle(), _gyro.yaw(), getDesiredYawAngle()));
    Serial.println("-----------------------------------------");
}
