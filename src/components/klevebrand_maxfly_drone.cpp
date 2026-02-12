#include "klevebrand_maxfly_drone.h"

void KlevebrandMaxFlyDrone::setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    if (!Serial)
    {
        Serial.println("FAILED TO START SERIAL...");
    }
    while (!Serial)
    {
    }

    Serial.println("STARTING DRONE...");

    _gyro.setup();

    pid_repository->setup();

    setupMotors();

    setFlightModeAcro();

    Serial.println("DRONE STARTED!");
}

static long last_run_start_micros_timestamp = 0;

void KlevebrandMaxFlyDrone::run()
{
    if (delayToKeepFeedbackLoopHz(last_run_start_micros_timestamp) > 0)
    {
        return;
    }

    last_run_start_micros_timestamp = micros();

    // Get the latest data from the gyroscope
    updateGyro();

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
        if (throttle > PID_THROTTLE_THRESHOLD)
        {
            runPidOptimizer();
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
}

void KlevebrandMaxFlyDrone::runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw)
{
    motorLeftFront().setSpeed(pid.pidThrottleLF(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle));
    motorRightFront().setSpeed(pid.pidThrottleRF(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle));
    motorLeftBack().setSpeed(pid.pidThrottleLB(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle));
    motorRightBack().setSpeed(pid.pidThrottleRB(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle));
}

void KlevebrandMaxFlyDrone::printThrottle()
{
    Serial.print(pid.pidThrottleLF(throttle, _gyro.roll(), roll_desired_angle, _gyro.pitch(), pitch_desired_angle, _gyro.yaw(), yaw_desired_angle));
    Serial.print("    ");
    Serial.println(pid.pidThrottleRF(throttle, _gyro.roll(), roll_desired_angle, _gyro.pitch(), pitch_desired_angle, _gyro.yaw(), yaw_desired_angle));
    Serial.print(pid.pidThrottleLB(throttle, _gyro.roll(), roll_desired_angle, _gyro.pitch(), pitch_desired_angle, _gyro.yaw(), yaw_desired_angle));
    Serial.print("    ");
    Serial.println(pid.pidThrottleRB(throttle, _gyro.roll(), roll_desired_angle, _gyro.pitch(), pitch_desired_angle, _gyro.yaw(), yaw_desired_angle));
    Serial.println("-----------------------------------------");
}
