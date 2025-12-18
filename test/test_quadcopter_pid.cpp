#include <unity.h>
#include "../src/components/quadcopter_pid/quadcopter_pid.h"

void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}

/// @brief
/// Test the PID throttle output when PID constants are zero.
/// The throttle output from the PID should be the same as the throttle input when the constants are zero.
void test_zero_pid_throttle()
{
    QuadcopterPid pid(0, 0, 0, 0, 0, 0, 0 ,0, 0);

    float throttle = 1500;

    float gyroRoll = 0;
    float gyroPitch = 0;
    float gyroYaw = 0;

    float desiredRoll = 0;
    float desiredPitch = 0;
    float desiredYaw = 0;

    float throttleLf = pid.pidThrottleLF(throttle, gyroRoll, desiredRoll, gyroPitch, desiredPitch, gyroYaw, desiredYaw, true);
    float throttleRf = pid.pidThrottleRF(throttle, gyroRoll, desiredRoll, gyroPitch, desiredPitch, gyroYaw, desiredYaw, true);
    float throttleLb = pid.pidThrottleLB(throttle, gyroRoll, desiredRoll, gyroPitch, desiredPitch, gyroYaw, desiredYaw, true);
    float throttleRb = pid.pidThrottleRB(throttle, gyroRoll, desiredRoll, gyroPitch, desiredPitch, gyroYaw, desiredYaw, true);

    TEST_ASSERT_EQUAL(throttle, throttleLf);
    TEST_ASSERT_EQUAL(throttle, throttleRf);
    TEST_ASSERT_EQUAL(throttle, throttleLb);
    TEST_ASSERT_EQUAL(throttle, throttleRb);
}

/// @brief
/// Tests the PID throttle in a couple of loops to make sure P, I and D respond as predicted for pitch and roll.
void test_pid_throttle_response_pitch_roll()
{
    QuadcopterPid pid(1.25, 0.01, 8, 1.25, 0.01, 8, 1.25, 0.01, 8);

    float throttle = 1500;

    float gyroRoll = 45;
    float gyroPitch = -0.9;
    float gyroYaw = 0;

    float gyroPreviousRoll = 42;
    float gyroPreviousPitch = -0.1;
    float gyroPreviousYaw = -0.1;

    float desiredRoll = 0;
    float desiredPitch = 0;
    float desiredYaw = 0;

    pid.saveRollError(gyroPreviousRoll, desiredRoll);
    pid.savePitchError(gyroPreviousPitch, desiredPitch);
    pid.saveYawError(gyroPreviousYaw, desiredYaw, true);

    // Run the integral update 3 times to get a little bit bigger values, in reality, this equals three feedback loop-runs.
    pid.updateIntegral(gyroRoll, desiredRoll, gyroPitch, desiredPitch, gyroYaw, desiredYaw, true);
    pid.updateIntegral(gyroRoll, desiredRoll, gyroPitch, desiredPitch, gyroYaw, desiredYaw, true);
    pid.updateIntegral(gyroRoll, desiredRoll, gyroPitch, desiredPitch, gyroYaw, desiredYaw, true);

    float throttleLf = pid.pidThrottleLF(throttle, gyroRoll, desiredRoll, gyroPitch, desiredPitch, gyroYaw, desiredYaw, true);
    float throttleRf = pid.pidThrottleRF(throttle, gyroRoll, desiredRoll, gyroPitch, desiredPitch, gyroYaw, desiredYaw, true);
    float throttleLb = pid.pidThrottleLB(throttle, gyroRoll, desiredRoll, gyroPitch, desiredPitch, gyroYaw, desiredYaw, true);
    float throttleRb = pid.pidThrottleRB(throttle, gyroRoll, desiredRoll, gyroPitch, desiredPitch, gyroYaw, desiredYaw, true);

    TEST_ASSERT_EQUAL(1589, throttleLf);
    TEST_ASSERT_EQUAL(1425, throttleRf);
    TEST_ASSERT_EQUAL(1574, throttleLb);
    TEST_ASSERT_EQUAL(1410, throttleRb);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_zero_pid_throttle);
    RUN_TEST(test_pid_throttle_response_pitch_roll);

    UNITY_END();
}