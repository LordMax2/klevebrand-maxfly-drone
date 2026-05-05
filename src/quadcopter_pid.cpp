#include "quadcopter_pid.h"

QuadcopterPidMotorThrottle_t QuadcopterPid::getPidMotorThrottle(const float throttle, const float gyro_roll, const float roll_desired_angle, const float gyro_pitch, const float pitch_desired_angle, const float gyro_yaw, const float yaw_desired_angle, const float delta_time_seconds)
{
    const float pid_throttle_lf = pidThrottleLF(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle, delta_time_seconds);
    const float pid_throttle_rf = pidThrottleRF(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle, delta_time_seconds);
    const float pid_throttle_lb = pidThrottleLB(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle, delta_time_seconds);
    const float pid_throttle_rb = pidThrottleRB(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle, delta_time_seconds);

    const float max_motor_pid_throttle = max(max(pid_throttle_lf, pid_throttle_rf), max(pid_throttle_lb, pid_throttle_rb));

    float throttle_reduction = 0.0f;
    if (max_motor_pid_throttle > 100.0f)
    {
        throttle_reduction = max_motor_pid_throttle - 100.0f;
    }

    return {
      max(0.0f, pid_throttle_lf - throttle_reduction),
      max(0.0f, pid_throttle_lb - throttle_reduction),
      max(0.0f, pid_throttle_rf - throttle_reduction),
      max(0.0f, pid_throttle_rb - throttle_reduction),
    };
}

float QuadcopterPid::pidThrottleLF(const float throttle, const float gyro_roll, const float roll_desired_angle, const float gyro_pitch, const float pitch_desired_angle, const float gyro_yaw, const float yaw_desired_angle, const float delta_time_seconds)
{
  if (throttle < PID_THROTTLE_THRESHOLD)
    return THROTTLE_MINIMUM;

  return throttle - rollPid(gyro_roll, roll_desired_angle, delta_time_seconds) + pitchPid(gyro_pitch, pitch_desired_angle, delta_time_seconds) + yawPid(gyro_yaw, yaw_desired_angle, delta_time_seconds);
}

float QuadcopterPid::pidThrottleLB(const float throttle, const float gyro_roll, const float roll_desired_angle, const float gyro_pitch, const float pitch_desired_angle, const float gyro_yaw, const float yaw_desired_angle, const float delta_time_seconds)
{
  if (throttle < PID_THROTTLE_THRESHOLD)
    return THROTTLE_MINIMUM;

  return throttle - rollPid(gyro_roll, roll_desired_angle, delta_time_seconds) - pitchPid(gyro_pitch, pitch_desired_angle, delta_time_seconds) - yawPid(gyro_yaw, yaw_desired_angle, delta_time_seconds);
}

float QuadcopterPid::pidThrottleRF(const float throttle, const float gyro_roll, const float roll_desired_angle, const float gyro_pitch, const float pitch_desired_angle, const float gyro_yaw, const float yaw_desired_angle, const float delta_time_seconds)
{
  if (throttle < PID_THROTTLE_THRESHOLD)
    return THROTTLE_MINIMUM;

  return throttle + rollPid(gyro_roll, roll_desired_angle, delta_time_seconds) + pitchPid(gyro_pitch, pitch_desired_angle, delta_time_seconds) - yawPid(gyro_yaw, yaw_desired_angle, delta_time_seconds);
}

float QuadcopterPid::pidThrottleRB(const float throttle, const float gyro_roll, const float roll_desired_angle, const float gyro_pitch, const float pitch_desired_angle, const float gyro_yaw, const float yaw_desired_angle, const float delta_time_seconds)
{
  if (throttle < PID_THROTTLE_THRESHOLD)
    return THROTTLE_MINIMUM;

  return throttle + rollPid(gyro_roll, roll_desired_angle, delta_time_seconds) - pitchPid(gyro_pitch, pitch_desired_angle, delta_time_seconds) + yawPid(gyro_yaw, yaw_desired_angle, delta_time_seconds);
}