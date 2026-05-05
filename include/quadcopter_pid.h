#ifndef QUADCOPTER_PID_H 
#define QUADCOPTER_PID_H

#include <Arduino.h>
#include "pid_optimizer.h"
#include "gyro_pid.h"

#define PID_THROTTLE_THRESHOLD 10 
#define PID_ACRO_MAX_ANGLE_RATE_INTEGRAL_THRESHOLD 30
#define PID_MAX 15

#define THROTTLE_MINIMUM 0 
#define THROTTLE_MAXIMUM 100

struct QuadcopterPidMotorThrottle_t
{
    QuadcopterPidMotorThrottle_t(const float pid_throttle_lf, const float pid_throttle_lb, const float pid_throttle_rf, const float pid_throttle_rb)
    {
        this->pid_throttle_lf = pid_throttle_lf;
        this->pid_throttle_lb = pid_throttle_lb;
        this->pid_throttle_rf = pid_throttle_rf;
        this->pid_throttle_rb = pid_throttle_rb;
    }

    float pid_throttle_lf;
    float pid_throttle_lb;
    float pid_throttle_rf;
    float pid_throttle_rb;
};

class QuadcopterPid : public GyroPid
{
public:
    QuadcopterPid(float yaw_kp, float yaw_ki, float yaw_kd, bool yaw_compass_mode, float pitch_kp, float pitch_ki, float pitch_kd, float roll_kp, float roll_ki, float roll_kd, float, const int feedback_loop_hz) : GyroPid(yaw_kp, yaw_ki, yaw_kd, yaw_compass_mode, pitch_kp, pitch_ki, pitch_kd, roll_kp, roll_ki, roll_kd, PID_MAX, feedback_loop_hz) {};

    QuadcopterPidMotorThrottle_t getPidMotorThrottle(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, float delta_time_seconds);

private:
    float pidThrottleLF(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, float delta_time_seconds);
    float pidThrottleLB(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, float delta_time_seconds);
    float pidThrottleRF(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, float delta_time_seconds);
    float pidThrottleRB(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, float delta_time_seconds);


};

#endif // QUADCOPTER_PID_H
