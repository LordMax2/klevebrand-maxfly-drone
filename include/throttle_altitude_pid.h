#ifndef THROTTLE_PID_H
#define THROTTLE_PID_H

#include <Arduino.h>
#include "pid_optimizer.h"

class ThrottleAltitudePid
{
public:
    ThrottleAltitudePid(float kp, float ki, float kd, float pid_max) : _pid_max(pid_max),
                                                                       _pid_optimizer(kp, ki, kd)
    {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    };

    void reset();

    void updateIntegral(float current_altitude, float desired_altitude);

    void printPid(float current_altitude, float desired_altitude);
    void printConstants();

    float getKp();
    float getKi();
    float getKd();

    void runOptimizer(float current_altitude, float desired_altitude);

    void saveError(float current_altitude, float desired_altitude);

    float throttlePid(float current_altitude, float desired_altitude);

private:
    float _pid_max;
    PidOptimizer _pid_optimizer;

    float altitudeError(float current_altitude, float desired_altitude);
    float _altitude_previous_error = 0;

    float altitudePidP(float current_altitude, float desired_altitude);
    float _throttle_pid_i = 0;
    float altitudePidD(float current_altitude, float desired_altitude);

    float _kp;
    float _ki;
    float _kd;
};

#endif // THROTTLE_PID_H