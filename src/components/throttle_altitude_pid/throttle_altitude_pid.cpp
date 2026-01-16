#include "throttle_altitude_pid.h"

void ThrottleAltitudePid::reset()
{
    _pid_optimizer = PidOptimizer(_kp, _ki, _kd);
}

void ThrottleAltitudePid::printConstants()
{
    Serial.print("Throttle Altitude PID Constants - Kp: ");
    Serial.print(_kp);
    Serial.print(", Ki: ");
    Serial.print(_ki);
    Serial.print(", Kd: ");
    Serial.println(_kd);
}

float ThrottleAltitudePid::getKp()
{
    return _pid_optimizer.getKp();
}

float ThrottleAltitudePid::getKi()
{
    return _pid_optimizer.getKi();
}

float ThrottleAltitudePid::getKd()
{
    return _pid_optimizer.getKd();
}

void ThrottleAltitudePid::runOptimizer(float current_altitude, float desired_altitude)
{
    float current_error = altitudeError(current_altitude, desired_altitude);

    _pid_optimizer.run(current_error);
}

float ThrottleAltitudePid::altitudeError(float current_altitude, float desired_altitude)
{
    return desired_altitude - current_altitude;
}

void ThrottleAltitudePid::saveError(float current_altitude, float desired_altitude)
{
    _altitude_previous_error = altitudeError(current_altitude, desired_altitude);
}

float ThrottleAltitudePid::altitudePidP(float current_altitude, float desired_altitude)
{
    return _pid_optimizer.getKp() * altitudeError(current_altitude, desired_altitude);
}

float ThrottleAltitudePid::altitudePidD(float current_altitude, float desired_altitude)
{
    return _pid_optimizer.getKd() * (altitudeError(current_altitude, desired_altitude) - _altitude_previous_error);
}

float ThrottleAltitudePid::throttlePid(float current_altitude, float desired_altitude) 
{
    return constrain(altitudePidP(current_altitude, desired_altitude) + _throttle_pid_i + altitudePidD(current_altitude, desired_altitude), -_pid_max, _pid_max);
}

void ThrottleAltitudePid::printPid(float current_altitude, float desired_altitude) {
    Serial.print("Throttle Altitude PID: ");
    Serial.println(throttlePid(current_altitude, desired_altitude));
}

void ThrottleAltitudePid::updateIntegral(float current_altitude, float desired_altitude) 
{
    _throttle_pid_i = constrain(_throttle_pid_i+ (getKi() * altitudeError(current_altitude, desired_altitude)), -_pid_max, _pid_max);
}