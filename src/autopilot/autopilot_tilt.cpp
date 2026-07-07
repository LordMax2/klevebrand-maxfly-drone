#include "autopilot/autopilot_tilt.h"
#include "klevebrand_maxfly_drone.h"
#include <math.h>
#include <template_drone.h>
#include "Arduino.h"

namespace
{
    unsigned long _last_goto_timestamp = 0;
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
void AutopilotTilt<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::goTo(TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType> *drone, const float latitude, const float longitude, const float altitude)
{
    // If the drone tilts morethan 40 degrees, dont run throttle PID controller
    if (abs(drone->getRoll()) > 40.0f || abs(drone->getPitch()) > 40.0f)
    {
        drone->setDesiredPitchAngle(0);
        drone->setDesiredRollAngle(0);

        return;
    }

    const auto current_altitude = drone->getAltitude();
    const auto current_longitude = drone->getLongitude();
    const auto current_latitude = drone->getLatitude();

    const long now = drone->timestampMicroseconds();
    const float delta_time_seconds = (now - _last_goto_timestamp) / 1000000.0f;
    _last_goto_timestamp = now;

    const float target_velocity = _altitude_pid.pid(current_altitude, altitude, delta_time_seconds);

    const float current_vz = drone->getVelocityZ();
    const float throttle_adjustment = _altitude_velocity_pid.pid(current_vz, target_velocity, delta_time_seconds);

    if (abs(throttle_adjustment) < 20.0f)
    {
        _altitude_pid.updateIntegral(current_altitude, altitude, delta_time_seconds);
    }

    constexpr float hover_learn_altitude_gate = 5.0f;

    if (abs(altitude - current_altitude) < hover_learn_altitude_gate)
    {
        constexpr float hover_learn_rate = 0.5f;
        _hover_throttle = constrain(_hover_throttle - hover_learn_rate * current_vz * delta_time_seconds, 0.0f, 100.0f);
    }

    const float final_throttle = constrain(_hover_throttle + throttle_adjustment, 0.0f, 100.0f);

    drone->setThrottle(final_throttle);

    // Return early if we are not within 20 meters of the target altitude
    if (abs(altitude - current_altitude) > 20.0f || true) // Return early for now to test altitude hover!
    {
        drone->setDesiredPitchAngle(0);
        drone->setDesiredRollAngle(0);

        return;
    }

    constexpr float METERS_PER_DEGREE = 111320.0f;

    const float latitude_radians = current_latitude * (PI / 180.0f);
    const float cosinus_latitude = cosf(latitude_radians);

    const float north_error_meters = (latitude - current_latitude) * METERS_PER_DEGREE;
    const float east_error_meters = (longitude - current_longitude) * cosinus_latitude * METERS_PER_DEGREE;

    const float target_north_velocity = _latitude_pid.pid(0.0f, north_error_meters, delta_time_seconds);
    const float target_east_velocity = _longitude_pid.pid(0.0f, east_error_meters, delta_time_seconds);

    const float current_north_velocity = drone->getVelocityX();
    const float current_east_velocity = drone->getVelocityY();

    const float pitch_adjustment = _latitude_velocity_pid.pid(current_north_velocity, target_north_velocity,
                                                              delta_time_seconds);
    const float roll_adjustment = _longitude_velocity_pid.
        pid(current_east_velocity, target_east_velocity, delta_time_seconds);

    if (abs(pitch_adjustment) < 10.0f)
    {
        _latitude_pid.updateIntegral(0.0f, north_error_meters, delta_time_seconds);
    }
    if (abs(roll_adjustment) < 10.0f)
    {
        _longitude_pid.updateIntegral(0.0f, east_error_meters, delta_time_seconds);
    }

    _latitude_pid.saveError(0.0f, north_error_meters);
    _longitude_pid.saveError(0.0f, east_error_meters);
    _latitude_velocity_pid.saveError(current_north_velocity, target_north_velocity);
    _longitude_velocity_pid.saveError(current_east_velocity, target_east_velocity);

    drone->setDesiredYawAngle(0.0f);
    drone->setDesiredPitchAngle(pitch_adjustment * -1);
    drone->setDesiredRollAngle(roll_adjustment * -1);
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
float AutopilotTilt<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::getDestinationYawCompassAngle(const float target_latitude, const float target_longitude,
                                                   const float current_latitude, const float current_longitude)
{
    const float delta_latitude = target_latitude - current_latitude;
    const float delta_longitude = target_longitude - current_longitude;

    float bearing_degrees = atan2f(delta_longitude, delta_latitude) * (180.0f / PI);

    if (bearing_degrees < 0.0f)
    {
        bearing_degrees += 360.0f;
    }

    return bearing_degrees;
}

template class AutopilotTilt<QuadcopterPid, QuadcopterPosition<Bno08xDroneGyro>, Bno08xDroneGyro, HardwareProcessorArduino>;
