#include "drone_gps_controller.h"

void DroneGpsController::setup()
{
    skywire_http_gps_step_worker.start();

    bool recieved_start_location = false;

    while (!recieved_start_location)
    {
        skywire_http_gps_step_worker.run();

        String result = skywire_http_gps_step_worker.getLatestGpsResponse();

        if (result.length() > 0)
        {
            _start_location_info = GpsLocationInfo_t::parseFromGpsAcpString(result);

            recieved_start_location = true;
        }
    }

    Serial.println("DRONE GPS CONTROLLER STARTED");

    delay(1000);
}

void DroneGpsController::run()
{
    skywire_http_gps_step_worker.run();
}

void DroneGpsController::goTo(float latitude, float longitude, float altitude)
{
    skywire_http_gps_step_worker.run();

    String result = skywire_http_gps_step_worker.getLatestGpsResponse();

    if(result.length() == 0)
    {
        return;
    }

    GpsLocationInfo_t current_location_info = GpsLocationInfo_t::parseFromGpsAcpString(result);

    // Return early if no GPS fix
    if (current_location_info.fix < 2)
    {
        return;
    }

    // If the drone tilts morethan 40 degrees, dont run throttle PID controller
    if(abs(_drone.roll()) > 40.0f || abs(_drone.pitch()) > 40.0f)
    {
        return;
    }

    _altitude_pid.updateIntegral(current_location_info.altitude, altitude);
    _altitude_pid.runOptimizer(current_location_info.altitude, altitude);

    float throttle_adjustment = _altitude_pid.throttlePid(current_location_info.altitude, altitude);
    _drone.setThrottle(throttle_adjustment);

    // Return early if we are within 2 meters of the target altitude
    if(abs(altitude - current_location_info.altitude) > 2.0f) return;

    float yaw_destination_angle = getDestinationYawCompassAngle(latitude, longitude, current_location_info.latitude, current_location_info.longitude);

    _drone.setDesiredYawAngle(yaw_destination_angle);

    float yaw_error = yawError(_drone.yaw(), yaw_destination_angle);

    // Tilt towards target if the compass is less than 10 degrees off, otherwise level the drone again
    if (yaw_error < 10.0f)
    {
        _drone.setDesiredPitchAngle(10.0f);
    }
    else
    {
        _drone.setDesiredPitchAngle(0.0f);
    }
}

float DroneGpsController::getDestinationYawCompassAngle(float target_latitude, float target_longitude, float current_latitude, float current_longitude)
{
    return atan2f(target_latitude - current_latitude, target_longitude - current_longitude);
}

float DroneGpsController::yawError(float gyro_yaw, float yaw_desired_angle)
{
    auto error = fmod((gyro_yaw + 180), 360) - fmod((yaw_desired_angle + 180), 360);
    float absolute_error = min(abs(error), 360 - abs(error));

    if (error < 0)
    {
        return -1 * absolute_error;
    }

    return absolute_error;
}