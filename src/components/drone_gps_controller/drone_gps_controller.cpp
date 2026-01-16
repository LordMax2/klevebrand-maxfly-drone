#include "drone_gps_controller.h"

void DroneGpsController::setup()
{
    skywire_gps_client.start();

    bool recieved_start_location = false;

    while (!recieved_start_location)
    {
        SkywireResponseResult_t result = skywire_gps_client.get();

        if(result.is_success)
        {
            _start_location_info = GpsLocationInfo_t::parseFromGpsAcpString(result.response_content);

            recieved_start_location = true;
        }
    }

    Serial.println("DRONE GPS CONTROLLER STARTED");

    delay(1000);
}

void DroneGpsController::goTo(float latitude, float longitude, float altitude)
{
    SkywireResponseResult_t result = skywire_gps_client.get();

    if (!result.is_success)
    {
        return;
    }

    GpsLocationInfo_t current_location_info = GpsLocationInfo_t::parseFromGpsAcpString(result.response_content);

    if(current_location_info.fix < 2)
    {
        return;
    }

    float yaw_destination_angle = getDestinationYawCompassAngle(latitude, longitude, current_location_info.latitude, current_location_info.longitude);

    _drone.setDesiredYawAngle(yaw_destination_angle);

    float yaw_error = yawError(_drone.yaw(), yaw_destination_angle);

    if(yaw_error < 10.0f)
    {
        _drone.setDesiredPitchAngle(10.0f);
    }
    else
    {
        _drone.setDesiredPitchAngle(0.0f);
    }
}

float DroneGpsController::getDestinationYawCompassAngle(float target_latitude, float target_longitude, float current_latitude, float current_longitude) {
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