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
    
    if(!result.is_success)
    {
        return;
    }

    GpsLocationInfo_t current_location_info = GpsLocationInfo_t::parseFromGpsAcpString(result.response_content);

    float yaw_destination_angle = atan2f(latitude - current_location_info.latitude, longitude - current_location_info.longitude);

    _drone.setDesiredYawAngle(yaw_destination_angle);
}
