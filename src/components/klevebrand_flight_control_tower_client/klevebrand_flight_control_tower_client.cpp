#include "klevebrand_flight_control_tower_client.h"

void KlevebrandFlightControlTowerClient::setup()
{
    http_stepper_client.start();
}

DroneRequestDto KlevebrandFlightControlTowerClient::getDroneRequest(String serial_number)
{
    String path = "api/v1/dronerequest/" + serial_number;
    SkywireResponseResult_t result = http_stepper_client.get(path);

    if (result.is_success)
    {
        Serial.println("Received response from flight control tower:");
        Serial.println(result.response_content);

        DroneRequestDto drone_request = DroneRequestDto::parseFromCsvString(result.response_content);

        return drone_request;
    }

    return DroneRequestDto::getInvalidDroneRequest();
}