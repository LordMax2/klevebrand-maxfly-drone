#include "klevebrand_flight_control_tower_client.h"
#include "ArduinoJson.h"

void KlevebrandFlightControlTowerClient::setup()
{
    http_stepper_client.start();
}

DroneRequestDto KlevebrandFlightControlTowerClient::getDroneRequest(String serial_number)
{
    String path = "api/v1/dronerequest/" + serial_number;
    SkywireResponseResult_t result = http_stepper_client.get(path);

    int response_json_start = result.response_content.indexOf('{');
    int response_json_end = result.response_content.lastIndexOf('}');

    if (result.is_success && response_json_start != -1 && response_json_end != -1)
    {
        String response_json = result.response_content.substring(response_json_start, response_json_end + 1);

        JsonDocument doc;
        deserializeJson(doc, response_json);

        Serial.println("Drone request JSON, json start: " + String(response_json_start) + ", json end: " + String(response_json_end));
        Serial.println(response_json);

        int flight_mode_id = doc["flight_mode_id"] | 0;
        float throttle = doc["throttle"] | 0.0f;
        float yaw = doc["yaw"] | 0.0f;
        float pitch = doc["pitch"] | 0.0f;
        float roll = doc["roll"] | 0.0f;
        bool enable_power = doc["enable_power"] | false;
        bool enable_motors = doc["enable_motors"] | false;

        return DroneRequestDto(
            flight_mode_id,
            throttle,
            yaw,
            pitch,
            roll,
            enable_power,
            enable_motors,
            true);
    }

    return DroneRequestDto(
        0,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        false,
        false,
        false
    );
}