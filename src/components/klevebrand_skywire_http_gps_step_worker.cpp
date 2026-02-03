#include "klevebrand_skywire_http_gps_step_worker.h"

DroneRequest_t SkywireHttpGpsStepWorker::latest_drone_request_response = DroneRequest_t(0, false, false, 0.0f, 0.0f);
String SkywireHttpGpsStepWorker::latest_gps_response = "";

static void printHext(String& data) {
    for (size_t i = 0; i < data.length(); i++) {
        if (i > 0 && i % 16 == 0) {
            Serial.println();  // new line every 16 bytes
        }
        uint8_t byte = (uint8_t)data[i];
        if (byte < 0x10) Serial.print("0");   // leading zero
        Serial.print(byte, HEX);
        Serial.print(" ");
    }
    Serial.println();  // final newline
}

void SkywireHttpGpsStepWorker::setLatestHttpResponse(String &response)
{
    Serial.print("bubbel");

    String response_copy = response;

    Serial.println("SETTING LATEST HTTP RESPONSE:");

    response_copy.replace("OK", "");
    response_copy.replace("<<<", "");
    response_copy.replace("ERROR", "");
    response_copy.replace("\r", "");
    response_copy.replace("\n", "");

    printHext(response_copy);

    DroneRequest_t test = DroneRequest_t(0, false, false, 0.0f, 0.0f);

    Serial.println(response_copy);

    memcpy(&test, response_copy.c_str(), sizeof(DroneRequest_t));

    Serial.println("PARSED DRONE REQUEST:");
    Serial.print("Flight Mode ID: " + String(test.flight_mode_id) + "\n");
    Serial.print("Enable Power: " + String(test.enable_power) + "\n");
    Serial.print("Enable Motors: " + String(test.enable_motors) + "\n");
    Serial.print("Longitude: " + String(test.longitude) + "\n");
    Serial.print("Latitude: " + String(test.latitude) + "\n");
}

void SkywireHttpGpsStepWorker::setLatestGpsResponse(String &response)
{
    latest_gps_response = response;

    latest_gps_response.replace("OK", "");
    latest_gps_response.replace("$GPSACP: ", "");
    latest_gps_response.replace("\r", "");
    latest_gps_response.replace("\n", "");
}

DroneRequest_t SkywireHttpGpsStepWorker::getLatestDroneRequest()
{
    return latest_drone_request_response;
}

String SkywireHttpGpsStepWorker::getLatestGpsResponse()
{
    return latest_gps_response;
}

GpsLocationInfo_t SkywireHttpGpsStepWorker::getLatestGpsLocationInfo()
{
    String latest_gps_response = getLatestGpsResponse();

    if (latest_gps_response == ",,,,,,,,,," || latest_gps_response.length() == 0)
    {
        return GpsLocationInfo_t::empty();
    }

    return GpsLocationInfo_t::parseFromGpsAcpString(latest_gps_response);
}