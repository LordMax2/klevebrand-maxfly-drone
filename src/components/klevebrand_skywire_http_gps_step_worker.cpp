#include "klevebrand_skywire_http_gps_step_worker.h"

DroneRequest_t SkywireHttpGpsStepWorker::latest_drone_request_response = DroneRequest_t(0, false, false, 0.0f, 0.0f);
String SkywireHttpGpsStepWorker::latest_gps_response = "";

void SkywireHttpGpsStepWorker::setLatestHttpResponse(String &response)
{
    String response_copy = response;

    response_copy.replace("OK", "");
    response_copy.replace("<<<", "");
    response_copy.replace("ERROR", "");
    response_copy.replace("+CME", "");
    response_copy.replace("\r", "");
    response_copy.replace("\n", "");

    Serial.println(response_copy);
}

void SkywireHttpGpsStepWorker::setLatestGpsResponse(String &response)
{
    latest_gps_response = response;

    latest_gps_response.replace("OK", "");
    latest_gps_response.replace("$GPSACP: ", "");
    latest_gps_response.replace("\r", "");
    latest_gps_response.replace("\n", "");
    latest_gps_response.replace(":", "");
    latest_gps_response.replace(" ", "");
    latest_gps_response.replace("wrong", "");
    latest_gps_response.replace("state", "");

    Serial.println(latest_gps_response);
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