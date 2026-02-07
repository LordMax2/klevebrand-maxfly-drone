#include "klevebrand_skywire_http_gps_step_worker.h"

DroneRequest_t SkywireHttpGpsStepWorker::latest_drone_request_response = DroneRequest_t::empty();
GpsLocationInfo_t SkywireHttpGpsStepWorker::latest_gps_response = GpsLocationInfo_t::empty();

void SkywireHttpGpsStepWorker::setLatestHttpResponse(String &response)
{
    String response_copy = response;

    response_copy.replace("OK", "");
    response_copy.replace("<<<", "");
    response_copy.replace("ERROR", "");
    response_copy.replace("+CME", "");
    response_copy.replace("\r", "");
    response_copy.replace("\n", "");

    latest_drone_request_response = DroneRequest_t::parseFromCsvString(response_copy);

    //Serial.println(latest_drone_request_response.toString());
}

DroneRequest_t SkywireHttpGpsStepWorker::getLatestDroneRequest()
{
    return latest_drone_request_response;
}

void SkywireHttpGpsStepWorker::setLatestGpsResponse(String &response)
{
    String response_copy = response;

    response_copy.replace("OK", "");
    response_copy.replace("$GPSACP: ", "");
    response_copy.replace("\r", "");
    response_copy.replace("\n", "");
    response_copy.replace(":", "");
    response_copy.replace(" ", "");
    response_copy.replace("wrong", "");
    response_copy.replace("state", "");

    latest_gps_response = GpsLocationInfo_t::parseFromGpsAcpString(response_copy);

    //Serial.println(latest_gps_response.toString());
}

GpsLocationInfo_t SkywireHttpGpsStepWorker::getLatestGpsResponse() {
    return latest_gps_response;
}