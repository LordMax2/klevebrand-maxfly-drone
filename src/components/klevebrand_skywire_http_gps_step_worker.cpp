#include "klevebrand_skywire_http_gps_step_worker.h"

String SkywireHttpGpsStepWorker::latest_http_response = "";
String SkywireHttpGpsStepWorker::latest_gps_response = "";

void SkywireHttpGpsStepWorker::setLatestHttpResponse(String &response)
{
    latest_http_response = response;

    latest_http_response.replace("OK", "");
    latest_http_response.replace("<<<", "");
    latest_http_response.replace("\r", "");
    latest_http_response.replace("\n", "");
    latest_http_response.replace("ERROR", "");
}

void SkywireHttpGpsStepWorker::setLatestGpsResponse(String &response)
{
    latest_gps_response = response;

    latest_gps_response.replace("OK", "");
    latest_gps_response.replace("$GPSACP: ", "");
    latest_gps_response.replace("\r", "");
    latest_gps_response.replace("\n", "");
}

String SkywireHttpGpsStepWorker::getLatestHttpResponse()
{
    return latest_http_response;
}

String SkywireHttpGpsStepWorker::getLatestGpsResponse()
{
    return latest_gps_response;
}

DroneRequest_t SkywireHttpGpsStepWorker::getLatestDroneRequest()
{
    String latest_http_response = getLatestHttpResponse();

    if (latest_http_response.length() == 0)
    {
        return DroneRequest_t::getInvalidDroneRequest();
    }

    return DroneRequest_t::parseFromCsvString(latest_http_response);
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