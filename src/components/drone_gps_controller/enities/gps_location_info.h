#ifndef GPS_LOCATION_INFO_H
#define GPS_LOCATION_INFO_H

#include "Arduino.h"

struct GpsLocationInfo_t
{
    String timestamp_utc;
    float latitude;
    float longitude;
    float altitude;
    int fix;
    String course_over_ground;
    float speed_over_ground_kmh;
    int number_of_satellites;
    int number_of_satellites_glonass;

    static GpsLocationInfo_t parseFromGpsAcpString(String &gpsacp) 
    {
        GpsLocationInfo_t info;

        
    };
};

#endif // GPS_LOCATION_INFO_H

// $GPSACP: 161550.000,5950.9081N,01739.3691E,0.4,34.2,3,0.0,0.0,0.0,150126,10,06