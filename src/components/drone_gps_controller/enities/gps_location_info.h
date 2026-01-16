#ifndef GPS_LOCATION_INFO_H
#define GPS_LOCATION_INFO_H

#include "Arduino.h"

struct GpsLocationInfo_t
{
    String timestamp_utc;
    float latitude;
    float longitude;
    float altitude;

    /* GPS Fix 
     * 0 = No Fix
     * 1 = No Fix
     * 2 = 2D Fix
     * 3 = 3D Fix
     */
    int fix;
    String course_over_ground;
    float speed_over_ground_kmh;
    int number_of_satellites;
    int number_of_satellites_glonass;

    static GpsLocationInfo_t parseFromGpsAcpString(String value) 
    {
        GpsLocationInfo_t info;

        String timestamp_utc = "";
        float latitude = 0.0f;
        float longitude = 0.0f;
        float altitude = 0.0f;
        int fix = 0;
        String course_over_ground = "";
        float speed_over_ground_kmh = 0.0f;
        int number_of_satellites = 0;
        int number_of_satellites_glonass = 0;


        const char *char_array_pointer = value.c_str();
        char *field_content = strtok((char *) char_array_pointer, ",");

        int field_index = 0;

        while(field_content != NULL && field_index < 11) {
            switch (field_index)
            {
                case 0:
                    timestamp_utc = field_content;
                    break;
                case 1:
                    latitude = atof(field_content);
                    break;
                case 2:
                    longitude = atof(field_content);
                    break;
                case 3:
                    
            }

            field_index++;

            field_content = strtok(NULL, ",");
        }
        
    };
};

#endif // GPS_LOCATION_INFO_H

// $GPSACP: 161550.000,5950.9081N,01739.3691E,0.4,34.2,3,0.0,0.0,0.0,150126,10,06