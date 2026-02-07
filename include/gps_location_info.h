#ifndef GPS_LOCATION_INFO_H
#define GPS_LOCATION_INFO_H

#include "Arduino.h"

struct GpsLocationInfo_t
{
    GpsLocationInfo_t(
        String timestamp,
        String date,
        float latitude,
        float longitude,
        float altitude,
        int fix,
        String course_over_ground,
        float speed_over_ground_kmh,
        float speed_over_ground_knots,
        int number_of_satellites,
        int number_of_satellites_glonass)
    {
        this->timestamp_utc = timestamp;
        this->date = date;
        this->latitude = latitude;
        this->longitude = longitude;
        this->altitude = altitude;
        this->fix = fix;
        this->course_over_ground = course_over_ground;
        this->speed_over_ground_kmh = speed_over_ground_kmh;
        this->speed_over_ground_knots = speed_over_ground_knots;
        this->number_of_satellites = number_of_satellites;
        this->number_of_satellites_glonass = number_of_satellites_glonass;
    }

    String timestamp_utc;
    String date;
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
    float speed_over_ground_knots;
    int number_of_satellites;
    int number_of_satellites_glonass;

    String toString()
    {
        return "Timestamp: " + timestamp_utc + ", Date: " + date + ", Latitude: " + String(latitude) + ", Longitude: " + String(longitude) + ", Altitude: " + String(altitude) + ", Fix: " + String(fix) + ", CourseOverGround: " + course_over_ground + ", SpeedOverGroundKmh: " + String(speed_over_ground_kmh) + ", SpeedOverGroundKnots: " + String(speed_over_ground_knots) + ", NumberOfSatellites: " + String(number_of_satellites) + ", NumberOfSatellitesGlonass: " + String(number_of_satellites_glonass);
    }

    static GpsLocationInfo_t empty()
    {
        return GpsLocationInfo_t("", "", 0.0f, 0.0f, 0.0f, 0, "", 0.0f, 0.0f, 0, 0);
    }

    static GpsLocationInfo_t parseFromGpsAcpString(String value)
    {
        String timestamp_utc = "";
        String date = "";
        float latitude = 0.0f;
        float longitude = 0.0f;
        float altitude = 0.0f;
        int fix = 0;
        String course_over_ground = "";
        float speed_over_ground_kmh = 0.0f;
        float speed_over_ground_knots = 0.0f;
        int number_of_satellites = 0;
        int number_of_satellites_glonass = 0;

        const char *char_array_pointer = value.c_str();
        char *field_content = strtok((char *)char_array_pointer, ",");

        int field_index = 0;

        while (field_content != NULL && field_index < 11)
        {
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
            case 4:
                altitude = atof(field_content);
                break;
            case 5:
                fix = atoi(field_content);
                break;
            case 6:
                course_over_ground = field_content;
                break;
            case 7:
                speed_over_ground_knots = atof(field_content);
            case 8:
                speed_over_ground_kmh = atof(field_content);
                break;
            case 9:
                date = field_content;
                break;
            case 10:
                number_of_satellites = atoi(field_content);
                break;
            case 11:
                number_of_satellites_glonass = atoi(field_content);
                break;
            default:
                break;
            }

            field_index++;

            field_content = strtok(NULL, ",");
        }

        return GpsLocationInfo_t{
            timestamp_utc,
            date,
            latitude,
            longitude,
            altitude,
            fix,
            course_over_ground,
            speed_over_ground_kmh,
            speed_over_ground_knots,
            number_of_satellites,
            number_of_satellites_glonass};
    };
};

#endif // GPS_LOCATION_INFO_H

// $GPSACP: 161550.000,5950.9081N,01739.3691E,0.4,34.2,3,0.0,0.0,0.0,150126,10,06
