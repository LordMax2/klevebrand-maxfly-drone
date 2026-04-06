#ifndef GPS_LOCATION_INFO_H
#define GPS_LOCATION_INFO_H

#include "Arduino.h"

#define GPS_TIMESTAMP_SIZE 16
#define GPS_DATE_SIZE 16
#define GPS_COURSE_SIZE 16

struct GpsLocationInfo_t
{
    GpsLocationInfo_t(
        const char *timestamp,
        const char *date,
        float latitude,
        float longitude,
        float altitude,
        int fix,
        const char *course_over_ground,
        float speed_over_ground_kmh,
        float speed_over_ground_knots,
        int number_of_satellites,
        int number_of_satellites_glonass)
    {
        strncpy(this->timestamp_utc, timestamp != nullptr ? timestamp : "", sizeof(this->timestamp_utc) - 1);
        this->timestamp_utc[sizeof(this->timestamp_utc) - 1] = '\0';
        strncpy(this->date, date != nullptr ? date : "", sizeof(this->date) - 1);
        this->date[sizeof(this->date) - 1] = '\0';
        this->latitude = latitude;
        this->longitude = longitude;
        this->altitude = altitude;
        this->fix = fix;
        strncpy(this->course_over_ground, course_over_ground != nullptr ? course_over_ground : "", sizeof(this->course_over_ground) - 1);
        this->course_over_ground[sizeof(this->course_over_ground) - 1] = '\0';
        this->speed_over_ground_kmh = speed_over_ground_kmh;
        this->speed_over_ground_knots = speed_over_ground_knots;
        this->number_of_satellites = number_of_satellites;
        this->number_of_satellites_glonass = number_of_satellites_glonass;
    }

    char timestamp_utc[GPS_TIMESTAMP_SIZE];
    char date[GPS_DATE_SIZE];
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
    char course_over_ground[GPS_COURSE_SIZE];
    float speed_over_ground_kmh;
    float speed_over_ground_knots;
    int number_of_satellites;
    int number_of_satellites_glonass;

    String toString() const
    {
        String result;
        result.reserve(192);
        result += F("Timestamp: ");
        result += timestamp_utc;
        result += F(", Date: ");
        result += date;
        result += F(", Latitude: ");
        result += latitude;
        result += F(", Longitude: ");
        result += longitude;
        result += F(", Altitude: ");
        result += altitude;
        result += F(", Fix: ");
        result += fix;
        result += F(", CourseOverGround: ");
        result += course_over_ground;
        result += F(", SpeedOverGroundKmh: ");
        result += speed_over_ground_kmh;
        result += F(", SpeedOverGroundKnots: ");
        result += speed_over_ground_knots;
        result += F(", NumberOfSatellites: ");
        result += number_of_satellites;
        result += F(", NumberOfSatellitesGlonass: ");
        result += number_of_satellites_glonass;
        return result;
    }

    static GpsLocationInfo_t empty()
    {
        return GpsLocationInfo_t("", "", 0.0f, 0.0f, 0.0f, 0, "", 0.0f, 0.0f, 0, 0);
    }

    static GpsLocationInfo_t parseFromGpsAcpString(const char *value)
    {
        char timestamp_utc[GPS_TIMESTAMP_SIZE] = {0};
        char date[GPS_DATE_SIZE] = {0};
        float latitude = 0.0f;
        float longitude = 0.0f;
        float altitude = 0.0f;
        int fix = 0;
        char course_over_ground[GPS_COURSE_SIZE] = {0};
        float speed_over_ground_kmh = 0.0f;
        float speed_over_ground_knots = 0.0f;
        int number_of_satellites = 0;
        int number_of_satellites_glonass = 0;

        char buffer[128];
        strncpy(buffer, value != nullptr ? value : "", sizeof(buffer) - 1);
        buffer[sizeof(buffer) - 1] = '\0';
        char *field_content = strtok(buffer, ",");

        int field_index = 0;

        while (field_content != NULL && field_index < 12)
        {
            switch (field_index)
            {
            case 0:
                strncpy(timestamp_utc, field_content, sizeof(timestamp_utc) - 1);
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
                strncpy(course_over_ground, field_content, sizeof(course_over_ground) - 1);
                break;
            case 7:
                speed_over_ground_knots = atof(field_content);
                break;
            case 8:
                speed_over_ground_kmh = atof(field_content);
                break;
            case 9:
                strncpy(date, field_content, sizeof(date) - 1);
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
    }

    static GpsLocationInfo_t parseFromGpsAcpString(String value)
    {
        return parseFromGpsAcpString(value.c_str());
    }
};

#endif // GPS_LOCATION_INFO_H

// $GPSACP: 161550.000,5950.9081N,01739.3691E,0.4,34.2,3,0.0,0.0,0.0,150126,10,06
