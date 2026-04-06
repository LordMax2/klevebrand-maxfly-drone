#ifndef DRONE_REQUEST_DTO_H
#define DRONE_REQUEST_DTO_H

#include "Arduino.h"

struct DroneRequest_t
{
    DroneRequest_t(
        int flight_mode_id,
        bool enable_power,
        bool enable_motors,
        float longitude,
        float latitude) : flight_mode_id(flight_mode_id),
                          enable_power(enable_power),
                          enable_motors(enable_motors),
                          longitude(longitude),
                          latitude(latitude) {}

    int flight_mode_id;
    bool enable_power;
    bool enable_motors;
    float longitude;
    float latitude;

    static DroneRequest_t empty() {
        return DroneRequest_t(0, false, false, 0.0f, 0.0f);
    }
    
    String toString() const {
        String result;
        result.reserve(96);
        result += F("FlightModeId: ");
        result += flight_mode_id;
        result += F(", EnablePower: ");
        result += enable_power;
        result += F(", EnableMotors: ");
        result += enable_motors;
        result += F(", Longitude: ");
        result += longitude;
        result += F(", Latitude: ");
        result += latitude;
        return result;
    }

    static DroneRequest_t parseFromCsvString(const char *value)
    {
        int flight_mode_id = 0;
        bool enable_power = 0;
        bool enable_motors = 0;
        float longitude = 0;
        float latitude = 0;

        char buffer[96];
        strncpy(buffer, value != nullptr ? value : "", sizeof(buffer) - 1);
        buffer[sizeof(buffer) - 1] = '\0';
        char *field_content = strtok(buffer, ",");

        int field_index = 0;

        while (field_content != NULL && field_index < 5)
        {
            switch (field_index)
            {
            case 0:
                flight_mode_id = atoi(field_content);
                break;
            case 1:
                enable_power = atoi(field_content) == 1;
                break;
            case 2:
                enable_motors = atoi(field_content) == 1;
                break;
            case 3:
                longitude = atof(field_content);
                break;
            case 4:
                latitude = atof(field_content);
                break;
            }

            field_index++;

            field_content = strtok(NULL, ",");
        }

        return DroneRequest_t(flight_mode_id, enable_power, enable_motors, longitude, latitude);
    }

    static DroneRequest_t parseFromCsvString(String &value)
    {
        return parseFromCsvString(value.c_str());
    }
};

#endif // DRONE_REQUEST_DTO_H
