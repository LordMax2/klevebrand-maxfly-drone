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
    
    String toString() {
        return "FlightModeId: " + String(flight_mode_id) + ", EnablePower: " + String(enable_power) + ", EnableMotors: " + String(enable_motors) + ", Longitude: " + String(longitude) + ", Latitude: " + String(latitude);
    }

    static DroneRequest_t parseFromCsvString(String &value)
    {
        int flight_mode_id = 0;
        bool enable_power = 0;
        bool enable_motors = 0;
        float longitude = 0;
        float latitude = 0;

        const char *char_array_pointer = value.c_str();
        char *field_content = strtok((char *)char_array_pointer, ",");

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
};

#endif // DRONE_REQUEST_DTO_H