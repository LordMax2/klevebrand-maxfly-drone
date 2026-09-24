#pragma once

#ifndef DRONE_REQUEST_H
#define DRONE_REQUEST_H

// Firmware-local DroneRequest: same include guard as klevebrand-skywire-framework so GetDroneRequest
// parsing can read FCT assignmentAltitude (CSV field 6 / JSON). 0 or omitted means no altitude command.

#include <stdlib.h>
#include <string.h>

namespace drone_request_detail
{
inline const char *skipWhitespace(const char *value)
{
    while (value != nullptr && (*value == ' ' || *value == '\t' || *value == '\n' || *value == '\r'))
    {
        value++;
    }

    return value;
}

inline bool looksLikeJsonObject(const char *value)
{
    const char *cursor = skipWhitespace(value);

    return cursor != nullptr && *cursor == '{';
}

inline const char *findJsonValue(const char *json, const char *key)
{
    if (json == nullptr || key == nullptr)
    {
        return nullptr;
    }

    const size_t key_length = strlen(key);
    const char *cursor = json;

    while (*cursor != '\0')
    {
        if (*cursor != '"')
        {
            cursor++;
            continue;
        }

        cursor++;

        if (strncmp(cursor, key, key_length) == 0 && cursor[key_length] == '"')
        {
            cursor = skipWhitespace(cursor + key_length + 1);

            if (*cursor != ':')
            {
                continue;
            }

            return skipWhitespace(cursor + 1);
        }
    }

    return nullptr;
}

inline bool tryReadJsonFloat(const char *json, const char *key, float &out_value)
{
    const char *value = findJsonValue(json, key);

    if (value == nullptr || *value == '\0' || *value == ',' || *value == '}' || *value == 'n')
    {
        return false;
    }

    if (*value == '"')
    {
        value++;
    }

    out_value = static_cast<float>(atof(value));

    return true;
}

inline bool tryReadJsonInt(const char *json, const char *key, int &out_value)
{
    const char *value = findJsonValue(json, key);

    if (value == nullptr || *value == '\0' || *value == ',' || *value == '}' || *value == 'n')
    {
        return false;
    }

    if (*value == '"')
    {
        value++;
    }

    out_value = atoi(value);

    return true;
}

inline bool tryReadJsonBool(const char *json, const char *key, bool &out_value)
{
    const char *value = findJsonValue(json, key);

    if (value == nullptr || *value == '\0' || *value == ',' || *value == '}' || *value == 'n')
    {
        return false;
    }

    if (*value == '"')
    {
        value++;
    }

    if (strncmp(value, "true", 4) == 0)
    {
        out_value = true;

        return true;
    }

    if (strncmp(value, "false", 5) == 0)
    {
        out_value = false;

        return true;
    }

    out_value = atoi(value) == 1;

    return true;
}

} // namespace drone_request_detail

struct DroneRequest_t
{
    DroneRequest_t() = default;

    DroneRequest_t(
        const int flight_mode_id,
        const bool enable_power,
        const bool enable_motors,
        const float longitude,
        const float latitude,
        const float assignment_altitude = 0.0f) : flight_mode_id(flight_mode_id),
                                                  enable_power(enable_power),
                                                  enable_motors(enable_motors),
                                                  longitude(longitude),
                                                  latitude(latitude),
                                                  assignment_altitude(assignment_altitude)
    {
    }

    int flight_mode_id = 0;
    bool enable_power = false;
    bool enable_motors = false;
    float longitude = 0.0f;
    float latitude = 0.0f;
    float assignment_altitude = 0.0f;

    static DroneRequest_t empty()
    {
        return {0, false, false, 0.0f, 0.0f, 0.0f};
    }

    bool hasCommandedAltitude() const
    {
        return assignment_altitude != 0.0f;
    }

    static DroneRequest_t parseFromJsonString(const char *value)
    {
        int flight_mode_id = 0;
        bool enable_power = false;
        bool enable_motors = false;
        float longitude = 0.0f;
        float latitude = 0.0f;
        float assignment_altitude = 0.0f;

        if (!drone_request_detail::tryReadJsonInt(value, "flightModeId", flight_mode_id))
        {
            (void)drone_request_detail::tryReadJsonInt(value, "FlightModeId", flight_mode_id);
        }

        if (!drone_request_detail::tryReadJsonBool(value, "enablePower", enable_power))
        {
            (void)drone_request_detail::tryReadJsonBool(value, "EnablePower", enable_power);
        }

        if (!drone_request_detail::tryReadJsonBool(value, "enableMotors", enable_motors))
        {
            (void)drone_request_detail::tryReadJsonBool(value, "EnableMotors", enable_motors);
        }

        if (!drone_request_detail::tryReadJsonFloat(value, "lon", longitude) &&
            !drone_request_detail::tryReadJsonFloat(value, "Lon", longitude))
        {
            if (!drone_request_detail::tryReadJsonFloat(value, "longitude", longitude))
            {
                (void)drone_request_detail::tryReadJsonFloat(value, "Longitude", longitude);
            }
        }

        if (!drone_request_detail::tryReadJsonFloat(value, "lat", latitude) &&
            !drone_request_detail::tryReadJsonFloat(value, "Lat", latitude))
        {
            if (!drone_request_detail::tryReadJsonFloat(value, "latitude", latitude))
            {
                (void)drone_request_detail::tryReadJsonFloat(value, "Latitude", latitude);
            }
        }

        if (!drone_request_detail::tryReadJsonFloat(value, "assignmentAltitude", assignment_altitude))
        {
            (void)drone_request_detail::tryReadJsonFloat(value, "AssignmentAltitude", assignment_altitude);
        }

        return {flight_mode_id, enable_power, enable_motors, longitude, latitude, assignment_altitude};
    }

    static DroneRequest_t parseFromCsvString(char *value)
    {
        if (drone_request_detail::looksLikeJsonObject(value))
        {
            return parseFromJsonString(value);
        }

        int flight_mode_id = 0;
        bool enable_power = false;
        bool enable_motors = false;
        float longitude = 0;
        float latitude = 0;
        float assignment_altitude = 0;
        char empty[] = "";

        char *field_content = strtok(value != nullptr ? value : empty, ",");
        int field_index = 0;

        while (field_content != nullptr && field_index < 6)
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
                longitude = static_cast<float>(atof(field_content));
                break;
            case 4:
                latitude = static_cast<float>(atof(field_content));
                break;
            case 5:
                assignment_altitude = static_cast<float>(atof(field_content));
                break;
            default:;
            }

            field_index++;
            field_content = strtok(nullptr, ",");
        }

        return {flight_mode_id, enable_power, enable_motors, longitude, latitude, assignment_altitude};
    }
};

#endif
