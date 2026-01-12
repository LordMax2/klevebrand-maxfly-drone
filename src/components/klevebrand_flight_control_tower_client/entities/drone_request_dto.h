#ifndef DRONE_REQUEST_DTO_H
#define DRONE_REQUEST_DTO_H

struct DroneRequestDto
{
    DroneRequestDto(
        int flight_mode_id,
        float throttle,
        float yaw,
        float pitch,
        float roll,
        bool enable_power,
        bool enable_motors,
        bool is_valid) : flight_mode_id(flight_mode_id),
                         throttle(throttle),
                         yaw(yaw),
                         pitch(pitch),
                         roll(roll),
                         enable_power(enable_power),
                         enable_motors(enable_motors), is_valid(is_valid) {}

    static DroneRequestDto parseFromCsvString(String value)
    {
        int flight_mode_id = 0;
        float throttle = 0.0f;
        float yaw = 0.0f;
        float pitch = 0.0f;
        float roll = 0.0f;
        bool enable_power = false;
        bool enable_motors = false;

        // strtok needs a char array pointer, so we convert the String to a char array pointer
        const char *char_array_pointer = value.c_str();
        char *filed_content = strtok((char *)char_array_pointer, ",");

        int field_index = 0;

        // There is always 7 fields in the response content, so we look for the first 7 fields, its hard coded, but fast and quite simple
        while (filed_content != NULL && field_index < 7)
        {
            switch (field_index)
            {
            case 0:
                flight_mode_id = atoi(filed_content);
                break;
            case 1:
                throttle = atof(filed_content);
                break;
            case 2:
                yaw = atof(filed_content);
                break;
            case 3:
                pitch = atof(filed_content);
                break;
            case 4:
                roll = atof(filed_content);
                break;
            case 5:
                enable_power = atob(filed_content[0]);
                break;
            case 6:
                enable_motors = atob(filed_content[0]);
                break;
            }

            field_index++;

            // Look for the next field
            filed_content = strtok(NULL, ",");
        }

        return DroneRequestDto(
            flight_mode_id,
            throttle,
            yaw,
            pitch,
            roll,
            enable_power,
            enable_motors,
            true);
    };

    static DroneRequestDto getInvalidDroneRequest()
    {
        return DroneRequestDto(
            0,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            false,
            false,
            false);
    }

    const int flight_mode_id;
    const float throttle;
    const float yaw;
    const float pitch;
    const float roll;
    const bool enable_power;
    const bool enable_motors;
    const bool is_valid;

private:
    static bool atob(char value)
    {
        return (value == '1' || value == 't' || value == 'T');
    }
};

#endif // DRONE_REQUEST_DTO_H