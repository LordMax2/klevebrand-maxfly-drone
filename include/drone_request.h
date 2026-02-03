#ifndef DRONE_REQUEST_DTO_H
#define DRONE_REQUEST_DTO_H

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

    const int flight_mode_id;
    const bool enable_power;
    const bool enable_motors;
    const float longitude;
    const float latitude;
};

#endif // DRONE_REQUEST_DTO_H