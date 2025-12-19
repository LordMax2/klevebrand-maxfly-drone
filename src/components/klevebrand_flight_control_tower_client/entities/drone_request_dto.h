#ifndef DRONE_REQUEST_DTO_H
#define DRONE_REQUEST_DTO_H

struct DroneRequestDto {
    DroneRequestDto(
        int flight_mode_id,
        float throttle,
        float yaw,
        float pitch,
        float roll,
        bool enable_power,
        bool enable_motors,
        bool is_valid
    ) : flight_mode_id(flight_mode_id),
        throttle(throttle),
        yaw(yaw),
        pitch(pitch),
        roll(roll),
        enable_power(enable_power),
        enable_motors(enable_motors), is_valid(is_valid) {}

    const int flight_mode_id;
    const float throttle;
    const float yaw;
    const float pitch;
    const float roll;
    const bool enable_power;
    const bool enable_motors;
    const bool is_valid;
};

#endif // DRONE_REQUEST_DTO_H