#pragma once

class KalmanEngine3x3
{
    float _state_vector[3]{};

    float _covariance_matrix[3][3]{};

    const float _process_noise_position;
    const float _process_noise_velocity;
    const float _process_noise_bias;

public:
    explicit KalmanEngine3x3(float process_noise_position = 0.01f,
                             float process_noise_velocity = 0.01f,
                             float process_noise_bias = 0.0001f)
        : _process_noise_position(process_noise_position),
          _process_noise_velocity(process_noise_velocity),
          _process_noise_bias(process_noise_bias)
    {
    }

    ~KalmanEngine3x3() = default;

    float predictKinematics(float input, float delta_time_seconds);
    float updateZeroState(float measured_value, float R_sensor);
    float updateVelocityState(float measured_velocity, float R_sensor);

    void reset();

    float getVelocity() const
    {
        return _state_vector[1];
    }

    float getPosition() const
    {
        return _state_vector[0];
    }
};
