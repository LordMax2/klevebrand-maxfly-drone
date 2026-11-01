#include "kalman_engine_3x3.h"

void KalmanEngine3x3::reset()
{
    _state_vector[0] = 0.0f;
    _state_vector[1] = 0.0f;
    _state_vector[2] = 0.0f;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            _covariance_matrix[i][j] = i == j ? 1.0f : 0.0f;
        }
    }
}

float KalmanEngine3x3::predictKinematics(const float input, const float delta_time_seconds)
{
    const float pure_input = input - _state_vector[2];

    _state_vector[0] = _state_vector[0] + _state_vector[1] * delta_time_seconds + 0.5f * pure_input * delta_time_seconds * delta_time_seconds;
    _state_vector[1] = _state_vector[1] + pure_input * delta_time_seconds;

    float p00 = _covariance_matrix[0][0], p01 = _covariance_matrix[0][1], p02 = _covariance_matrix[0][2];
    float p10 = _covariance_matrix[1][0], p11 = _covariance_matrix[1][1], p12 = _covariance_matrix[1][2];
    float p20 = _covariance_matrix[2][0], p21 = _covariance_matrix[2][1], p22 = _covariance_matrix[2][2];

    _covariance_matrix[0][0] = p00 + delta_time_seconds * (p10 + p01 + delta_time_seconds * p11) - delta_time_seconds * delta_time_seconds * (p20 + p02 + delta_time_seconds * p21) + 0.25f * delta_time_seconds * delta_time_seconds * delta_time_seconds * delta_time_seconds * p22 + _process_noise_position;
    _covariance_matrix[0][1] = p01 + delta_time_seconds * p11 - 0.5f * delta_time_seconds * delta_time_seconds * p21 - delta_time_seconds * p02 - 0.5f * delta_time_seconds * delta_time_seconds * p12 + 0.5f * delta_time_seconds * delta_time_seconds * delta_time_seconds * p22;
    _covariance_matrix[0][2] = p02 + delta_time_seconds * p12 - 0.5f * delta_time_seconds * delta_time_seconds * p22;

    _covariance_matrix[1][0] = p10 + delta_time_seconds * p11 - delta_time_seconds * p20 - 0.5f * delta_time_seconds * delta_time_seconds * p21 - 0.5f * delta_time_seconds * delta_time_seconds * p12 + 0.5f * delta_time_seconds * delta_time_seconds * delta_time_seconds * p22;
    _covariance_matrix[1][1] = p11 - delta_time_seconds * p21 - delta_time_seconds * p12 + delta_time_seconds * delta_time_seconds * p22 + _process_noise_velocity;
    _covariance_matrix[1][2] = p12 - delta_time_seconds * p22;

    _covariance_matrix[2][0] = p20 + delta_time_seconds * p21 - 0.5f * delta_time_seconds * delta_time_seconds * p22;
    _covariance_matrix[2][1] = p21 - delta_time_seconds * p22;
    _covariance_matrix[2][2] = p22 + _process_noise_bias;

    return _state_vector[0];
}

float KalmanEngine3x3::updateZeroState(const float measured_value, const float R_sensor)
{
    const float error = measured_value - _state_vector[0];
    const float S = _covariance_matrix[0][0] + R_sensor;

    if (S == 0.0f)
    {
        return error;
    }

    float K[3];
    K[0] = _covariance_matrix[0][0] / S;
    K[1] = _covariance_matrix[1][0] / S;
    K[2] = _covariance_matrix[2][0] / S;

    _state_vector[0] += K[0] * error;
    _state_vector[1] += K[1] * error;
    _state_vector[2] += K[2] * error;

    float p00 = _covariance_matrix[0][0], p01 = _covariance_matrix[0][1], p02 = _covariance_matrix[0][2];

    _covariance_matrix[0][0] -= K[0] * p00;  _covariance_matrix[0][1] -= K[0] * p01;  _covariance_matrix[0][2] -= K[0] * p02;
    _covariance_matrix[1][0] -= K[1] * p00;  _covariance_matrix[1][1] -= K[1] * p01;  _covariance_matrix[1][2] -= K[1] * p02;
    _covariance_matrix[2][0] -= K[2] * p00;  _covariance_matrix[2][1] -= K[2] * p01;  _covariance_matrix[2][2] -= K[2] * p02;

    return error;
}

float KalmanEngine3x3::updateVelocityState(const float measured_velocity, const float R_sensor)
{
    const float error = measured_velocity - _state_vector[1];
    const float S = _covariance_matrix[1][1] + R_sensor;

    if (S == 0.0f)
    {
        return error;
    }

    float K[3];
    K[0] = _covariance_matrix[0][1] / S;
    K[1] = _covariance_matrix[1][1] / S;
    K[2] = _covariance_matrix[2][1] / S;

    _state_vector[0] += K[0] * error;
    _state_vector[1] += K[1] * error;
    _state_vector[2] += K[2] * error;

    float p10 = _covariance_matrix[1][0], p11 = _covariance_matrix[1][1], p12 = _covariance_matrix[1][2];

    _covariance_matrix[0][0] -= K[0] * p10;  _covariance_matrix[0][1] -= K[0] * p11;  _covariance_matrix[0][2] -= K[0] * p12;
    _covariance_matrix[1][0] -= K[1] * p10;  _covariance_matrix[1][1] -= K[1] * p11;  _covariance_matrix[1][2] -= K[1] * p12;
    _covariance_matrix[2][0] -= K[2] * p10;  _covariance_matrix[2][1] -= K[2] * p11;  _covariance_matrix[2][2] -= K[2] * p12;

    return error;
}
