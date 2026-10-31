#include "drone_components/bno08x_spi_drone_gyro.h"

static constexpr unsigned long RESET_PULSE_MILLISECONDS = 10;

unsigned long Bno08xSpiDroneGyro::timestampMilliseconds()
{
    return _yaw_pitch_roll.timestamp_milliseconds;
}

bool Bno08xSpiDroneGyro::setModeEuler()
{
    const bool result = _gyro.enableReport(SH2_ARVR_STABILIZED_RV, 4000);

    if (!result)
    {
        Serial.println("FAILED TO SETUP EULER MODE.");
        return false;
    }

    _mode = MODE_EULER;

    return result;
}

bool Bno08xSpiDroneGyro::setModeAcro()
{
    const bool result = _gyro.enableReport(SH2_GYROSCOPE_CALIBRATED, 2000);

    if (!result)
    {
        Serial.println("FAILED TO SETUP ACRO MODE.");
        return false;
    }

    _mode = MODE_ACRO;

    return result;
}

bool Bno08xSpiDroneGyro::setModeEulerAndAcceleration()
{
    const bool euler_enabled = _gyro.enableReport(SH2_ARVR_STABILIZED_RV, 4000);
    const bool acceleration_enabled = _gyro.enableReport(SH2_ACCELEROMETER, 4000);

    if (!euler_enabled || !acceleration_enabled)
    {
        Serial.println("FAILED TO SETUP EULER AND ACCELERATION MODE.");
        return false;
    }

    _mode = MODE_EULER_AND_ACCELERATION;

    return true;
}

void Bno08xSpiDroneGyro::setup()
{
    Serial.println("SETTING UP GYROSCOPE.");

    // PS0/WAKE high selects SPI as the BNO085 comes out of this reset (the board pulls it up as well).
    ImxrtGpioPad::emc(wake_pad).setModeOutput(true);
    _reset_pad.setModeOutput(true);
    reset();

    SPI1.setMISO(miso_pin);

    if (!_gyro.begin_SPI(chip_select_pin, interrupt_pin, &SPI1))
    {
        Serial.println("FAILED TO CONNECT TO BNO085...");

        while (true)
        {
            delay(10);
        }
    }

    Serial.println("BNO085 SET UP!");

    delay(300);
}

void Bno08xSpiDroneGyro::reset()
{
    _reset_pad.write(false);
    delay(RESET_PULSE_MILLISECONDS);

    _reset_pad.write(true);
    delay(RESET_PULSE_MILLISECONDS);
}

float Bno08xSpiDroneGyro::yaw()
{
    return _yaw_pitch_roll.yaw;
}

float Bno08xSpiDroneGyro::pitch()
{
    return _yaw_pitch_roll.pitch;
}

float Bno08xSpiDroneGyro::roll()
{
    return _yaw_pitch_roll.roll;
}

float Bno08xSpiDroneGyro::accelerationX()
{
    if (_mode == MODE_EULER_AND_ACCELERATION) return _rotated_acceleration.x;

    return _acceleration.x;
}

float Bno08xSpiDroneGyro::accelerationY()
{
    if (_mode == MODE_EULER_AND_ACCELERATION) return _rotated_acceleration.y;

    return _acceleration.y;
}

float Bno08xSpiDroneGyro::accelerationZ()
{
    if (_mode == MODE_EULER_AND_ACCELERATION) return _rotated_acceleration.z;

    return _acceleration.z;
}

void Bno08xSpiDroneGyro::printYawPitchRoll()
{
    Serial.print(_yaw_pitch_roll.yaw);
    Serial.print("\t");
    Serial.print(_yaw_pitch_roll.pitch);
    Serial.print("\t");
    Serial.println(_yaw_pitch_roll.roll);
}

bool Bno08xSpiDroneGyro::reload()
{
    bool updated = false;
    bool has_euler = false;
    bool has_acceleration = false;
    const bool needs_acceleration = _mode == MODE_EULER_AND_ACCELERATION;
    const uint8_t max_events_to_drain = needs_acceleration ? 8 : 1;

    for (uint8_t index = 0; index < max_events_to_drain; index++)
    {
        if (!_gyro.getSensorEvent(&_sensor_value))
        {
            break;
        }

        if (_sensor_value.sensorId == SH2_ARVR_STABILIZED_RV)
        {
            _rotational_vector = _sensor_value.un.arvrStabilizedRV;

            const YawPitchRoll_t yaw_pitch_roll = quaternionsToYawPitchRoll(&_sensor_value.un.arvrStabilizedRV, true);

            _yaw_pitch_roll.yaw = yaw_pitch_roll.yaw;
            _yaw_pitch_roll.pitch = yaw_pitch_roll.pitch;
            _yaw_pitch_roll.roll = yaw_pitch_roll.roll;

            has_euler = true;
            updated = true;
        }

        if (_sensor_value.sensorId == SH2_GYROSCOPE_CALIBRATED)
        {
            const float gyro_roll = _sensor_value.un.gyroscope.x;
            const float gyro_pitch = _sensor_value.un.gyroscope.y;
            const float gyro_yaw = _sensor_value.un.gyroscope.z;

            _yaw_pitch_roll.yaw = gyro_yaw * RAD_TO_DEG;
            _yaw_pitch_roll.pitch = gyro_pitch * RAD_TO_DEG;
            _yaw_pitch_roll.roll = gyro_roll * RAD_TO_DEG;

            updated = true;
        }

        if (_sensor_value.sensorId == SH2_ACCELEROMETER)
        {
            _acceleration = _sensor_value.un.accelerometer;

            has_acceleration = true;
            updated = true;
        }

        if (updated)
        {
            _yaw_pitch_roll.timestamp_milliseconds = millis();
        }

        if (!needs_acceleration)
        {
            break;
        }

        if (has_euler && has_acceleration)
        {
            break;
        }
    }

    if (needs_acceleration && updated)
    {
        _rotated_acceleration.x = _acceleration.x;
        _rotated_acceleration.y = _acceleration.y;
        _rotated_acceleration.z = _acceleration.z;

        rotateVectorByQuaternion(
            _rotated_acceleration.x, _rotated_acceleration.y, _rotated_acceleration.z,
            _rotational_vector.real, _rotational_vector.i, _rotational_vector.j, _rotational_vector.k
        );

        _rotated_acceleration.z -= 9.80665f;
    }

    return updated;
}

void Bno08xSpiDroneGyro::rotateVectorByQuaternion(float &x, float &y, float &z,
                                                  const float r, const float i, const float j, const float k)
{
    const float tx = 2.0f * (j * z - k * y);
    const float ty = 2.0f * (k * x - i * z);
    const float tz = 2.0f * (i * y - j * x);

    x += r * tx + (j * tz - k * ty);
    y += r * ty + (k * tx - i * tz);
    z += r * tz + (i * ty - j * tx);
}

YawPitchRoll_t Bno08xSpiDroneGyro::quaternionsToYawPitchRoll(const sh2_RotationVectorWAcc_t *rotational_vector,
                                                             const bool to_degrees)
{
    return quaternionsToYawPitchRoll(rotational_vector->real, rotational_vector->i, rotational_vector->j,
                                     rotational_vector->k, to_degrees);
}

YawPitchRoll_t Bno08xSpiDroneGyro::quaternionsToYawPitchRoll(const float qr, const float qi, const float qj,
                                                             const float qk, const bool to_degrees)
{
    YawPitchRoll_t yaw_pitch_roll;

    const float sqr = sq(qr);
    const float sqi = sq(qi);
    const float sqj = sq(qj);
    const float sqk = sq(qk);

    yaw_pitch_roll.yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    yaw_pitch_roll.pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    yaw_pitch_roll.roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (to_degrees)
    {
        yaw_pitch_roll.yaw *= RAD_TO_DEG;
        yaw_pitch_roll.pitch *= RAD_TO_DEG;
        yaw_pitch_roll.roll *= RAD_TO_DEG;
    }

    return yaw_pitch_roll;
}
