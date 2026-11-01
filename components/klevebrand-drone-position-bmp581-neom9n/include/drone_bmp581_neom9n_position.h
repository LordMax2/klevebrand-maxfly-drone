#pragma once

#include "Arduino.h"
#include <HardwareSerial.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "concept_drone_gyro.h"
#include "bmp581_barometer.h"
#include "kalman_engine_3x3.h"
#include "position_local_meters.h"

static constexpr float GPS_DEGREES_SCALE = 1e-7f;
static constexpr unsigned long GPS_BAUD_RATE = 38400UL;
static constexpr uint8_t GPS_NAVIGATION_FREQUENCY_HZ = 10;
static constexpr float METERS_PER_DEGREE_LATITUDE = 111320.0f;
static constexpr float GPS_POSITION_VARIANCE = 4.0f;
static constexpr float GPS_VELOCITY_VARIANCE = 0.01f;
static constexpr float GPS_VELOCITY_SCALE = 1e-3f;
static constexpr float GPS_MILLIMETERS_TO_METERS = 1e-3f;
static constexpr float GPS_ALTITUDE_VARIANCE_FLOOR = 4.0f;

/*
 * Position and velocity from the NEO-M9N, fused with the gyro's acceleration through a Kalman filter per axis, plus
 * pressure and temperature from the BMP581 on the default SPI bus. setup() waits for a 3D GPS fix.
 */
template <DroneGyroConcept SomeDroneGyroType>
class DroneBmp581Neom9nPosition
{
    SFE_UBLOX_GNSS_SERIAL _gps;
    HardwareSerial &_gps_serial;
    Bmp581Barometer _barometer;

    float _latitude = 0.0f;
    float _longitude = 0.0f;
    float _origin_latitude = 0.0f;
    float _origin_longitude = 0.0f;
    float _origin_altitude_meters = 0.0f;

    unsigned long _last_imu_update_microseconds = 0;

    bool _ready = false;
    bool _has_3d_fix = false;

    KalmanEngine3x3 _kalman_altitude{0.01f, 0.01f, 0.0001f};
    KalmanEngine3x3 _kalman_east{0.01f, 0.01f, 0.0001f};
    KalmanEngine3x3 _kalman_north{0.01f, 0.01f, 0.0001f};

    SomeDroneGyroType *const _gyro;

    PositionLocalMeters latitudeLongitudeToLocalMeters(float latitude, float longitude) const;
    float localMetersToLatitude(float north_meters) const;
    float localMetersToLongitude(float east_meters) const;

    bool setupNeoM9n();

    void updateFilters(bool has_gyro_update);

public:
    DroneBmp581Neom9nPosition(SomeDroneGyroType *gyro, HardwareSerial &gps_serial,
                              const uint8_t barometer_chip_select_pin, const int barometer_read_frequency_hz = 25)
        : _gps_serial(gps_serial), _barometer(barometer_chip_select_pin, barometer_read_frequency_hz), _gyro(gyro)
    {
    }

    void setup();

    float getTemperature() const { return _barometer.temperature(); }

    float getPressure() const { return _barometer.pressure(); }

    float getAltitude() const;

    float getVelocityZ() const;

    float getVelocityX() const;

    float getVelocityY() const;

    float getLongitude() const;

    float getLatitude() const;

    float getRawLongitude() const { return _longitude; }

    float getRawLatitude() const { return _latitude; }

    bool isReady() const { return _ready; }

    bool hasGpsFix() const { return _has_3d_fix; }

    void run(bool has_gyro_update);
};

#include "drone_bmp581_neom9n_position.ipp"
