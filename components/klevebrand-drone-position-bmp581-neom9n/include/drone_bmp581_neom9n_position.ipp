#include <math.h>

template <DroneGyroConcept SomeDroneGyroType>
PositionLocalMeters DroneBmp581Neom9nPosition<SomeDroneGyroType>::latitudeLongitudeToLocalMeters(const float latitude,
                                                                                                 const float longitude) const
{
    const float origin_latitude_radians = _origin_latitude * DEG_TO_RAD;
    const float meters_per_degree_longitude = METERS_PER_DEGREE_LATITUDE * cos(origin_latitude_radians);

    return PositionLocalMeters{
        (longitude - _origin_longitude) * meters_per_degree_longitude,
        (latitude - _origin_latitude) * METERS_PER_DEGREE_LATITUDE
    };
}

template <DroneGyroConcept SomeDroneGyroType>
float DroneBmp581Neom9nPosition<SomeDroneGyroType>::localMetersToLatitude(const float north_meters) const
{
    return _origin_latitude + north_meters / METERS_PER_DEGREE_LATITUDE;
}

template <DroneGyroConcept SomeDroneGyroType>
float DroneBmp581Neom9nPosition<SomeDroneGyroType>::localMetersToLongitude(const float east_meters) const
{
    const float origin_latitude_radians = _origin_latitude * DEG_TO_RAD;
    const float meters_per_degree_longitude = METERS_PER_DEGREE_LATITUDE * cos(origin_latitude_radians);

    return _origin_longitude + east_meters / meters_per_degree_longitude;
}

template <DroneGyroConcept SomeDroneGyroType>
bool DroneBmp581Neom9nPosition<SomeDroneGyroType>::setupNeoM9n()
{
    _gps_serial.begin(GPS_BAUD_RATE);
    delay(100);

    while (_gps_serial.available() > 0)
    {
        _gps_serial.read();
    }

    Serial.print(F("CONNECTING TO NEO-M9N"));

    if (!_gps.begin(_gps_serial))
    {
        Serial.println(F("FAILED TO CONNECT TO NEO-M9N."));

        return false;
    }

    Serial.println(F("SUCCESSFULLY CONNECTED TO NEO-M9N"));

    _gps.setUART1Output(COM_TYPE_UBX);
    _gps.setNavigationFrequency(GPS_NAVIGATION_FREQUENCY_HZ);
    _gps.setAutoPVT(true);

    Serial.print(F("NAVIGATION FREQUENCY SET TO "));
    Serial.print(GPS_NAVIGATION_FREQUENCY_HZ);
    Serial.println(F(" HZ"));
    Serial.println(F("WAITING FOR GPS LOCK..."));

    const unsigned long gps_wait_start_ms = millis();

    while (true)
    {
        const bool got_pvt = _gps.getPVT(1000);

        Serial.print(F("GPS WAIT t="));
        Serial.print((millis() - gps_wait_start_ms) / 1000UL);
        Serial.print(F("s PVT="));
        Serial.print(got_pvt ? 1 : 0);

        if (!got_pvt)
        {
            Serial.println(F(" (no nav packet yet)"));
            continue;
        }

        const uint8_t fix_type = _gps.getFixType(0);
        const uint8_t satellite_count = _gps.getSIV(0);

        _latitude = _gps.getLatitude() * GPS_DEGREES_SCALE;
        _longitude = _gps.getLongitude() * GPS_DEGREES_SCALE;

        Serial.print(F(" FIX="));
        Serial.print(fix_type);
        Serial.print(F(" SATS="));
        Serial.print(satellite_count);
        Serial.print(F(" LAT="));
        Serial.print(_latitude, 7);
        Serial.print(F(" LON="));
        Serial.println(_longitude, 7);

        if (fix_type >= 3)
        {
            _origin_latitude = _latitude;
            _origin_longitude = _longitude;
            _origin_altitude_meters = _gps.getAltitudeMSL(0) * GPS_MILLIMETERS_TO_METERS;
            _has_3d_fix = true;

            Serial.println(F("GPS LOCK ACQUIRED"));

            break;
        }
    }

    return true;
}

template <DroneGyroConcept SomeDroneGyroType>
void DroneBmp581Neom9nPosition<SomeDroneGyroType>::setup()
{
    if (!_barometer.setup())
    {
        Serial.println(F("BMP581 UNAVAILABLE. CONTINUING WITHOUT PRESSURE."));
    }

    if (!setupNeoM9n())
    {
        return;
    }

    _kalman_altitude.reset();
    _kalman_east.reset();
    _kalman_north.reset();

    _ready = true;
}

template <DroneGyroConcept SomeDroneGyroType>
float DroneBmp581Neom9nPosition<SomeDroneGyroType>::getAltitude() const
{
    return _kalman_altitude.getPosition();
}

template <DroneGyroConcept SomeDroneGyroType>
float DroneBmp581Neom9nPosition<SomeDroneGyroType>::getVelocityZ() const
{
    return _kalman_altitude.getVelocity();
}

template <DroneGyroConcept SomeDroneGyroType>
float DroneBmp581Neom9nPosition<SomeDroneGyroType>::getVelocityX() const
{
    return _kalman_east.getVelocity();
}

template <DroneGyroConcept SomeDroneGyroType>
float DroneBmp581Neom9nPosition<SomeDroneGyroType>::getVelocityY() const
{
    return _kalman_north.getVelocity();
}

template <DroneGyroConcept SomeDroneGyroType>
float DroneBmp581Neom9nPosition<SomeDroneGyroType>::getLatitude() const
{
    return localMetersToLatitude(_kalman_north.getPosition());
}

template <DroneGyroConcept SomeDroneGyroType>
float DroneBmp581Neom9nPosition<SomeDroneGyroType>::getLongitude() const
{
    return localMetersToLongitude(_kalman_east.getPosition());
}

template <DroneGyroConcept SomeDroneGyroType>
void DroneBmp581Neom9nPosition<SomeDroneGyroType>::run(const bool has_gyro_update)
{
    if (isReady())
    {
        updateFilters(has_gyro_update);
    }

    // The BMP581 doesn't depend on the GPS, so pressure and temperature keep updating without one.
    _barometer.run();
}

template <DroneGyroConcept SomeDroneGyroType>
void DroneBmp581Neom9nPosition<SomeDroneGyroType>::updateFilters(const bool has_gyro_update)
{
    const unsigned long now = micros();

    if (_gps.getPVT())
    {
        _latitude = _gps.getLatitude() * GPS_DEGREES_SCALE;
        _longitude = _gps.getLongitude() * GPS_DEGREES_SCALE;

        _has_3d_fix = _gps.getFixType(0) >= 3;

        if (_has_3d_fix)
        {
            const auto [east_meters, north_meters] = latitudeLongitudeToLocalMeters(_latitude, _longitude);
            const float east_velocity_meters_per_second = _gps.getNedEastVel(0) * GPS_VELOCITY_SCALE;
            const float north_velocity_meters_per_second = _gps.getNedNorthVel(0) * GPS_VELOCITY_SCALE;
            const float up_velocity_meters_per_second = -_gps.getNedDownVel(0) * GPS_VELOCITY_SCALE;
            const float altitude_above_origin_meters = _gps.getAltitudeMSL(0) * GPS_MILLIMETERS_TO_METERS - _origin_altitude_meters;
            const float vertical_accuracy_meters = _gps.getVerticalAccEst(0) * GPS_MILLIMETERS_TO_METERS;
            const float vertical_accuracy_variance = vertical_accuracy_meters * vertical_accuracy_meters;
            const float altitude_variance = vertical_accuracy_variance > GPS_ALTITUDE_VARIANCE_FLOOR
                ? vertical_accuracy_variance
                : GPS_ALTITUDE_VARIANCE_FLOOR;

            _kalman_east.updateZeroState(east_meters, GPS_POSITION_VARIANCE);
            _kalman_north.updateZeroState(north_meters, GPS_POSITION_VARIANCE);
            _kalman_altitude.updateZeroState(altitude_above_origin_meters, altitude_variance);

            _kalman_east.updateVelocityState(east_velocity_meters_per_second, GPS_VELOCITY_VARIANCE);
            _kalman_north.updateVelocityState(north_velocity_meters_per_second, GPS_VELOCITY_VARIANCE);
            _kalman_altitude.updateVelocityState(up_velocity_meters_per_second, GPS_VELOCITY_VARIANCE);
        }
    }

    if (has_gyro_update)
    {
        const float imu_acceleration_x = _gyro->accelerationX();
        const float imu_acceleration_y = _gyro->accelerationY();
        const float imu_acceleration_z = _gyro->accelerationZ();

        if (_last_imu_update_microseconds != 0)
        {
            if (const float delta_time_seconds = (now - _last_imu_update_microseconds) * 1e-6f; delta_time_seconds > 0.0f && delta_time_seconds < 0.1f)
            {
                _kalman_east.predictKinematics(imu_acceleration_x, delta_time_seconds);
                _kalman_north.predictKinematics(imu_acceleration_y, delta_time_seconds);
                _kalman_altitude.predictKinematics(imu_acceleration_z, delta_time_seconds);
            }
        }

        _last_imu_update_microseconds = now;
    }
}
