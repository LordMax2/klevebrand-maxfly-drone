#ifndef KLEVEBRAND_MAXFLY_DRONE_QUADCOPTER_POSITION_H
#define KLEVEBRAND_MAXFLY_DRONE_QUADCOPTER_POSITION_H

#include <Adafruit_BMP280.h>
#include "base_drone_position.h"

class QuadcopterPosition : public BaseDronePosition {
    Adafruit_BMP280 _bmp_device;

    float _temperature = 0.0f;
    float _pressure = 0.0f;
    float _altitude = 0.0f;

    unsigned long _last_run_timestamp_microseconds = 0;

    static constexpr unsigned long run_interval_micros = 40000;
    static constexpr float sea_level_pressure_pa = 101325.0f;

    static float pressureToAltitudeMeters(float pressure_pa, float sea_level_pressure_pa);

public:
    QuadcopterPosition() = default;

    void setup();

    float getTemperature() const { return _temperature; }

    float getPressure() const { return _pressure; }

    float getAltitude() override;

    void run();
};

#endif // KLEVEBRAND_MAXFLY_DRONE_QUADCOPTER_POSITION_H
