#include "quadcopter_position.h"
#include <math.h>

void QuadcopterPosition::setup() {
    if (!_bmp_device.begin()) {
        Serial.println("FAILED TO SETUP BMP280..");

        return;
    }

    _bmp_device.setSampling(
        Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X2,
        Adafruit_BMP280::SAMPLING_X4,
        Adafruit_BMP280::FILTER_X4,
        Adafruit_BMP280::STANDBY_MS_1
    );
}

float QuadcopterPosition::pressureToAltitudeMeters(const float pressure_pa, const float sea_level_pressure_pa) {
    return 44330.0f * (1.0f - pow(pressure_pa / sea_level_pressure_pa, 0.1903f));
}

float QuadcopterPosition::getAltitude() {
    return _altitude;
}

void QuadcopterPosition::run() {
    const unsigned long now = micros();

    if (_last_run_timestamp_microseconds != 0 && now - _last_run_timestamp_microseconds < run_interval_microseconds) {
        return;
    }

    _last_run_timestamp_microseconds = now;

    _pressure = _bmp_device.readPressure();
    _temperature = _bmp_device.readTemperature();

    if (_pressure <= 0.0f) {
        return;
    }

    _altitude = pressureToAltitudeMeters(_pressure, sea_level_pressure_pa);
}
