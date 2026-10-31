#pragma once

#include "Arduino.h"
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include "yaw_pitch_roll.h"
#include "concept_drone_gyro.h"
#include "imxrt_gpio_pad.h"

/*
 * The BNO085 on SPI1, with the same modes and readings as Bno08xDroneGyro from klevebrand-drone-gyro-bno085, which
 * only speaks I2C. The board straps the BNO085 for SPI (PS0/WAKE and PS1 high).
 */
class Bno08xSpiDroneGyro
{
public:
    // The reset line is on pad GPIO_EMC_<reset_pad>, which has no Teensy pin number.
    explicit Bno08xSpiDroneGyro(const int reset_pad) : _reset_pad(ImxrtGpioPad::emc(reset_pad))
    {
    }

    void setup();
    bool reload();
    void reset();
    float yaw();
    float pitch();
    float roll();
    float accelerationX();
    float accelerationY();
    float accelerationZ();
    void printYawPitchRoll();
    bool setModeAcro();
    bool setModeEuler();
    bool setModeEulerAndAcceleration();
    unsigned long timestampMilliseconds();

private:
    static constexpr uint8_t chip_select_pin = 38;
    static constexpr uint8_t interrupt_pin = 41;
    static constexpr uint8_t miso_pin = 39;
    static constexpr uint8_t wake_pad = 11;

    const ImxrtGpioPad _reset_pad;

    // No reset pin for the Adafruit driver: reset() drives the pad itself.
    Adafruit_BNO08x _gyro;
    sh2_SensorValue_t _sensor_value;
    YawPitchRoll_t _yaw_pitch_roll{};
    sh2_Accelerometer_t _acceleration = {0.0f, 0.0f, 0.0f};
    sh2_RotationVectorWAcc_t _rotational_vector = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    sh2_Accelerometer_t _rotated_acceleration = {0.0f, 0.0f, 0.0f};
    enum Mode
    {
        MODE_NONE,
        MODE_ACRO,
        MODE_EULER,
        MODE_EULER_AND_ACCELERATION
    };
    Mode _mode = MODE_NONE;

    static YawPitchRoll_t quaternionsToYawPitchRoll(const sh2_RotationVectorWAcc_t *rotational_vector,
                                                    bool degrees = false);
    static YawPitchRoll_t quaternionsToYawPitchRoll(float qr, float qi, float qj, float qk, bool degrees = false);
    static void rotateVectorByQuaternion(float &x, float &y, float &z, float r, float i, float j, float k);
};

static_assert(DroneGyroConcept<Bno08xSpiDroneGyro>, "Bno08xSpiDroneGyro does not meet DroneGyroConcept");
