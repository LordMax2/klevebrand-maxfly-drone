#pragma once

#include "Arduino.h"
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include "yaw_pitch_roll.h"
#include "concept_drone_gyro.h"
#include "concept_digital_output.h"

/*
 * The BNO085 on SPI, with the same modes and readings as Bno08xDroneGyro from klevebrand-drone-gyro-bno085, which
 * only speaks I2C. The board must strap PS1 high; setup() holds PS0/WAKE high through the reset to select SPI.
 * Configure the bus pins (MISO, MOSI, SCK) before setup(), which starts the bus.
 */
template <DigitalOutputConcept SomeDigitalOutputType>
class Bno08xSpiDroneGyro
{
public:
    Bno08xSpiDroneGyro(SPIClass &spi, const uint8_t chip_select_pin, const uint8_t interrupt_pin,
                       const SomeDigitalOutputType reset_line, const SomeDigitalOutputType wake_line)
        : _spi(spi), _chip_select_pin(chip_select_pin), _interrupt_pin(interrupt_pin), _reset_line(reset_line),
          _wake_line(wake_line)
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
    static constexpr unsigned long reset_pulse_milliseconds = 10;

    SPIClass &_spi;
    const uint8_t _chip_select_pin;
    const uint8_t _interrupt_pin;
    const SomeDigitalOutputType _reset_line;
    const SomeDigitalOutputType _wake_line;

    // No reset pin for the Adafruit driver: reset() drives the reset line itself.
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

#include "bno08x_spi_drone_gyro.ipp"
