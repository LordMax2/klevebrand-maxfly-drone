#pragma once

#include <SPI.h>
#include "bno08x_spi_drone_gyro.h"
#include "imxrt_gpio_pad.h"

/*
 * The BNO085 as the board wires it: SPI1 with MISO on pin 39, chip select 38 and interrupt 41. The reset line is pad
 * GPIO_EMC_<reset_pad> and PS0/WAKE is pad GPIO_EMC_11, which have no Teensy pin numbers. The board straps the BNO085
 * for SPI (PS0/WAKE and PS1 high).
 */
class MaxFlyGyro : public Bno08xSpiDroneGyro<ImxrtGpioPad>
{
    using LibraryGyro = Bno08xSpiDroneGyro<ImxrtGpioPad>;

    static constexpr uint8_t chip_select_pin = 38;
    static constexpr uint8_t interrupt_pin = 41;
    static constexpr uint8_t miso_pin = 39;
    static constexpr uint8_t wake_pad = 11;

public:
    explicit MaxFlyGyro(const int reset_pad)
        : LibraryGyro(SPI1, chip_select_pin, interrupt_pin, ImxrtGpioPad::emc(reset_pad), ImxrtGpioPad::emc(wake_pad))
    {
    }

    void setup()
    {
        SPI1.setMISO(miso_pin);

        LibraryGyro::setup();
    }
};

static_assert(DroneGyroConcept<MaxFlyGyro>, "MaxFlyGyro does not meet DroneGyroConcept");
