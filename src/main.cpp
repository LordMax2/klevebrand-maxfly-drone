#include <SPI.h>
#include "klevebrand_maxfly_drone.h"
#include "imxrt_gpio_pad.h"
#include "drone_components/pwm_drone_motor.h"
#include "drone_components/status_leds.h"
#include "receivers/control_manager.h"
#include "receivers/pwm_receiver_controller.h"
#include "receivers/skywire_drone_controller.h"

/*
 * The board's wiring: every pin, pad and serial port is set here and nowhere else. Numbers are Teensy 4.1 pins; pads
 * without one go by their GPIO_EMC or GPIO_B1 number.
 */

// The BNO085 on SPI1, with MISO on SPI1's alternate pin. Its reset line and PS0/WAKE are GPIO_EMC pads.
static constexpr uint8_t gyro_chip_select_pin = 38;
static constexpr uint8_t gyro_interrupt_pin = 41;
static constexpr uint8_t gyro_miso_pin = 39;
static constexpr uint8_t gyro_reset_pad = 10;
static constexpr uint8_t gyro_wake_pad = 11;

static MaxFlyGyro gyro(SPI1, gyro_chip_select_pin, gyro_interrupt_pin, ImxrtGpioPad::emc(gyro_reset_pad),
                       ImxrtGpioPad::emc(gyro_wake_pad));

// The NEO-M9N on Serial1 and the BMP581 on the default SPI bus.
static constexpr uint8_t barometer_chip_select_pin = 32;

static MaxFlyPosition position(&gyro, Serial1, barometer_chip_select_pin);

// The ICM-45686 shares the BMP581's SPI bus and has no driver yet, so its chip select must stay high.
static constexpr uint8_t icm45686_chip_select_pin = 10;

// OUT1-OUT4, ESC connector pins 6, 5, 4 and 3: left front, right front, left back, right back. OUT_EN_N of the output
// buffers is a GPIO_EMC pad.
static PwmDroneMotor motors[4];
static constexpr int motor_pins[4] = {2, 3, 4, 33};
static constexpr uint8_t output_enable_pad = 33;

static KlevebrandMaxFlyDrone drone(gyro, position, motors, motor_pins, ImxrtGpioPad::emc(output_enable_pad));

using PwmReceiverControllerType = PwmReceiverController<PwmReceiverControlModeNone, PwmReceiverControlModeAcro, PwmReceiverControlModeAutoLevel>;

// RC IN 1-8 are pads GPIO_B1_04 to GPIO_B1_11.
static constexpr uint8_t receiver_first_channel_pad = 4;

static auto receiver = PwmReceiverControllerType(receiver_first_channel_pad, 1, 4, 3, 2, 7, 8);

// The Skywire on Serial2. RTS, MODEM_ON and MODEM_PWR_EN are GPIO_EMC pads.
static constexpr uint8_t modem_request_to_send_pad = 13;
static constexpr uint8_t modem_on_pad = 16;
static constexpr uint8_t modem_power_enable_pad = 30;

static SkywireDroneController skywire_controller(Serial2, ImxrtGpioPad::emc(modem_request_to_send_pad),
                                                 ImxrtGpioPad::emc(modem_on_pad),
                                                 ImxrtGpioPad::emc(modem_power_enable_pad));
static auto control_manager = ControlManager(&receiver, &skywire_controller);

// The status and error LEDs are GPIO_EMC pads.
static constexpr uint8_t status_led_pad = 20;
static constexpr uint8_t error_led_pad = 21;

static const StatusLeds status_leds(ImxrtGpioPad::emc(status_led_pad), ImxrtGpioPad::emc(error_led_pad));

void setup() {
    pinMode(icm45686_chip_select_pin, OUTPUT);
    digitalWrite(icm45686_chip_select_pin, HIGH);

    SPI1.setMISO(gyro_miso_pin);

    status_leds.showStarting();

    // Startup the gyroscope and motors
    drone.setup();

    control_manager.setup();

    status_leds.showReady();
}

void loop() {
    control_manager.run(&drone);

    // Run the drone feedback-loop
    drone.run();
}
