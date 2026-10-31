#include "klevebrand_maxfly_drone.h"
#include "drone_components/pwm_drone_motor.h"
#include "drone_components/status_leds.h"
#include "receivers/control_manager.h"
#include "receivers/pwm_receiver_controller.h"
#include "receivers/skywire_drone_controller.h"

// OUT1-OUT4, ESC connector pins 6, 5, 4 and 3: left front, right front, left back, right back.
static PwmDroneMotor motors[4];
static constexpr int motor_pins[4] = {2, 3, 4, 33};

static KlevebrandMaxFlyDrone drone(motors, motor_pins);

using PwmReceiverControllerType = PwmReceiverController<PwmReceiverControlModeNone, PwmReceiverControlModeAcro, PwmReceiverControlModeAutoLevel>;

static auto receiver = PwmReceiverControllerType(1, 4, 3, 2, 7, 8);
static SkywireDroneController skywire_controller;
static auto control_manager = ControlManager(&receiver, &skywire_controller);

void setup() {
    StatusLeds::showStarting();

    // Startup the gyroscope and motors
    drone.setup();

    control_manager.setup();

    StatusLeds::showReady();
}

void loop() {
    control_manager.run(&drone);

    // Run the drone feedback-loop
    drone.run();
}
