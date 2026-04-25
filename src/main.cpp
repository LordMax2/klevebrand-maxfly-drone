#include "klevebrand_maxfly_drone.h"
#include "servo_drone_motor.h"
#include "control_manager.h"
#include "pwm_receiver_controller.h"
#include "skywire_drone_controller.h"

//#define SKYWIRE_EXPERIMENTAL

static ServoDroneMotor motors[4];
static constexpr int motor_pins[4] = {2, 3, 6, 7};

static auto drone = KlevebrandMaxFlyDrone(motors, motor_pins);

static auto receiver = PwmReceiverController(1, 4, 3, 2, 7);
static SkywireDroneController skywire_controller;
static auto control_arbiter = ControlManager(&receiver, &skywire_controller);

void setup() {
    // Startup the gyroscope and motors
    drone.setup();

    control_arbiter.setup();
}

void loop() {
    control_arbiter.run(&drone);

    // Run the drone feedback-loop
    drone.run();
}
