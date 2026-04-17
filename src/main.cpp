#include "klevebrand_maxfly_drone.h"
#include "pwm_receiver.h"
#include "servo_drone_motor.h"
#include "control_manager.h"
#include "pwm_receiver_controller.h"
#include "skywire_drone_controller.h"

//#define SKYWIRE_EXPERIMENTAL

ServoDroneMotor motors[4];
constexpr int motor_pins[4] = {2, 3, 6, 7};

auto drone = KlevebrandMaxFlyDrone(motors, motor_pins);

auto receiver = PwmReceiverController(1, 4, 3, 2, 7);
SkywireDroneController skywire_controller;
auto control_arbiter = ControlManager(&receiver, &skywire_controller);

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
