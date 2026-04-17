#include "klevebrand_maxfly_drone.h"
#include "pwm_receiver.h"
#include "servo_drone_motor.h"
#include "control_manager.h"
#include "pwm_receiver_controller.h"
#include "skywire_drone_controller.h"

//#define SKYWIRE_EXPERIMENTAL

ServoDroneMotor motors[4];

auto drone = KlevebrandMaxFlyDrone(motors);

auto receiver = PwmReceiverController(1, 4, 3, 2, 7);
SkywireDroneController skywire_controller;
auto control_arbiter = ControlManager(&receiver, &skywire_controller);

void setup() {
    motors[0].setup(2);
    motors[1].setup(3);
    motors[2].setup(6);
    motors[3].setup(7);

    // Startup the gyroscope and motors
    drone.setup();

    control_arbiter.setup();
}

void loop() {
    control_arbiter.run(&drone);

    // Run the drone feedback-loop
    drone.run();
}
