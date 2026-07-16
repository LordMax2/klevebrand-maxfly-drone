#include "klevebrand_maxfly_drone.h"
#include "drone_components/servo_drone_motor.h"
#include "receivers/control_manager.h"
#include "receivers/pwm_receiver_controller.h"
#include "receivers/skywire_drone_controller.h"

static ServoDroneMotor motors[4];
static constexpr int motor_pins[4] = {2, 3, 6, 7};

static KlevebrandMaxFlyDrone drone(motors, motor_pins);

using PwmReceiverControllerType = PwmReceiverController<PwmReceiverControlModeNone, PwmReceiverControlModeAcro, PwmReceiverControlModeAutoLevel>;

static auto receiver = PwmReceiverControllerType(1, 4, 3, 2, 7);
static SkywireDroneController skywire_controller;
static auto control_manager = ControlManager(&receiver, &skywire_controller);

void setup() {
    // Startup the gyroscope and motors
    drone.setup();

    control_manager.setup();
}

void loop() {
    control_manager.run(&drone);

    // Run the drone feedback-loop
    drone.run();
}
