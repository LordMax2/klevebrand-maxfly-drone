#include "klevebrand_maxfly_drone.h"
#include "pwm_receiver.h"
#include "servo_drone_motor.h"
#include "drone_pwm_receiver.h"
#include "skywire-command-startup-worker.h"
#include "skywire-command-tcp-gps-step-worker.h"

//#define SKYWIRE_EXPERIMENTAL

ServoDroneMotor motors[4];

KlevebrandMaxFlyDrone drone = KlevebrandMaxFlyDrone(motors);

#ifdef SKYWIRE_EXPERIMENTAL
SkywireTcpGpsStepWorker *worker;
#endif
DronePwmReceiver receiver = DronePwmReceiver(1, 4, 3, 2, 7);


void setup() {
    motors[0].setup(2);
    motors[1].setup(3);
    motors[2].setup(6);
    motors[3].setup(7);

    Serial3.begin(115200);

    // Startup the gyroscope and motors
    drone.setup();

#ifdef SKYWIRE_EXPERIMENTAL
    SkywireCommandStartupWorker startup_worker(&Serial3, false);

    while (!startup_worker.run()) {
    }
    Serial.println("Skywire started.");
#endif

    // Startup the reciever
    receiver.setup();
}

void loop() {
#ifdef SKYWIRE_EXPERIMENTAL
    if (worker == nullptr) {
        worker = new SkywireTcpGpsStepWorker(&Serial3, "flightcontroltower.klevebrand.se", 13000, 20000, false);
    }
#endif
    // Set drone flight values from the receiver
    receiver.setThrottleYawPitchRoll(&drone);

    // Set the flight mode of the drone from the receiver
    receiver.setFlightMode(&drone);

    // Run the drone feedback-loop
    drone.run();

#ifdef SKYWIRE_EXPERIMENTAL
    // Run the GPS controller
    char yawStr[8], pitchStr[8], rollStr[8], throttleStr[8];

    dtostrf(drone.getYaw(), 6, 1, yawStr);
    dtostrf(drone.getPitch(), 6, 1, pitchStr);
    dtostrf(drone.getRoll(), 6, 1, rollStr);
    dtostrf(drone.getThrottle(), 6, 1, throttleStr);

    char payload_to_send[128];
    snprintf_P(
        payload_to_send,
        sizeof(payload_to_send),
        PSTR("1;1337;%s;%s;%s;%s;%s;120.5;59.8586;17.6389;42.5;1013.2;2;7"),
        drone.isMotorsEnabled() ? "true" : "false",
        yawStr,
        pitchStr,
        rollStr,
        throttleStr
    );

    worker->setPayloadToSend(payload_to_send);
    worker->run();
#endif
}
