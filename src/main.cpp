#include "klevebrand_maxfly_drone.h"
#include "pwm_receiver.h"
#include "servo_drone_motor.h"
#include "drone_pwm_receiver.h"
#include "klevebrand_skywire_gps_stepper_client.h"
#include "klevebrand_flight_control_tower_client.h"

ServoDroneMotor motors[4] = {
    ServoDroneMotor::getServoDroneMotor(3),
    ServoDroneMotor::getServoDroneMotor(2),
    ServoDroneMotor::getServoDroneMotor(6),
    ServoDroneMotor::getServoDroneMotor(7),
};

KlevebrandMaxFlyDrone drone = KlevebrandMaxFlyDrone(motors);
DronePwmReceiver receiver = DronePwmReceiver(1, 4, 3, 2, 7);

KlevebrandSkywireGpsStepperClient skywireGpsStepperClient = KlevebrandSkywireGpsStepperClient(Serial3);
KlevebrandFlightControlTowerClient flightControlTowerClient = KlevebrandFlightControlTowerClient(Serial3);

void setup()
{
  // Startup the gyroscope and motors
  drone.setup();

  //skywireGpsStepperClient.start();
  flightControlTowerClient.setup();

  bool lockedGps = true;
  bool lockedTower = false;

  while(true)
  {
      DroneRequestDto tower_response = flightControlTowerClient.getDroneRequest("1337");

      if(tower_response.is_valid) {
        Serial.print("Flight Control Tower Response: ");
        Serial.print(" Flight Mode ID: ");
        Serial.print(tower_response.flight_mode_id);
        Serial.print(" Throttle: ");
        Serial.println(tower_response.throttle);
    }
  }

  // Startup the reciever
  receiver.setup();
}

void loop()
{
  // Set drone flight values from the receiver
  receiver.setThrottleYawPitchRoll(&drone);

  // Set the flight mode of the drone from the receiver
  receiver.setFlightMode(&drone);

  // Run the drone feedback-loop
  drone.run();
}
