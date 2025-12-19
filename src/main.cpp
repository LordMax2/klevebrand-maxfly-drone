#include "klevebrand_maxfly_drone.h"
#include "pwm_receiver.h"
#include "./components/drone_pwm_receiver/drone_pwm_receiver.h"
#include "./components/klevebrand_flight_control_tower_client/klevebrand_flight_control_tower_client.h"

uint8_t motor_pin_numbers[16] = { 3, 2, 7, 6};

KlevebrandMaxFlyDrone drone = KlevebrandMaxFlyDrone(motor_pin_numbers);
DronePwmReceiver receiver = DronePwmReceiver(1, 4, 3, 2, 7);
KlevebrandFlightControlTowerClient httpStepperClient(Serial3);


String rx_buffer = "";

void setup()
{
  // Startup the gyroscope and motors
  drone.setup();

  // Startup the reciever
  receiver.setup();

  // Setup 4g LTE modem
  httpStepperClient.setup();
}

void loop()
{
  // Set drone flight values from the receiver
  receiver.setThrottleYawPitchRoll(&drone);

  // Set the flight mode of the drone from the receiver 
  receiver.setFlightMode(&drone);

  // Run the drone feedback-loop
  drone.run();

  // Get flight controller instructions
  DroneRequestDto drone_request = httpStepperClient.getDroneRequest("1339");

  if(drone_request.is_valid)
  {
    Serial.println("Received valid drone request from flight control tower");

    Serial.print("Throttle: ");
    Serial.print(drone_request.throttle);
    Serial.print(", Yaw: ");
    Serial.print(drone_request.yaw);
    Serial.print(", Pitch: ");
    Serial.print(drone_request.pitch);
    Serial.print(", Roll: ");
    Serial.print(drone_request.roll);
    Serial.print(", Enable Power: ");
    Serial.print(drone_request.enable_power);
    Serial.print(", Enable Motors: ");
    Serial.println(drone_request.enable_motors);
  }
}
