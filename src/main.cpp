#include "klevebrand_maxfly_drone.h"
#include "pwm_receiver.h"
#include "servo_drone_motor.h"
#include "./components/drone_pwm_receiver/drone_pwm_receiver.h"
#include "./components/klevebrand_flight_control_tower_client/klevebrand_flight_control_tower_client.h"

const int motor_pins[] = {3, 2, 6, 7};

Servo motor_servos[4];

ServoDroneMotor motors[4] = {
  ServoDroneMotor(motor_servos[0]),
  ServoDroneMotor(motor_servos[1]),
  ServoDroneMotor(motor_servos[2]),
  ServoDroneMotor(motor_servos[3])
};

KlevebrandMaxFlyDrone drone = KlevebrandMaxFlyDrone(motors);
DronePwmReceiver receiver = DronePwmReceiver(1, 4, 3, 2, 7);
KlevebrandFlightControlTowerClient http_stepper_client(Serial3);

void setup()
{
  // Attach the motors before starting the drone, this couldnt be done above in the "global" scope, so has to be done here
  for (int i = 0; i < 4; i++) {
    motor_servos[i].attach(motor_pins[i]);
  }

  // Startup the gyroscope and motors
  drone.setup();

  // Startup the reciever
  receiver.setup();

  // Setup 4g LTE modem
  http_stepper_client.setup();
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
  DroneRequestDto drone_request = http_stepper_client.getDroneRequest("1339");

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
