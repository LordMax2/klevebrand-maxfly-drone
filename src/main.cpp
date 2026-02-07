#include "klevebrand_maxfly_drone.h"
#include "pwm_receiver.h"
#include "servo_drone_motor.h"
#include "drone_pwm_receiver.h"
#include "drone_gps_controller.h"
#include "bno08x_drone_gyro.h"

ServoDroneMotor motors[4];

KlevebrandMaxFlyDrone drone = KlevebrandMaxFlyDrone(motors);
DroneGpsController gps_controller = DroneGpsController(Serial3, drone);
DronePwmReceiver receiver = DronePwmReceiver(1, 4, 3, 2, 7);

void setup()
{
  motors[0].setup(3);
  motors[1].setup(2);
  motors[2].setup(7);
  motors[3].setup(6);

  // Startup the gyroscope and motors
  drone.setup();
  //gps_controller.setup();

  //while (true)
  //{
  //  gps_controller.run();
  //}

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
