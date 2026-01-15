#include "klevebrand_maxfly_drone.h"
#include "pwm_receiver.h"
#include "servo_drone_motor.h"
#include "./components/drone_pwm_receiver/drone_pwm_receiver.h"

ServoDroneMotor motors[4] = {
    ServoDroneMotor::getServoDroneMotor(3),
    ServoDroneMotor::getServoDroneMotor(2),
    ServoDroneMotor::getServoDroneMotor(6),
    ServoDroneMotor::getServoDroneMotor(7),
};

KlevebrandMaxFlyDrone drone = KlevebrandMaxFlyDrone(motors);
DronePwmReceiver receiver = DronePwmReceiver(1, 4, 3, 2, 7);

void setup()
{
  // Startup the gyroscope and motors
  drone.setup();

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
