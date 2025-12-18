#include "klevebrand_maxfly_drone.h"
#include "klevebrand-skywire-tcp-client.h"
#include "pwm_receiver.h"
#include "./components/drone_pwm_receiver/drone_pwm_receiver.h"

uint8_t motor_pin_numbers[16] = { 3, 2, 7, 6};

KlevebrandMaxFlyDrone drone = KlevebrandMaxFlyDrone(motor_pin_numbers);
DronePwmReceiver receiver = DronePwmReceiver(1, 4, 3, 2, 7);
SkywireTcpClient tcpClient(Serial3, "flightcontroltower.klevebrand.se", 13000);

void setup()
{
  // Startup the gyroscope and motors
  drone.setup();

  // Startup the reciever
  receiver.setup();

  tcpClient.open();
}

void loop()
{
  // Set drone flight values from the receiver
  receiver.setThrottleYawPitchRoll(&drone);

  // Set the flight mode of the drone from the receiver 
  receiver.setFlightMode(&drone);

  // Run the drone feedback-loop
  drone.run();

  while(tcpClient.available()) 
  {
    String message = tcpClient.readString();

    Serial.println(message);
  }
}
