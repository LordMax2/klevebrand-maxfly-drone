#include "klevebrand_maxfly_drone.h"
#include "klevebrand-skywire-tcp-client.h"
#include "pwm_receiver.h"
#include "./components/drone_pwm_receiver/drone_pwm_receiver.h"
#include "klevebrand-skywire-http-stepper-client.h"

uint8_t motor_pin_numbers[16] = { 3, 2, 7, 6};

KlevebrandMaxFlyDrone drone = KlevebrandMaxFlyDrone(motor_pin_numbers);
DronePwmReceiver receiver = DronePwmReceiver(1, 4, 3, 2, 7);
SkywireTcpClient tcpClient(Serial3, "213.66.134.107", 13000);

SkywireHttpStepperClient httpStepperClient(Serial3, "flightcontroltower.klevebrand.se", 80);

String rx_buffer = "";

void setup()
{
  // Startup the gyroscope and motors
  drone.setup();

  // Startup the reciever
  receiver.setup();

  httpStepperClient.start();
}

void loop()
{
  // Set drone flight values from the receiver
  receiver.setThrottleYawPitchRoll(&drone);

  // Set the flight mode of the drone from the receiver 
  receiver.setFlightMode(&drone);

  // Run the drone feedback-loop
  drone.run();

  SkywireResponseResult_t result = httpStepperClient.get("api/v1/weather/coordinates?latitude=59.8586&longitude=17.6389");

  if(result.is_success)
  {
      Serial.println("HTTP GET SUCCESS:");
      Serial.println(result.response_content);
  }

  Serial.println("Loop complete");
}
