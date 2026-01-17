#ifndef KLEVEBRAND_FLIGHT_CONTROL_TOWER_CLIENT_H
#define KLEVEBRAND_FLIGHT_CONTROL_TOWER_CLIENT_H

#include "klevebrand-skywire-http-stepper-client.h"
#include "drone_request_dto.h"

#define KLEVEBRAND_FLIGHT_CONTROL_TOWER_CLIENT_DEFAULT_BASE_URL "flightcontroltower.klevebrand.se"
#define KLEVEBRAND_FLIGHT_CONTROL_TOWER_CLIENT_DEFAULT_PORT 80

class KlevebrandFlightControlTowerClient {
public:
    KlevebrandFlightControlTowerClient(HardwareSerial& skywire_serial)
        : http_stepper_client(skywire_serial, KLEVEBRAND_FLIGHT_CONTROL_TOWER_CLIENT_DEFAULT_BASE_URL, KLEVEBRAND_FLIGHT_CONTROL_TOWER_CLIENT_DEFAULT_PORT) {};

    void setup();

    DroneRequestDto getDroneRequest(String serial_number);
private:
    SkywireHttpStepperClient http_stepper_client;
};


#endif // KLEVEBRAND_FLIGHT_CONTROL_TOWER_CLIENT_H