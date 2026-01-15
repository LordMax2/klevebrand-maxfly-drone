#include <components/klevebrand-skywire-gps-stepper-client/klevebrand-skywire-gps-stepper-client.h>
#ifndef DRONE_GPS_CONTROLLER_H
#define DRONE_GPS_CONTROLLER_H

class DroneGpsController
{
public:
    DroneGpsController(HardwareSerial &hardwareSerial) : skywire_gps_client(hardwareSerial) {}
    void setup();

private:
    KlevebrandSkywireGpsStepperClient skywire_gps_client;

    
};

#endif // DRONE_GPS_CONTROLLER_H