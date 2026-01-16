#ifndef DRONE_GPS_CONTROLLER_H
#define DRONE_GPS_CONTROLLER_H

#include <components/klevebrand-skywire-gps-stepper-client/klevebrand-skywire-gps-stepper-client.h>
#include "./entities/gps_location_info.h"
#include "../../klevebrand_maxfly_drone.h"

class DroneGpsController
{
public:
    DroneGpsController(HardwareSerial &hardwareSerial, KlevebrandMaxFlyDrone &drone) : skywire_gps_client(hardwareSerial), _start_location_info(GpsLocationInfo_t::empty()), _drone(drone) {}

    void setup();
    void goTo(float latitude, float longitude, float altitude);

private:
    KlevebrandSkywireGpsStepperClient skywire_gps_client;
    GpsLocationInfo_t _start_location_info;
    KlevebrandMaxFlyDrone &_drone;

};

#endif // DRONE_GPS_CONTROLLER_H