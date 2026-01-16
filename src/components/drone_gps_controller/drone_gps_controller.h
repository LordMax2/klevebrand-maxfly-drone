#ifndef DRONE_GPS_CONTROLLER_H
#define DRONE_GPS_CONTROLLER_H

#include <components/klevebrand_skywire_gps_stepper_client/klevebrand_skywire_gps_stepper_client.h>
#include "./entities/gps_location_info.h"
#include "../../klevebrand_maxfly_drone.h"
#include "../throttle_altitude_pid/throttle_altitude_pid.h"

class DroneGpsController
{
public:
    DroneGpsController(HardwareSerial &hardwareSerial, KlevebrandMaxFlyDrone &drone) : skywire_gps_client(hardwareSerial), _start_location_info(GpsLocationInfo_t::empty()), _drone(drone), _altitude_pid(0.1f, 0.001f, 0.1f, 50) {}

    void setup();
    void goTo(float latitude, float longitude, float altitude);

private:
    KlevebrandSkywireGpsStepperClient skywire_gps_client;
    GpsLocationInfo_t _start_location_info;
    KlevebrandMaxFlyDrone &_drone;
    ThrottleAltitudePid _altitude_pid;

    float yawError(float gyro_yaw, float yaw_desired_angle);
    float getDestinationYawCompassAngle(float target_latitude, float target_longitude, float current_latitude, float current_longitude);

};

#endif // DRONE_GPS_CONTROLLER_H