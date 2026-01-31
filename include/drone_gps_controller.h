#ifndef DRONE_GPS_CONTROLLER_H
#define DRONE_GPS_CONTROLLER_H

#include "klevebrand_maxfly_drone.h"
#include "gps_location_info.h"
#include "throttle_altitude_pid.h"
#include "klevebrand-skywire-http-gps-step-worker.h"

class DroneGpsController
{
public:
    DroneGpsController(HardwareSerial &hardwareSerial, KlevebrandMaxFlyDrone &drone) : 
        skywire_http_gps_step_worker(hardwareSerial, "flightcontroltower.klevebrand.se", 80, "api/v1/drone/1337"),
        _start_location_info(GpsLocationInfo_t::empty()), 
        _drone(drone), 
        _altitude_pid(1.0f, 0.0f, 15.0f, 50) {}

    void setup();
    void goTo(float latitude, float longitude, float altitude);
    void run();

private:
    
    SkywireHttpGpsStepWorker skywire_http_gps_step_worker;
    GpsLocationInfo_t _start_location_info;
    KlevebrandMaxFlyDrone &_drone;
    ThrottleAltitudePid _altitude_pid;

    float yawError(float gyro_yaw, float yaw_desired_angle);
    float getDestinationYawCompassAngle(float target_latitude, float target_longitude, float current_latitude, float current_longitude);

};

#endif // DRONE_GPS_CONTROLLER_H