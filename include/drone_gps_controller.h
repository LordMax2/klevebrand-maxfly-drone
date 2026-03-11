#ifndef DRONE_GPS_CONTROLLER_H
#define DRONE_GPS_CONTROLLER_H

#include "klevebrand_maxfly_drone.h"
#include "gps_location_info.h"
#include "pid.h"
#include "klevebrand_skywire_http_gps_step_worker.h"
#include "skywire-command-startup-worker.h"
#include "pid_yaw_compass.h"

class DroneGpsController
{
public:
    DroneGpsController(
        HardwareSerial *hardware_serial) : _skywire_http_gps_worker(hardware_serial, "flightcontroltower.klevebrand.se", 80, "api/v1/dronerequest/1337", "api/v1/drone/1337/setstate"),
                                          _start_location_info(GpsLocationInfo_t::empty()),
                                          _altitude_pid(1.0f, 0.0f, 15.0f, 50) {}

    static void setup(HardwareSerial *hardware_serial);
    void goTo(KlevebrandMaxFlyDrone *drone, float latitude, float longitude, float altitude);
    void run(KlevebrandMaxFlyDrone *drone);

private:
    SkywireHttpGpsStepWorker _skywire_http_gps_worker;
    GpsLocationInfo_t _start_location_info;
    Pid _altitude_pid;

    static float getDestinationYawCompassAngle(float target_latitude, float target_longitude, float current_latitude, float current_longitude);
};

#endif // DRONE_GPS_CONTROLLER_H