#ifndef AUTOPILOT_TILT_H
#define AUTOPILOT_TILT_H

#include "pid.h"
#include "template_drone.h"

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
class AutopilotTilt
{
public:
    void goTo(TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType> *drone, float latitude, float longitude, float altitude);

private:
    Pid _altitude_pid = Pid(1.3f, 0.05f, 0.0f, 2.0f, 200);
    Pid _latitude_pid = Pid(0.5f, 0.05f, 0.0f, 5.0f, 200);
    Pid _longitude_pid = Pid(0.5f, 0.05f, 0.0f, 5.0f, 200);
    Pid _altitude_velocity_pid = Pid(3.0f, 0.0f, 0.0f, 20.0f, 200);
    Pid _latitude_velocity_pid = Pid(3.0f, 0.0f, 0.0f, 15.0f, 200);
    Pid _longitude_velocity_pid = Pid(3.0f, 0.0f, 0.0f, 15.0f, 200);

    float _hover_throttle = 60.0f;

    static float getDestinationYawCompassAngle(float target_latitude, float target_longitude, float current_latitude, float current_longitude);
};

#endif
