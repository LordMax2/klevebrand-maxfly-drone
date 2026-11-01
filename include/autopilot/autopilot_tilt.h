#ifndef AUTOPILOT_TILT_H
#define AUTOPILOT_TILT_H

#include "pid.h"
#include "template_drone.h"

inline constexpr bool AUTOPILOT_HORIZONTAL_ENABLED = false;
inline constexpr float AUTOPILOT_HOVER_ALTITUDE_OFFSET_METERS = 2.0f;
inline constexpr float AUTOPILOT_HOVER_THROTTLE_MINIMUM = 45.0f;
inline constexpr float AUTOPILOT_HOVER_THROTTLE_MAXIMUM = 75.0f;
inline constexpr float AUTOPILOT_THROTTLE_MINIMUM = 30.0f;

template <class SomeGyroPidType, DronePositionConcept SomePositionType, DroneGyroConcept SomeGyroType, HardwareProcessorConcept SomeHardwareProcessorType>
class AutopilotTilt
{
public:
    void goTo(TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType> *drone, float latitude, float longitude, float altitude);

private:
    Pid _altitude_pid = Pid(0.3f, 0.0f, 0.0f, 1.0f, 200);
    Pid _latitude_pid = Pid(0.8f, 0.05f, 0.0f, 5.0f, 200);
    Pid _longitude_pid = Pid(0.8f, 0.05f, 0.0f, 5.0f, 200);
    Pid _altitude_velocity_pid = Pid(5.0f, 0.0f, 0.0f, 25.0f, 200);
    Pid _latitude_velocity_pid = Pid(8.0f, 0.0f, 0.0f, 15.0f, 200);
    Pid _longitude_velocity_pid = Pid(8.0f, 0.0f, 0.0f, 15.0f, 200);

    float _hover_throttle = 60.0f;
    unsigned long _last_goto_timestamp = 0;

    static float getDestinationYawCompassAngle(float target_latitude, float target_longitude, float current_latitude, float current_longitude);
};

#endif
