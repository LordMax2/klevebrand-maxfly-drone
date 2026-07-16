#pragma once

#include "concepts.h"
#include "concept_control_mode.h"
#include "klevebrand_maxfly_drone.h"

template <class T>
concept ConceptPwmReceiverControlMode = requires(T pwm_receiver_control_mode, KlevebrandMaxFlyDrone* drone, int throttle_pwm, int yaw_pwm, int pitch_pwm, int roll_pwm)
{
    { pwm_receiver_control_mode.controlModeType() } -> same_as<ControlMode_t>;
    { pwm_receiver_control_mode.applyThrottleYawPitchRoll(drone, throttle_pwm, yaw_pwm, pitch_pwm, roll_pwm) } -> same_as<void>;
};

