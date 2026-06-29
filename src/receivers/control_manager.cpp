#include "../../include/receivers/control_manager.h"

void ControlManager::setup() const
{
    if (_pwm_receiver_controller != nullptr)
    {
        _pwm_receiver_controller->setup();
    }

    if (_skywire_controller != nullptr)
    {
        SkywireDroneController::setup();
    }
}

void ControlManager::run(KlevebrandMaxFlyDrone *drone)
{
    if (_skywire_controller != nullptr)
    {
        SkywireDroneController::run(drone);
    }

    if (_pwm_receiver_controller != nullptr && _pwm_receiver_controller->wantsControl())
    {
        _active_controller = ActiveController_t::pwm;
        _pwm_receiver_controller->apply(drone);
        return;
    }

    if (_skywire_controller != nullptr && _skywire_controller->wantsControl())
    {
        _active_controller = ActiveController_t::skywire;
        _skywire_controller->apply(drone);
        return;
    }

    _active_controller = ActiveController_t::none;
}

ActiveController_t ControlManager::activeControlSource() const
{
    return _active_controller;
}
