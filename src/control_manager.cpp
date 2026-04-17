#include "control_manager.h"

void ControlManager::setup()
{
    if (_pwm_receiver != nullptr)
    {
        _pwm_receiver->setup();
    }

    if (_skywire_controller != nullptr)
    {
        _skywire_controller->setup();
    }
}

void ControlManager::run(KlevebrandMaxFlyDrone *drone)
{
    if (_skywire_controller != nullptr)
    {
        _skywire_controller->run(drone);
    }

    if (_pwm_receiver != nullptr && _pwm_receiver->wantsControl())
    {
        _active_control_source = ActiveControlSource_t::pwm;
        _pwm_receiver->apply(drone);
        return;
    }

    if (_skywire_controller != nullptr && _skywire_controller->wantsControl())
    {
        _active_control_source = ActiveControlSource_t::skywire;
        _skywire_controller->apply(drone);
        return;
    }

    _active_control_source = ActiveControlSource_t::none;
}

ActiveControlSource_t ControlManager::activeControlSource() const
{
    return _active_control_source;
}
