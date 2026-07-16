template <class PwmReceiverControllerType>
void ControlManager<PwmReceiverControllerType>::setup() const
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

template <class PwmReceiverControllerType>
void ControlManager<PwmReceiverControllerType>::run(KlevebrandMaxFlyDrone *drone)
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

template <class PwmReceiverControllerType>
ActiveController_t ControlManager<PwmReceiverControllerType>::activeControlSource() const
{
    return _active_controller;
}
