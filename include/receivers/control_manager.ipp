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
        _skywire_controller->run(drone);
    }

    if (_pwm_receiver_controller != nullptr)
    {
        _pwm_receiver_controller->applyAutopilot(drone);
        _pwm_receiver_controller->apply(drone);
        _active_controller = ActiveController_t::pwm;
    }
    else
    {
        _active_controller = ActiveController_t::none;
    }

    if (_skywire_controller != nullptr && _skywire_controller->wantsControl())
    {
        _skywire_controller->apply(drone);
    }
}

template <class PwmReceiverControllerType>
ActiveController_t ControlManager<PwmReceiverControllerType>::activeControlSource() const
{
    return _active_controller;
}
