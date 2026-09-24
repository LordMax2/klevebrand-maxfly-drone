#include "receivers/skywire_drone_controller.h"

#include "autopilot/autopilot_tilt.h"
#include "drone_components/flight_mode_acro_local.h"
#include "drone_components/flight_mode_auto_level_local.h"

#ifdef SKYWIRE_EXPERIMENTAL
#include "skywire-command-startup-worker.h"

/**
 *
 */
SkywireDroneController::SkywireDroneController()
    : _worker(&Serial3, "flightcontroltower.klevebrand.se", 13000, 20000, false)
{
}

#else
SkywireDroneController::SkywireDroneController() = default;

#endif

void SkywireDroneController::setup()
{
#ifdef SKYWIRE_EXPERIMENTAL
    Serial3.begin(115200);

    SkywireCommandStartupWorker<receive_buffer_size> startup_worker(&Serial3, false);

    while (!startup_worker.run())
    {
    }

    Serial.println("Skywire started.");
#endif
}

void SkywireDroneController::run(KlevebrandMaxFlyDrone *drone)
{
#ifdef SKYWIRE_EXPERIMENTAL
    if (drone == nullptr)
    {
        return;
    }

    char yawStr[8], pitchStr[8], rollStr[8], throttleStr[8];
    char altitudeStr[12], latitudeStr[16], longitudeStr[16], temperatureStr[12], pressureStr[12];

    dtostrf(drone->getYaw(), 6, 1, yawStr);
    dtostrf(drone->getPitch(), 6, 1, pitchStr);
    dtostrf(drone->getRoll(), 6, 1, rollStr);
    dtostrf(drone->getThrottle(), 6, 1, throttleStr);
    dtostrf(drone->getAltitude(), 6, 2, altitudeStr);
    dtostrf(drone->getLatitude(), 10, 6, latitudeStr);
    dtostrf(drone->getLongitude(), 10, 6, longitudeStr);
    dtostrf(drone->position.getTemperature(), 6, 1, temperatureStr);
    dtostrf(drone->position.getPressure(), 8, 1, pressureStr);

    char payload_to_send[128];
    snprintf_P(
        payload_to_send,
        sizeof(payload_to_send),
        PSTR("1;1337;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%d;0"),
        drone->isMotorsEnabled() ? "true" : "false",
        yawStr,
        pitchStr,
        rollStr,
        throttleStr,
        altitudeStr,
        latitudeStr,
        longitudeStr,
        temperatureStr,
        pressureStr,
        static_cast<int>(drone->getControlMode())
    );

    _worker.setPayloadToSend(payload_to_send);
    _worker.run();

    setRequest(_worker.getLatestDroneRequest());
    requestControl();
#else
    (void)drone;
#endif
}

void SkywireDroneController::setRequest(const DroneRequest_t &request)
{
    _request = request;
    _has_pending_request = true;
}

void SkywireDroneController::clearRequest()
{
    _request = DroneRequest_t::empty();
    _has_pending_request = false;
}

void SkywireDroneController::requestControl()
{
    _wants_control = true;
}

void SkywireDroneController::releaseControl()
{
    _wants_control = false;
}

bool SkywireDroneController::wantsControl() const
{
    return _wants_control && _has_pending_request;
}

void SkywireDroneController::apply(KlevebrandMaxFlyDrone *drone) const
{
    if (drone == nullptr || !_has_pending_request)
    {
        return;
    }

    if (_request.enable_motors)
    {
        drone->enableMotors();
    }

    if (_request.flight_mode_id == auto_level)
    {
        static auto auto_level_local = FlightModeAutoLevelLocal();
        drone->activateControlMode(&auto_level_local);
    }
    else if (_request.flight_mode_id == acro)
    {
        static auto acro_local = FlightModeAcroLocal();
        drone->activateControlMode(&acro_local);
    }

    if (AUTOPILOT_HORIZONTAL_ENABLED && drone->isAutopilotEnabled() &&
        (_request.latitude != 0.0f || _request.longitude != 0.0f))
    {
        drone->setAutopilotHorizontalTarget(_request.latitude, _request.longitude);
    }

    if (drone->isAutopilotEnabled() && _request.hasCommandedAltitude())
    {
        drone->setAutopilotAltitudeTarget(_request.assignment_altitude);
    }
}
