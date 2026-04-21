#include "skywire_drone_controller.h"

#include "flight_mode_acro_local.h"
#include "flight_mode_auto_level_local.h"

#ifdef SKYWIRE_EXPERIMENTAL
#include "skywire-command-startup-worker.h"
#include "skywire-command-tcp-gps-step-worker.h"
#endif

void SkywireDroneController::setup()
{
#ifdef SKYWIRE_EXPERIMENTAL
    Serial3.begin(115200);

    SkywireCommandStartupWorker startup_worker(&Serial3, false);

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

    if (_worker == nullptr)
    {
        _worker = new SkywireTcpGpsStepWorker(&Serial3, "flightcontroltower.klevebrand.se", 13000, 20000, false);
    }

    char yawStr[8], pitchStr[8], rollStr[8], throttleStr[8];

    dtostrf(drone->getYaw(), 6, 1, yawStr);
    dtostrf(drone->getPitch(), 6, 1, pitchStr);
    dtostrf(drone->getRoll(), 6, 1, rollStr);
    dtostrf(drone->getThrottle(), 6, 1, throttleStr);

    char payload_to_send[128];
    snprintf_P(
        payload_to_send,
        sizeof(payload_to_send),
        PSTR("1;1337;%s;%s;%s;%s;%s;120.5;59.8586;17.6389;42.5;1013.2;2;7"),
        drone->isMotorsEnabled() ? "true" : "false",
        yawStr,
        pitchStr,
        rollStr,
        throttleStr
    );

    _worker->setPayloadToSend(payload_to_send);
    _worker->run();
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

bool SkywireDroneController::hasPendingRequest() const
{
    return _has_pending_request;
}

bool SkywireDroneController::wantsControl() const
{
    return _wants_control && _has_pending_request;
}

void SkywireDroneController::apply(KlevebrandMaxFlyDrone *drone)
{
    if (drone == nullptr || !_has_pending_request)
    {
        return;
    }

    if (_request.enable_motors)
    {
        drone->enableMotors();
    }
    else
    {
        drone->disableMotors();
    }

    if (_request.flight_mode_id == auto_level)
    {
        static auto auto_level_local = FlightModeAutoLevelLocal();
        drone->activateFlightMode(&auto_level_local);
    }
    else if (_request.flight_mode_id == acro)
    {
        static auto acro_local = FLightModeAcroLocal();
        drone->activateFlightMode(&acro_local);
    }

    // Mission coordinates are intentionally not applied yet.
}
