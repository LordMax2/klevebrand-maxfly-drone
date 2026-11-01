#include "receivers/skywire_drone_controller.h"

#include "autopilot/autopilot_tilt.h"
#include "drone_components/flight_mode_acro_local.h"
#include "drone_components/flight_mode_auto_level_local.h"

#ifdef SKYWIRE_EXPERIMENTAL
#include "skywire-command-startup-worker.h"
#endif

static constexpr unsigned long MODEM_SUPPLY_SETTLE_MILLISECONDS = 100;

// NL-SW-LTE-TC1WWG datasheet: hold ON_OFF low for 1 to 2 s to start the modem (2.5 s shuts it down).
static constexpr unsigned long MODEM_ON_OFF_PULSE_MILLISECONDS = 1500;

/**
 *
 */
SkywireDroneController::SkywireDroneController(HardwareSerial &modem_serial, const ImxrtGpioPad &request_to_send_line,
                                               const ImxrtGpioPad &on_line, const ImxrtGpioPad &power_enable_line)
    : _modem_serial(modem_serial), _request_to_send_line(request_to_send_line), _on_line(on_line),
      _power_enable_line(power_enable_line)
#ifdef SKYWIRE_EXPERIMENTAL
      , _worker(&modem_serial, "flightcontroltower.klevebrand.se", 13000, 20000, false)
#endif
{
}

void SkywireDroneController::setup()
{
#ifdef SKYWIRE_EXPERIMENTAL
    powerOnModem();

    _modem_serial.begin(115200);

    SkywireCommandStartupWorker<receive_buffer_size> startup_worker(&_modem_serial, false);

    while (!startup_worker.run())
    {
    }

    Serial.println("Skywire started.");
#endif
}

/*
 * Switches on the modem supply (it is off at boot) and pulses ON_OFF. RTS is asserted first: the modem starts with
 * hardware flow control on and only talks while RTS is low, until the startup worker turns flow control off with
 * AT&K0. The modem needs about 20 s more before it answers AT.
 */
void SkywireDroneController::powerOnModem() const
{
    _request_to_send_line.setModeOutput(false);
    _on_line.setModeOutput(false);
    _power_enable_line.setModeOutput(true);

    delay(MODEM_SUPPLY_SETTLE_MILLISECONDS);

    _on_line.write(true);
    delay(MODEM_ON_OFF_PULSE_MILLISECONDS);
    _on_line.write(false);
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

    // A plain literal: on the Teensy core PROGMEM is a named section, and a PSTR here would clash with the PSTRs
    // inside the skywire templates ("section type conflict").
    char payload_to_send[128];
    snprintf(
        payload_to_send,
        sizeof(payload_to_send),
        "1;1337;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%d;0",
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
}
