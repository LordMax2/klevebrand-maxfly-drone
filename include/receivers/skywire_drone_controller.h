#pragma once

#include <HardwareSerial.h>
#include "drone_request.h"
#include "imxrt_gpio_pad.h"
#include "../klevebrand_maxfly_drone.h"

#ifdef SKYWIRE_EXPERIMENTAL
#define CONCEPTS_H
#include "skywire-command-tcp-gps-step-worker.h"
#endif

class SkywireDroneController
{
public:
    /*
     * The Skywire modem on modem_serial. setup() asserts RTS (request_to_send_line), switches on the modem supply
     * (power_enable_line) and pulses ON_OFF, which the board pulls low while on_line is high.
     */
    SkywireDroneController(HardwareSerial &modem_serial, const ImxrtGpioPad &request_to_send_line,
                           const ImxrtGpioPad &on_line, const ImxrtGpioPad &power_enable_line);
    void setup();
    void run(KlevebrandMaxFlyDrone *drone);
    void setRequest(const DroneRequest_t &request);
    void clearRequest();
    void requestControl();
    void releaseControl();
    bool wantsControl() const;
    void apply(KlevebrandMaxFlyDrone *drone) const;

private:
    HardwareSerial &_modem_serial;
    const ImxrtGpioPad _request_to_send_line;
    const ImxrtGpioPad _on_line;
    const ImxrtGpioPad _power_enable_line;

    DroneRequest_t _request = DroneRequest_t::empty();
    bool _has_pending_request = false;
    bool _wants_control = false;

    void powerOnModem() const;

#ifdef SKYWIRE_EXPERIMENTAL
    static constexpr size_t receive_buffer_size = 256;
    static constexpr size_t socket_send_message_size = 160;
    static constexpr size_t socket_receive_size = 128;

    SkywireTcpGpsStepWorker<receive_buffer_size, socket_send_message_size, socket_receive_size> _worker;
#endif
};
