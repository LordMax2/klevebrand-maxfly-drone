#pragma once

#include "drone_request.h"
#include "../klevebrand_maxfly_drone.h"

#ifdef SKYWIRE_EXPERIMENTAL
#define CONCEPTS_H
#include "skywire-command-tcp-gps-step-worker.h"
#endif

class SkywireDroneController
{
public:
    SkywireDroneController();
    static void setup();
    void run(KlevebrandMaxFlyDrone *drone);
    void setRequest(const DroneRequest_t &request);
    void clearRequest();
    void requestControl();
    void releaseControl();
    bool wantsControl() const;
    void apply(KlevebrandMaxFlyDrone *drone) const;

private:
    DroneRequest_t _request = DroneRequest_t::empty();
    bool _has_pending_request = false;
    bool _wants_control = false;

#ifdef SKYWIRE_EXPERIMENTAL
    static constexpr size_t receive_buffer_size = 256;
    static constexpr size_t socket_send_message_size = 160;
    static constexpr size_t socket_receive_size = 128;

    SkywireTcpGpsStepWorker<receive_buffer_size, socket_send_message_size, socket_receive_size> _worker;
#endif
};
