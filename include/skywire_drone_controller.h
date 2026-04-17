#ifndef SKYWIRE_DRONE_CONTROLLER_H
#define SKYWIRE_DRONE_CONTROLLER_H

#include "drone_request.h"
#include "klevebrand_maxfly_drone.h"

class SkywireDroneController
{
public:
    void setup();
    void run(KlevebrandMaxFlyDrone *drone);
    void setRequest(const DroneRequest_t &request);
    void clearRequest();
    void requestControl();
    void releaseControl();
    bool hasPendingRequest() const;
    bool wantsControl() const;
    void apply(KlevebrandMaxFlyDrone *drone);

private:
    DroneRequest_t _request = DroneRequest_t::empty();
    bool _has_pending_request = false;
    bool _wants_control = false;

#ifdef SKYWIRE_EXPERIMENTAL
    class SkywireTcpGpsStepWorker *_worker = nullptr;
#endif
};

#endif // SKYWIRE_DRONE_CONTROLLER_H
