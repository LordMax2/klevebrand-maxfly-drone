#pragma once

#include "../entities/drone_request.h"
#include "../klevebrand_maxfly_drone.h"

class SkywireDroneController
{
public:
    static void setup();
    static void run(const KlevebrandMaxFlyDrone *drone);
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
    class SkywireTcpGpsStepWorker *_worker = nullptr;
#endif
};
