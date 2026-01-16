#ifndef KLEVEBRAND_SKYWIRE_GPS_STEPPER_CLIENT_H
#define KLEVEBRAND_SKYWIRE_GPS_STEPPER_CLIENT_H

#include "Arduino.h"
#include "klevebrand-skywire-framework.h"

class KlevebrandSkywireGpsStepperClient {
public:
    KlevebrandSkywireGpsStepperClient(HardwareSerial& skywire_serial)
        : _skywire(skywire_serial, false) {};

	bool start();
	SkywireResponseResult_t get();
private:
    Skywire _skywire; 
	String _rx_buffer;

    bool _gpsAcpSent = false;

	void serialReadToRxBuffer();
};

#endif // KLEVEBRAND_SKYWIRE_GPS_STEPPER_CLIENT_H