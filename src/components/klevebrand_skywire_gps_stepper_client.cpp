#include "klevebrand_skywire_gps_stepper_client.h"

bool KlevebrandSkywireGpsStepperClient::start()
{
    _skywire.start();

    _skywire.print("AT#GPIO=5,1,1,1\r");

    _skywire.enableGps();

    return true;
}

void KlevebrandSkywireGpsStepperClient::serialReadToRxBuffer()
{
	if (_skywire.available())
	{
		char c = _skywire.read();
		_rx_buffer += c;
	}
}

SkywireResponseResult_t KlevebrandSkywireGpsStepperClient::get()
{
    if(!_gpsAcpSent)
    {
        _skywire.print("AT$GPSACP\r");

        _gpsAcpSent = true;

        return SkywireResponseResult_t(false, "");
    }

	serialReadToRxBuffer();

	if (_rx_buffer.indexOf("\r\n") == -1 || _skywire.available() || _rx_buffer.indexOf("OK") == -1)
    { 
        return SkywireResponseResult_t(false, "");
    }
    String result = _rx_buffer;

    _rx_buffer = "";
    _gpsAcpSent = false;

    return SkywireResponseResult_t(true, result);
}