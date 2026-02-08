#ifndef KLEVEBRAND_SKYWIRE_HTTP_STEPPER_CLIENT_H
#define KLEVEBRAND_SKYWIRE_HTTP_STEPPER_CLIENT_H

#include "Arduino.h"
#include "klevebrand-skywire-framework.h"
#include "klevebrand-skywire-framework-step-worker.h"
#include "klevebrand-skywire-framework-step.h"
#include "klevebrand-skywire-framework-step-httprcv.h"
#include "klevebrand-skywire-framework-step-gpsacp.h"
#include "klevebrand-skywire-framework-step-httpring.h"
#include "gps_location_info.h"
#include "drone_request.h"

#define STEP_COUNT 5

class SkywireHttpGpsStepWorker : public SkywireStepWorker
{
public:
	SkywireHttpGpsStepWorker(
		HardwareSerial &skywire_serial,
		String base_url,
		int port,
		String path,
		void (*on_http_completed_function)(String result_content) = nullptr,
		void (*on_gps_completed_function)(String response_content) = nullptr,
		unsigned long timeout_milliseconds = 5000,
		bool debug_mode = false) : SkywireStepWorker(skywire_serial, debug_mode, timeout_milliseconds, STEP_COUNT)
	{
		this->steps = new SkywireStep *[STEP_COUNT];

		this->steps[0] = new GpsAcpSkywireStep(skywire, debug_mode, setLatestGpsResponse);
		this->steps[1] = new SkywireStep(skywire, "AT#HTTPCFG=0,\"" + base_url + "\",80,0,\"\",\"\",0,5", debug_mode, nullptr);
		this->steps[2] = new SkywireStep(skywire, "AT#HTTPQRY=0,0,\"/" + path + "\"", debug_mode, nullptr);
		this->steps[3] = new HttpRingSkywireStep(skywire, debug_mode, nullptr);
		this->steps[4] = new HttpRcvSkywireStep(skywire, debug_mode, setLatestHttpResponse);

		resetState();
	}

	GpsLocationInfo_t getLatestGpsResponse();
	DroneRequest_t getLatestDroneRequest();
	static void setLatestHttpResponse(String &response);
	static void setLatestGpsResponse(String &response);

private:
	static GpsLocationInfo_t latest_gps_response;
	static DroneRequest_t latest_drone_request_response;
};

#endif // KLEVEBRAND_SKYWIRE_HTTP_STEPPER_CLIENT_H
