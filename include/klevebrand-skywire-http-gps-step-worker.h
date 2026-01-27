#ifndef KLEVEBRAND_SKYWIRE_HTTP_STEPPER_CLIENT_H
#define KLEVEBRAND_SKYWIRE_HTTP_STEPPER_CLIENT_H

#include "Arduino.h"
#include "klevebrand-skywire-framework.h"
#include "klevebrand-skywire-framework-step-worker.h"
#include "klevebrand-skywire-framework-step.h"
#include "klevebrand-skywire-framework-step-httprcv.h"
#include "klevebrand-skywire-framework-step-gpsacp.h"
#include "klevebrand-skywire-framework-step-httpring.h"

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
		bool debug_mode = false) : SkywireStepWorker(skywire_serial, debug_mode, STEP_COUNT)
	{
		this->steps = new SkywireStep *[STEP_COUNT];

		this->steps[0] = new SkywireStep(skywire, "AT#HTTPCFG=0,\"" + base_url + "\"", debug_mode, nullptr);
		this->steps[1] = new SkywireStep(skywire, "AT#HTTPQRY=0,0,\"/" + path + "\"\r", debug_mode, nullptr);
		this->steps[2] = new HttpRingSkywireStep(skywire, debug_mode, nullptr);
		this->steps[3] = new HttpRcvSkywireStep(skywire, debug_mode, setLatestGpsResponse);
		this->steps[4] = new GpsAcpSkywireStep(skywire, debug_mode, setLatestHttpResponse);

		resetState();
	}
	
	String getLatestGpsResponse() {
		return _latest_gps_response;
	}

	String getLatestHttpResponse() {
		return _latest_http_response;
	}

	static void setLatestGpsResponse(String response) {
		_latest_gps_response = response;
	}

	static void setLatestHttpResponse(String response) {
		_latest_http_response = response;
	}

private:
	static String _latest_gps_response;
	static String _latest_http_response;
};

#endif // KLEVEBRAND_SKYWIRE_HTTP_STEPPER_CLIENT_H
