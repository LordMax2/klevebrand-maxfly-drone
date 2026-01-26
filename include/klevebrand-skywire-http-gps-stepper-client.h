#ifndef KLEVEBRAND_SKYWIRE_HTTP_STEPPER_CLIENT_H
#define KLEVEBRAND_SKYWIRE_HTTP_STEPPER_CLIENT_H

#include "Arduino.h"
#include "klevebrand-skywire-framework.h"
#include "klevebrand-skywire-http-stepper-client-step.h"
#include "klevebrand-wait_for_http_ring.h"
#include "klevebrand-wait_for_http_rcv.h"
#include "klevebrand-wait_for_gpc_acp.h"

#define STEP_COUNT 5

class SkywireHttpGpsStepperClient
{
public:
	SkywireHttpGpsStepperClient(
		HardwareSerial &skywire_serial,
		String base_url,
		int port,
		String path,
		void (*on_http_completed_function)(String result_content) = nullptr,
		void (*on_gps_completed_function)(String response_content) = nullptr,
		unsigned long timeout_milliseconds = 5000,
		bool debug_mode = false) : _skywire(skywire_serial, debug_mode)
	{
		this->_steps = new SkywireHttpStepperClientStep *[STEP_COUNT];

		this->_steps[0] = new SkywireHttpStepperClientStep(_skywire, "AT#HTTPCFG=0,\"" + base_url + "\"", debug_mode, nullptr);
		this->_steps[1] = new SkywireHttpStepperClientStep(_skywire, "AT#HTTPQRY=0,0,\"/" + path + "\"\r", debug_mode, nullptr);
		this->_steps[2] = new WaitForHttpRing(_skywire, debug_mode, nullptr);
		this->_steps[3] = new HttpRcv(_skywire, debug_mode, setLatestGpsResponse);
		this->_steps[4] = new GpsAcp(_skywire, debug_mode, setLatestHttpResponse);

		resetState();
	}
	bool start();
	void run();
	void resetState();
	
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
	Skywire _skywire;

	SkywireHttpStepperClientStep **_steps;

	static String _latest_gps_response;
	static String _latest_http_response;
};

#endif // KLEVEBRAND_SKYWIRE_HTTP_STEPPER_CLIENT_H
