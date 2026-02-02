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
		this->steps[1] = new SkywireStep(skywire, "AT#HTTPQRY=0,0,\"/" + path + "\"", debug_mode, nullptr);
		this->steps[2] = new HttpRingSkywireStep(skywire, debug_mode, nullptr);
		this->steps[3] = new HttpRcvSkywireStep(skywire, debug_mode, setLatestHttpResponse);
		this->steps[4] = new GpsAcpSkywireStep(skywire, debug_mode, setLatestGpsResponse);

		resetState();
	}
	
	String getLatestGpsResponse() {
		return "";
	}

	String getLatestHttpResponse() {
		return "";
	}

	static void setLatestHttpResponse(String &response)
	{
		String response_copy = response;

		response_copy.replace("OK", "");
		response_copy.replace("<<<", "");
		response_copy.replace("\r", "");
		response_copy.replace("\n", "");
		response_copy.replace("ERROR", "");

		Serial.println(response_copy);
	}

	static void setLatestGpsResponse(String& response) {
		String response_copy = response;

		response_copy.replace("OK", "");
		response_copy.replace("$GPSACP: ", "");
		response_copy.replace("\r", "");
		response_copy.replace("\n", "");

		Serial.println(response_copy);
	}
};

#endif // KLEVEBRAND_SKYWIRE_HTTP_STEPPER_CLIENT_H
