#ifndef KLEVEBRAND_SKYWIRE_HTTP_STEPPER_CLIENT_H
#define KLEVEBRAND_SKYWIRE_HTTP_STEPPER_CLIENT_H

#include "Arduino.h"
#include "skywire-command-worker.h"
#include "skywire-command.h"
#include "skywire-command-httprcv.h"
#include "skywire-command-gpsacp.h"
#include "skywire-command-httpring.h"
#include "skywire-command-httpsnd.h"
#include "gps_location_info.h"
#include "drone_request.h"

#define STEP_COUNT 7

class SkywireHttpGpsStepWorker : public SkywireCommandWorker
{
public:
	SkywireHttpGpsStepWorker(
		HardwareSerial *skywire_serial,
		String base_url,
		int port,
		String get_path,
		String post_path,
		void (*on_http_completed_function)(String result_content) = nullptr,
		void (*on_gps_completed_function)(String response_content) = nullptr,
		unsigned long timeout_milliseconds = 5000,
		bool debug_mode = true) : SkywireCommandWorker(skywire_serial, debug_mode, timeout_milliseconds, STEP_COUNT)
	{
		this->steps = new SkywireCommand *[STEP_COUNT];

		//this->steps[0] = new GpsAcpSkywireCommand(skywire, debug_mode, setLatestGpsResponse);
		this->steps[0] = new SkywireCommand(skywire, "AT#HTTPCFG=0,\"" + base_url + "\",80,0,\"\",\"\",0,5", debug_mode, nullptr);
		this->steps[1] = new SkywireCommand(skywire, "AT#HTTPQRY=0,0,\"/" + get_path + "\"", debug_mode, nullptr);
		this->steps[2] = new HttpRingSkywireCommand(skywire, debug_mode, nullptr);
		this->steps[3] = new HttpRcvSkywireCommand(skywire, debug_mode, setLatestHttpResponse);
		this->steps[4] = new HttpSndSkywireCommand(skywire, debug_mode, "/" + post_path, nullptr);
		this->steps[5] = new HttpRingSkywireCommand(skywire, debug_mode, nullptr);
		this->steps[6] = new HttpRcvSkywireCommand(skywire, debug_mode, nullptr);

		resetState();
	}

	void setPayloadToSend(String payload)
	{
		static_cast<HttpSndSkywireCommand *>(steps[4])->setPayload(payload);
	}
	GpsLocationInfo_t getLatestGpsResponse();
	DroneRequest_t getLatestDroneRequest();
	static void setLatestHttpResponse(String &response);
	static void setLatestGpsResponse(String &response);

private:
	static GpsLocationInfo_t _latest_gps_response;
	static DroneRequest_t _latest_drone_request_response;
};

#endif // KLEVEBRAND_SKYWIRE_HTTP_STEPPER_CLIENT_H
