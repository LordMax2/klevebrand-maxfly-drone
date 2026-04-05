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

#define HTTP_GPS_STEP_COUNT 7

class SkywireHttpGpsStepWorker : public SkywireCommandWorker
{
public:
	SkywireHttpGpsStepWorker(
		HardwareSerial *skywire_serial,
		String base_url,
		int port,
		String get_path,
		String post_path,
		unsigned long timeout_milliseconds,
		bool debug_mode) : SkywireCommandWorker(skywire_serial, debug_mode, timeout_milliseconds, HTTP_GPS_STEP_COUNT)
	{
		String http_cfg_command = "AT#HTTPCFG=0,\"" + base_url + "\",80,0,\"\",\"\",0,5";
		String http_qry_command = "AT#HTTPQRY=0,0,\"/" + get_path + "\"";
		String http_snd_path = "\"/" + post_path + "\"";

		//this->steps[0] = new GpsAcpSkywireCommand(skywire, debug_mode, setLatestGpsResponse);
		this->steps[0] = new SkywireCommand(skywire, http_cfg_command.c_str(), debug_mode, nullptr);
		this->steps[1] = new SkywireCommand(skywire, http_qry_command.c_str(), debug_mode, nullptr);
		this->steps[2] = new HttpRingSkywireCommand(skywire, debug_mode, nullptr);
		this->steps[3] = new HttpRcvSkywireCommand(skywire, debug_mode, nullptr);
		this->steps[4] = new SkywireCommand(skywire, http_cfg_command.c_str(), debug_mode, nullptr);
		this->steps[5] = new HttpSndSkywireCommand(skywire, debug_mode, http_snd_path.c_str(), nullptr);
		this->steps[6] = new HttpRcvSkywireCommand(skywire, debug_mode, nullptr);

		reset();
	}

	void setPayloadToSend(char payload[128])
	{
		static_cast<HttpSndSkywireCommand *>(steps[5])->setPayload(payload);
	}
	GpsLocationInfo_t getLatestGpsResponse();
	DroneRequest_t getLatestDroneRequest();
	static void setLatestHttpResponse(char response[512]);
	static void setLatestGpsResponse(char response[512]);

private:
	static GpsLocationInfo_t _latest_gps_response;
	static DroneRequest_t _latest_drone_request_response;
};

#endif // KLEVEBRAND_SKYWIRE_HTTP_STEPPER_CLIENT_H
