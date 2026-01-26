#include "klevebrand-skywire-http-gps-stepper-client.h"

void SkywireHttpGpsStepperClient::resetState()
{
	for (int i = 0; i < STEP_COUNT; i++)
	{
		_steps[i]->resetState();
	}
}

bool SkywireHttpGpsStepperClient::start()
{
	_skywire.start();

	_skywire.enableGps();

	return true;
}

void SkywireHttpGpsStepperClient::run()
{
	for (int i = 0; i < STEP_COUNT; i++)
	{
		SkywireResponseResult_t step_result = _steps[i]->process();

		if (!step_result.is_success)
		{
			break;
		}
	}

	bool all_completed = true;

	for (int i = 0; i < STEP_COUNT; i++)
	{
		if (!_steps[i]->completed())
		{
			all_completed = false;
			break;
		}
	}

	if (all_completed)
	{
		resetState();
	}
}
