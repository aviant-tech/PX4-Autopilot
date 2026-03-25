#include <px4_platform_common/log.h>

#include "aviant.hpp"

using namespace aviant;
using namespace time_literals;


Aviant::Aviant() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	parameters_update();

	if (_params_av_nav_ind_en.get()) {
		_navigation_indicator = new Navigation();
		_navigation_indicator->start();
	}

	if (_params_av_temp_ind_en.get()) {
		_temperature_indicator = new Temperature();

		if (_temperature_indicator) {
			_temperature_indicator->start();

		} else {
			PX4_ERR("Temperature indicator alloc failed");
		}
	}

}

Aviant::~Aviant()
{
	if (_navigation_indicator) {
		_navigation_indicator->stop();
		delete _navigation_indicator;
	}

	if (_temperature_indicator) {
		_temperature_indicator->stop();
		delete _temperature_indicator;
	}

	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

void Aviant::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Placeholder: Any work for the Aviant main module
	// (Currently it only starts/stops sub-modules)

	ScheduleDelayed(100_ms);
	perf_end(_loop_perf);

}

bool Aviant::init()
{
	// Register callbacks, initialize state, etc.
	ScheduleNow();
	return true;
}

int Aviant::task_spawn(int argc, char *argv[])
{
	Aviant *instance = new Aviant();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int Aviant::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Aviant::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	PX4_INFO_RAW("Aviant module is running.\n");

	if (_navigation_indicator) {
		PX4_INFO_RAW("\n");
		_navigation_indicator->print_status();
	}

	if (_temperature_indicator) {
		PX4_INFO_RAW("\n");
		_temperature_indicator->print_status();
	}

	return 0;
}

int Aviant::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module contains custom modules for Aviant's drone platform.

It is separated from other parts of the codebase in order to decouple aviant-specific features from the upstream PX4
codebase, making it easier to maintain and update PX4.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("aviant", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

void Aviant::parameters_update(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
	}
}

extern "C" __EXPORT int aviant_main(int argc, char *argv[])
{
    return Aviant::main(argc, argv);
}
