#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include "navigation/Navigation.hpp"
#include "temperature/Temperature.hpp"
#include "thrust/Thrust.hpp"

using namespace aviant;

class Aviant : public ModuleBase<Aviant>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	explicit Aviant();
	~Aviant() override;

	void Run() override;

	bool init();
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	int print_status() override;
	static int print_usage(const char *reason = nullptr);

private:
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	void parameters_update(bool force = false);

	Navigation *_navigation_indicator{nullptr};
	Temperature *_temperature_indicator{nullptr};
	Thrust *_thrust_indicator{nullptr};

	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::AV_NAV_IND_EN>) _params_av_nav_ind_en,
		(ParamBool<px4::params::AV_TEMP_IND_EN>) _params_av_temp_ind_en,
		(ParamBool<px4::params::AV_THR_IND_EN>) _params_av_thr_ind_en
	);
};
