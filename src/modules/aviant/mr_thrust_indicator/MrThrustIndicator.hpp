#pragma once

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/aviant_mr_thrust_indicator.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>

using namespace time_literals;

namespace aviant
{
class MrThrustIndicator : public ModuleParams, public px4::ScheduledWorkItem
{
public:

	MrThrustIndicator();
	~MrThrustIndicator() override;
	bool Start();
	void Stop();

	void PrintStatus();
private:
	void Run() override;
	void ParametersUpdate(bool force = false);
	void UpdateFilter(AlphaFilter<float> &filter, float sample, float dt);

	// We want instance 0, since it represents the setpoints for MR motors.
	// (Instance 1 is only used for FW control surface allocation)
	uORB::Subscription _vehicle_torque_setpoint_sub{ORB_ID(vehicle_torque_setpoint), 0};
	// We want instance 0, since its z-component represents MR thrust.
	// Instance 0 is used for all thrusts on standard_vtol
	// (Instance 1 is unused, vtol_att_control sets all components to constant 0)
	uORB::Subscription _vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint), 0};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<aviant_mr_thrust_indicator_s> _aviant_mr_thrust_indicator_pub{ORB_ID(aviant_mr_thrust_indicator)};

	AlphaFilter<float> _thrust_max_filter;
	AlphaFilter<float> _thrust_max_bomi_filter;
	hrt_abstime _prev_update{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::AV_THR_IND_INT>) _params_avi_thr_ind_int,
		(ParamFloat<px4::params::AV_THR_IND_TAU>) _params_avi_thr_ind_tau
	);

	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
};
} // namespace aviant
