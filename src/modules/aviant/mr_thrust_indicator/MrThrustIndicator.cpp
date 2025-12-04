#include "MrThrustIndicator.hpp"
#include "px4_platform_common/defines.h"

using namespace time_literals;

namespace aviant
{

MrThrustIndicator::MrThrustIndicator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

MrThrustIndicator::~MrThrustIndicator()
{
	Stop();
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool MrThrustIndicator::Start()
{
	ParametersUpdate(true);

	_thrust_max_filter.reset(NAN);
	_thrust_max_bomi_filter.reset(NAN);

	ScheduleOnInterval(_params_avi_thr_ind_int.get() * 1_ms);
	return true;
}

void MrThrustIndicator::Stop()
{
	Deinit();
}

void MrThrustIndicator::Run()
{
	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	ParametersUpdate();

	// Main loop logic for Thrust Indicator module	// e.g., monitor thrust levels, log data, etc.

	vehicle_thrust_setpoint_s thrust_setpoint;
	vehicle_torque_setpoint_s torque_setpoint;

	if (!_vehicle_thrust_setpoint_sub.copy(&thrust_setpoint)) {
		// No thrust setpoint
		goto done;
	}

	if (!_vehicle_torque_setpoint_sub.copy(&torque_setpoint)) {
		// No torque setpoint
		goto done;
	}

	{
		aviant_mr_thrust_indicator_s out;


		const float R = torque_setpoint.xyz[0];
		const float P = torque_setpoint.xyz[1];
		constexpr float Y = 0.0f;
		const float Z = thrust_setpoint.xyz[2];

		// We calculate the worst motors without any yaw setpoints.
		// However, this is still slightly pessimistic, because we don't model
		// the yaw desaturation which happens when a motor saturates.

		// Heaviest loaded motor with normal allocation:
		// We use the heuristic that this is the rotor in the direction where roll, pitch and yaw demand is greatest (all absolute values point the same way)
		const float worst_motor_nom = 0.857f *  M_SQRT1_2_F * (std::abs(R) + std::abs(P)) + std::abs(Y) - Z;

		// Heaviest loaded bomi:
		// We use the heuristic that this is the rotor in the direction of roll, pitch.
		// This does not account for desaturation, and is therefore conservative.
		// Constants are from `derivation.py` script, assuming bottom rotor thrust and torque
		// reduced by 60% compared to top mmotor for the same commanded input.

		// NB! TODO: This is for loss of a bottom motor, loss of a top motor needs to be added, and is probably worse.
		const float worst_motor_bomi = 0.919f * std::abs(P) + 0.919f * std::abs(R) + 1.3f * std::abs(Y) + 0.700f * std::abs(Z);

		// TODO: Handle airmode
		// TODO: Handle desaturation
		const hrt_abstime now{hrt_absolute_time()};
		const float dt_s = (now - _prev_update) / 1e6f;
		UpdateFilter(_thrust_max_filter, worst_motor_nom, dt_s);
		UpdateFilter(_thrust_max_bomi_filter, worst_motor_bomi, dt_s);
		_prev_update = now;

		out.timestamp = _prev_update;
		out.thrust_max = _thrust_max_filter.getState();
		out.thrust_max_bomi = _thrust_max_bomi_filter.getState();

		if (out.thrust_max > 1.0f) {
			// Not enough thrust even in current conditions
			out.status = aviant_mr_thrust_indicator_s::THRUST_STATUS_CRITICAL;

		} else if (out.thrust_max_bomi > 1.0f) {
			// Not enough thrust if one motor fails
			out.status = aviant_mr_thrust_indicator_s::THRUST_STATUS_WARNING;

		} else {
			out.status = aviant_mr_thrust_indicator_s::THRUST_STATUS_NOMINAL;
		}

		_aviant_mr_thrust_indicator_pub.publish(out);
	}

done:
	perf_end(_loop_perf);
}

void MrThrustIndicator::PrintStatus()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
};

void MrThrustIndicator::ParametersUpdate(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
	}
}

void MrThrustIndicator::UpdateFilter(AlphaFilter<float> &filter, float sample, float dt_s)
{
	if (!PX4_ISFINITE(sample)) {
		filter.reset(NAN);
	}

	if (!PX4_ISFINITE(filter.getState())) {
		filter.reset(sample);

	} else {
		// We must update the filter parameters, since we don't know the sample rate
		filter.setParameters(dt_s, _params_avi_thr_ind_tau.get());
		filter.update(sample);
	}
}

} // namespace aviant
