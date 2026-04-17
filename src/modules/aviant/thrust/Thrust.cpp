#include "Thrust.hpp"

#include <math.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <parameters/param.h>

namespace aviant
{

using namespace time_literals;

Thrust::Thrust() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

Thrust::~Thrust()
{
	stop();
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool Thrust::start()
{
	parametersUpdate(true);

	for (int i = 0; i < MAX_MOTORS; i++) {
		_vpha_lpf[i].setParameters(SCHEDULE_INTERVAL_US * 1e-6f, _param_av_thr_vpha_tau.get());
	}

	_current_error_lpf.setParameters(SCHEDULE_INTERVAL_US * 1e-6f, _param_av_thr_cur_tau.get());

	ScheduleOnInterval(SCHEDULE_INTERVAL_US);
	return true;
}

void Thrust::stop()
{
	px4::ScheduledWorkItem::Deinit();
}

void Thrust::Run()
{
	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	parametersUpdate();

	aviant_motors_s motors_out = {};

	for (int i = 0; i < MAX_MOTORS; i++) {
		motors_out.mot_load_status[i] = aviant_motors_s::STATE_INACTIVE;
	}

	motors_out.anomalous_current_status = aviant_motors_s::STATE_INACTIVE;

	battery_status_s battery{};
	actuator_outputs_s pwm_main{};
	actuator_outputs_s pwm_aux{};

	// Bitwise | (not ||) so all three subs are consumed each cycle — avoids
	// discarding a fresh group when another hasn't ticked, and tolerates
	// airframes that only use one PWM group.
	if (_battery_status_sub.update(&battery)
	    | _actuator_outputs_subs[0].update(&pwm_main)
	    | _actuator_outputs_subs[1].update(&pwm_aux)) {

		float duties[MAX_MOTORS] = {};
		calculateDutyCycles(duties, pwm_main, pwm_aux);

		updateMotorVoltages(&motors_out, battery.voltage_v, duties);
		updateAnomalousCurrent(&motors_out, battery.voltage_v, battery.current_a, duties);
	}

	motors_out.timestamp = hrt_absolute_time();
	_aviant_motors_pub.publish(motors_out);

	perf_end(_loop_perf);
}

void Thrust::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
}

void Thrust::parametersUpdate(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		updateFunctionMapping();
	}
}

void Thrust::updateFunctionMapping()
{
	static const char *const prefixes[NUM_OUTPUT_GROUPS] = {"PWM_MAIN_FUNC", "PWM_AUX_FUNC"};

	for (int group = 0; group < NUM_OUTPUT_GROUPS; group++) {
		for (int i = 0; i < MAX_OUTPUTS; i++) {
			_output_to_motor[group][i] = -1;

			char param_name[20];
			snprintf(param_name, sizeof(param_name), "%s%d", prefixes[group], i + 1);
			param_t param = param_find_no_notification(param_name);

			if (param != PARAM_INVALID) {
				int32_t func = 0;

				if (param_get(param, &func) == 0 && func >= FUNC_MOTOR1
				    && func < FUNC_MOTOR1 + MAX_MOTORS) {
					_output_to_motor[group][i] = (int8_t)(func - FUNC_MOTOR1);
				}
			}
		}
	}
}

void Thrust::calculateDutyCycles(float (&duties_out)[MAX_MOTORS],
				 const actuator_outputs_s &pwm_main,
				 const actuator_outputs_s &pwm_aux) const
{
	const actuator_outputs_s *const groups[NUM_OUTPUT_GROUPS] = {&pwm_main, &pwm_aux};

	for (int group = 0; group < NUM_OUTPUT_GROUPS; group++) {
		const actuator_outputs_s &outputs = *groups[group];
		const int n_outputs = math::min((int)outputs.noutputs, MAX_OUTPUTS);

		for (int ch = 0; ch < n_outputs; ch++) {
			const int motor_idx = _output_to_motor[group][ch];

			if (motor_idx < 0 || motor_idx >= MAX_MOTORS) {
				continue;
			}

			const float pwm = outputs.output[ch];

			if (!PX4_ISFINITE(pwm)) {
				continue;
			}

			const float clamped = math::constrain(pwm, 1000.0f, 2000.0f);
			duties_out[motor_idx] = (clamped - 1000.0f) / 1000.0f;
		}
	}
}

float Thrust::predictCurrent(float v_bat, const float (&duties)[MAX_MOTORS]) const
{
	if (!PX4_ISFINITE(v_bat)) {
		return 0.0f;
	}

	const uint32_t psh_mask = _param_av_thr_mot_psh.get();
	const uint32_t top_mask = _param_av_thr_mot_top.get();

	const float psh_iref = _param_av_thr_psh_iref.get();
	const float psh_k    = _param_av_thr_psh_k.get();
	const float psh_vref = _param_av_thr_psh_vref.get();
	const float top_iref = _param_av_thr_top_iref.get();
	const float top_k    = _param_av_thr_top_k.get();
	const float top_vref = _param_av_thr_top_vref.get();

	// I_bus_motor = D * I_ref * (V_pha / V_ref) ^ k
	float predicted_current = 0.0f;

	for (int i = 0; i < MAX_MOTORS; i++) {
		if (duties[i] <= 0.0f) {
			continue;
		}

		const uint32_t motor_bit = (1u << i);
		float i_ref;
		float k;
		float v_ref;

		if (psh_mask & motor_bit) {
			i_ref = psh_iref;
			k = psh_k;
			v_ref = psh_vref;

		} else if (top_mask & motor_bit) {
			i_ref = top_iref;
			k = top_k;
			v_ref = top_vref;

		} else {
			// Motor isn't in either mask → no model, leave out of the sum.
			continue;
		}

		if (v_ref <= 0.0f) {
			// v_ref not configured → model is unusable, bail out entirely rather
			// than silently biasing the prediction and triggering spurious anomalies.
			return NAN;
		}

		const float vpha = v_bat * duties[i];
		predicted_current += duties[i] * i_ref * powf(vpha / v_ref, k);
	}

	return predicted_current;
}

void Thrust::updateMotorVoltages(aviant_motors_s *out, float v_bat, const float (&duties)[MAX_MOTORS])
{
	// v_bat == 0 means "unknown" per BatteryStatus.msg, not 0 V — treat as no data.
	if (!PX4_ISFINITE(v_bat) || v_bat <= 0.0f) {
		return;
	}

	const uint32_t psh_mask = _param_av_thr_mot_psh.get();
	const uint32_t top_mask = _param_av_thr_mot_top.get();
	const float psh_crit_v = _param_av_thr_psh_cri_v.get();
	const float psh_warn_v = _param_av_thr_psh_wrn_v.get();
	const float top_crit_v = _param_av_thr_top_cri_v.get();
	const float top_warn_v = _param_av_thr_top_wrn_v.get();

	for (int i = 0; i < MAX_MOTORS; i++) {
		if (duties[i] <= 0.0f) {
			continue;
		}

		const uint32_t motor_bit = (1u << i);
		float vpha_crit;
		float vpha_warn;

		if (psh_mask & motor_bit) {
			vpha_crit = psh_crit_v;
			vpha_warn = psh_warn_v;

		} else if (top_mask & motor_bit) {
			vpha_crit = top_crit_v;
			vpha_warn = top_warn_v;

		} else {
			// Motor isn't in either mask → no thresholds to evaluate, leave as STATE_INACTIVE.
			continue;
		}

		const float raw_vpha = v_bat * duties[i];
		const float vpha = _vpha_lpf[i].update(raw_vpha);
		out->mot_voltage_filtered_cv[i] = (int16_t)roundf(math::constrain(vpha * 100.0f,
						  (float)INT16_MIN, (float)INT16_MAX));

		if (vpha_crit > 0.0f && vpha >= vpha_crit) {
			out->mot_load_status[i] = aviant_motors_s::STATE_CRITICAL;

		} else if (vpha_warn > 0.0f && vpha >= vpha_warn) {
			out->mot_load_status[i] = aviant_motors_s::STATE_WARNING;

		} else {
			out->mot_load_status[i] = aviant_motors_s::STATE_NOMINAL;
		}
	}
}

void Thrust::updateAnomalousCurrent(aviant_motors_s *out, float v_bat, float current_a,
				    const float (&duties)[MAX_MOTORS])
{
	// v_bat == 0 and current_a == -1 mean "unknown" per BatteryStatus.msg — treat as no data
	// to avoid spurious anomaly warnings and poisoning the LPF state.
	if (!PX4_ISFINITE(v_bat) || !PX4_ISFINITE(current_a) || v_bat <= 0.0f || current_a < 0.0f) {
		return;
	}

	const float predicted_current = predictCurrent(v_bat, duties);

	if (!PX4_ISFINITE(predicted_current)) {
		return;
	}

	out->predicted_current_a = predicted_current;

	const float anomaly = current_a - predicted_current;
	const float filtered_anomaly = _current_error_lpf.update(anomaly);

	out->anomalous_current_filtered_ca = (int16_t)roundf(math::constrain(filtered_anomaly * 100.0f,
					     (float)INT16_MIN, (float)INT16_MAX));

	const float crit_hi = _param_av_thr_cri_hi_a.get();
	const float crit_lo = _param_av_thr_cri_lo_a.get();
	const float warn_hi = _param_av_thr_wrn_hi_a.get();
	const float warn_lo = _param_av_thr_wrn_lo_a.get();

	if (crit_hi > 0.0f && filtered_anomaly >= crit_hi) {
		out->anomalous_current_status = aviant_motors_s::STATE_CRITICAL;

	} else if (crit_lo > 0.0f && filtered_anomaly <= -crit_lo) {
		out->anomalous_current_status = aviant_motors_s::STATE_CRITICAL;

	} else if (warn_hi > 0.0f && filtered_anomaly >= warn_hi) {
		out->anomalous_current_status = aviant_motors_s::STATE_WARNING;

	} else if (warn_lo > 0.0f && filtered_anomaly <= -warn_lo) {
		out->anomalous_current_status = aviant_motors_s::STATE_WARNING;

	} else {
		out->anomalous_current_status = aviant_motors_s::STATE_NOMINAL;
	}
}

} // namespace aviant
