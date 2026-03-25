#pragma once

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <mathlib/math/filter/AlphaFilter.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionMultiArray.hpp>

#include <uORB/topics/aviant_motors.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/battery_status.h>

namespace aviant
{

using namespace time_literals;

class Thrust : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	Thrust();
	~Thrust() override;

	bool start();
	void stop();

	void print_status();

private:
	static constexpr uint64_t SCHEDULE_INTERVAL_US = 100_ms;
	static constexpr int MAX_MOTORS = 12;
	static constexpr int MAX_OUTPUTS = 16;
	static constexpr int NUM_OUTPUT_GROUPS = 2; // PWM_MAIN, PWM_AUX
	static constexpr int FUNC_MOTOR1 = 101;

	void Run() override;
	void parametersUpdate(bool force = false);

	/**
	 * Read PWM_MAIN_FUNCx and PWM_AUX_FUNCx parameters to build
	 * output channel → motor index mapping per actuator_outputs instance.
	 * Motor1 (func 101) → index 0, Motor2 (func 102) → index 1, etc.
	 */
	void updateFunctionMapping();

	/**
	 * For each active motor, update its phase voltage LPF and write mot_voltage_cv
	 * + mot_load_status into `out` based on configured warn/crit thresholds.
	 * No-op if v_bat is non-finite.
	 */
	void updateMotorVoltages(aviant_motors_s *out, float v_bat, const float (&duties)[MAX_MOTORS]);

	/**
	 * Update the anomalous current LPF from (current_a - predictCurrent(v_bat, ...))
	 * and write anomalous_current_ca + anomalous_current_status into `out`.
	 * No-op if v_bat or current_a is non-finite.
	 */
	void updateAnomalousCurrent(aviant_motors_s *out, float v_bat, float current_a,
				    const float (&duties)[MAX_MOTORS]);

	/**
	 * Translate raw PWM_MAIN / PWM_AUX actuator outputs into a per-motor duty
	 * cycle array (0..1), clamping each PWM to the calibrated ESC range
	 * [1000, 2000]us. Motors that are unmapped, idle, or have a non-finite
	 * PWM yield 0. Caller must zero-init duties_out before calling.
	 */
	void calculateDutyCycles(float (&duties_out)[MAX_MOTORS],
				 const actuator_outputs_s &pwm_main,
				 const actuator_outputs_s &pwm_aux) const;

	/**
	 * Sum predicted bus current across all mapped motors using the per-motor
	 * power-law model: I_bus_motor = D * I_ref * (V_bat * D / V_ref)^k.
	 * D = duty cycle %
	 * V_ref = arbitrary normalization voltage
	 * I_ref = fit model gain parameter
	 * k = fit model shape parameter
	 * Returns 0 if v_bat is non-finite.
	 */
	float predictCurrent(float v_bat, const float (&duties)[MAX_MOTORS]) const;

	// Maps output channel → motor index (0-based) per output group, -1 if not a motor
	int8_t _output_to_motor[NUM_OUTPUT_GROUPS][MAX_OUTPUTS] {};

	// Per-motor phase voltage low-pass filters
	AlphaFilter<float> _vpha_lpf[MAX_MOTORS];

	uORB::SubscriptionMultiArray<actuator_outputs_s, NUM_OUTPUT_GROUPS> _actuator_outputs_subs{ORB_ID::actuator_outputs};
	uORB::Subscription _battery_status_sub{ORB_ID::battery_status};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID::parameter_update, 1_s};

	uORB::Publication<aviant_motors_s> _aviant_motors_pub{ORB_ID::aviant_motors};

	AlphaFilter<float> _current_error_lpf;

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": thrust cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": thrust interval")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::AV_THR_MOT_TOP>) _param_av_thr_mot_top,
		(ParamInt<px4::params::AV_THR_MOT_PSH>) _param_av_thr_mot_psh,

		// Anomalous current model
		(ParamFloat<px4::params::AV_THR_TOP_IREF>) _param_av_thr_top_iref,
		(ParamFloat<px4::params::AV_THR_TOP_K>) _param_av_thr_top_k,
		(ParamFloat<px4::params::AV_THR_TOP_VREF>) _param_av_thr_top_vref,
		(ParamFloat<px4::params::AV_THR_PSH_IREF>) _param_av_thr_psh_iref,
		(ParamFloat<px4::params::AV_THR_PSH_K>) _param_av_thr_psh_k,
		(ParamFloat<px4::params::AV_THR_PSH_VREF>) _param_av_thr_psh_vref,
		(ParamFloat<px4::params::AV_THR_CUR_TAU>) _param_av_thr_cur_tau,
		(ParamFloat<px4::params::AV_THR_VPHA_TAU>) _param_av_thr_vpha_tau,

		// Anomalous current thresholds (percentage of predicted + minimum)
		(ParamFloat<px4::params::AV_THR_WARN_PCT>) _param_av_thr_warn_pct,
		(ParamFloat<px4::params::AV_THR_CRIT_PCT>) _param_av_thr_crit_pct,
		(ParamFloat<px4::params::AV_THR_CUR_MIN>) _param_av_thr_cur_min,

		// Top motor phase voltage
		(ParamFloat<px4::params::AV_THR_TOP_WRN_V>) _param_av_thr_top_wrn_v,
		(ParamFloat<px4::params::AV_THR_TOP_CRI_V>) _param_av_thr_top_cri_v,

		// Pusher motor phase voltage
		(ParamFloat<px4::params::AV_THR_PSH_WRN_V>) _param_av_thr_psh_wrn_v,
		(ParamFloat<px4::params::AV_THR_PSH_CRI_V>) _param_av_thr_psh_cri_v
	);
};

} // namespace aviant
