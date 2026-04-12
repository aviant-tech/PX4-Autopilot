#include "Temperature.hpp"

#include <math.h>
#include <stdint.h>
#include <mathlib/mathlib.h>

namespace aviant
{

using namespace time_literals;

Temperature::Temperature() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

Temperature::~Temperature()
{
	stop();
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool Temperature::start()
{
	parametersUpdate(true);

	ScheduleOnInterval(1_s);
	return true;
}

void Temperature::stop()
{
	px4::ScheduledWorkItem::Deinit();
}

void Temperature::Run()
{
	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	parametersUpdate();

	// Default to INT8_MIN so consumers that miss the STATE_INACTIVE flag
	// still see an obviously-bogus temperature rather than a plausible 0 degC.
	aviant_temperature_fc_s out = {};
	out.battery_internal = INT8_MIN;
	out.airspeed_internal = INT8_MIN;
	out.imu_internal = INT8_MIN;
	out.baro_internal = INT8_MIN;

	readBattery(&out);
	readAirspeed(&out);
	readImu(&out);
	readBaro(&out);

	out.timestamp = hrt_absolute_time();
	_aviant_temperature_fc_pub.publish(out);

	perf_end(_loop_perf);
}

void Temperature::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
}

void Temperature::parametersUpdate(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
	}
}

float Temperature::selectRepresentative(float min_temp, float max_temp,
					float landim_low, float lasap_low,
					float lasap_high, float landim_high) const
{
	if (max_temp >= landim_high) {
		return max_temp;

	} else if (min_temp <= landim_low) {
		return min_temp;

	} else if (max_temp >= lasap_high) {
		return max_temp;

	} else if (min_temp <= lasap_low) {
		return min_temp;
	}

	// Both within nominal range — blend toward whichever side the midpoint is closer to
	const float avg = (min_temp + max_temp) * 0.5f;
	return math::interpolate(avg, lasap_low, lasap_high, min_temp, max_temp);
}

uint8_t Temperature::evaluateState(float temperature, float landim_low, float lasap_low,
				   float lasap_high, float landim_high) const
{
	if (!PX4_ISFINITE(temperature)) {
		return aviant_temperature_fc_s::STATE_INACTIVE;
	}

	if (temperature <= landim_low || temperature >= landim_high) {
		return aviant_temperature_fc_s::STATE_CRITICAL;
	}

	if (temperature <= lasap_low || temperature >= lasap_high) {
		return aviant_temperature_fc_s::STATE_WARNING;
	}

	return aviant_temperature_fc_s::STATE_NOMINAL;
}

void Temperature::readBattery(aviant_temperature_fc_s *out)
{
	float min_temp = INFINITY;
	float max_temp = -INFINITY;

	int count = 0;

	for (auto &sub : _battery_status_subs) {
		battery_status_s battery_status;

		if (sub.copy(&battery_status) && hrt_elapsed_time(&battery_status.timestamp) <= SENSOR_TOUT
		    && PX4_ISFINITE(battery_status.temperature)) {
			min_temp = math::min(min_temp, battery_status.temperature);
			max_temp = math::max(max_temp, battery_status.temperature);

			count++;
		}
	}

	if (count == 0) {
		return;
	}

	const float repr = selectRepresentative(min_temp, max_temp,
						_param_av_t_bat_ll.get(), _param_av_t_bat_l.get(),
						_param_av_t_bat_h.get(), _param_av_t_bat_hh.get());
	out->battery_internal = (int8_t)roundf(math::constrain(repr, (float)INT8_MIN, (float)INT8_MAX));
	out->battery_internal_state = evaluateState(repr,
				      _param_av_t_bat_ll.get(), _param_av_t_bat_l.get(),
				      _param_av_t_bat_h.get(), _param_av_t_bat_hh.get());
}

void Temperature::readAirspeed(aviant_temperature_fc_s *out)
{
	float min_temp = INFINITY;
	float max_temp = -INFINITY;

	int count = 0;

	for (auto &sub : _differential_pressure_subs) {
		differential_pressure_s diff_press;

		if (sub.copy(&diff_press) && hrt_elapsed_time(&diff_press.timestamp) <= SENSOR_TOUT
		    && PX4_ISFINITE(diff_press.temperature)) {
			min_temp = math::min(min_temp, diff_press.temperature);
			max_temp = math::max(max_temp, diff_press.temperature);

			count++;
		}
	}

	if (count == 0) {
		return;
	}

	const float repr = selectRepresentative(min_temp, max_temp,
						_param_av_t_arspd_ll.get(), _param_av_t_arspd_l.get(),
						_param_av_t_arspd_h.get(), _param_av_t_arspd_hh.get());
	out->airspeed_internal = (int8_t)roundf(math::constrain(repr, (float)INT8_MIN, (float)INT8_MAX));
	out->airspeed_internal_state = evaluateState(repr,
				       _param_av_t_arspd_ll.get(), _param_av_t_arspd_l.get(),
				       _param_av_t_arspd_h.get(), _param_av_t_arspd_hh.get());
}

void Temperature::readImu(aviant_temperature_fc_s *out)
{
	float min_temp = INFINITY;
	float max_temp = -INFINITY;

	int count = 0;

	for (auto &sub : _vehicle_imu_status_subs) {
		vehicle_imu_status_s imu_status;

		if (sub.copy(&imu_status) && hrt_elapsed_time(&imu_status.timestamp) <= SENSOR_TOUT
		    && PX4_ISFINITE(imu_status.temperature_accel)) {
			min_temp = math::min(min_temp, imu_status.temperature_accel);
			max_temp = math::max(max_temp, imu_status.temperature_accel);

			count++;
		}
	}

	if (count == 0) {
		return;
	}

	const float repr = selectRepresentative(min_temp, max_temp,
						_param_av_t_imu_ll.get(), _param_av_t_imu_l.get(),
						_param_av_t_imu_h.get(), _param_av_t_imu_hh.get());
	out->imu_internal = (int8_t)roundf(math::constrain(repr, (float)INT8_MIN, (float)INT8_MAX));
	out->imu_internal_state = evaluateState(repr,
						_param_av_t_imu_ll.get(), _param_av_t_imu_l.get(),
						_param_av_t_imu_h.get(), _param_av_t_imu_hh.get());
}

void Temperature::readBaro(aviant_temperature_fc_s *out)
{
	float min_temp = INFINITY;
	float max_temp = -INFINITY;

	int count = 0;

	for (auto &sub : _sensor_baro_subs) {
		sensor_baro_s sensor_baro;

		if (sub.copy(&sensor_baro) && hrt_elapsed_time(&sensor_baro.timestamp) <= SENSOR_TOUT
		    && PX4_ISFINITE(sensor_baro.temperature)) {
			min_temp = math::min(min_temp, sensor_baro.temperature);
			max_temp = math::max(max_temp, sensor_baro.temperature);

			count++;
		}
	}

	if (count == 0) {
		return;
	}

	const float repr = selectRepresentative(min_temp, max_temp,
						_param_av_t_baro_ll.get(), _param_av_t_baro_l.get(),
						_param_av_t_baro_h.get(), _param_av_t_baro_hh.get());
	out->baro_internal = (int8_t)roundf(math::constrain(repr, (float)INT8_MIN, (float)INT8_MAX));
	out->baro_internal_state = evaluateState(repr,
				   _param_av_t_baro_ll.get(), _param_av_t_baro_l.get(),
				   _param_av_t_baro_h.get(), _param_av_t_baro_hh.get());
}

} // namespace aviant
