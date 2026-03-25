#pragma once

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionMultiArray.hpp>

#include <uORB/topics/aviant_temperature_fc.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/vehicle_imu_status.h>
#include <uORB/topics/sensor_baro.h>

namespace aviant
{

using namespace time_literals;

class Temperature : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	Temperature();
	~Temperature() override;

	bool start();
	void stop();

	void print_status();

private:
	void Run() override;
	void parametersUpdate(bool force = false);

	/**
	 * Select a single representative temperature from min/max sensor readings.
	 *
	 * We try to always show the most severe temperature with a single number,
	 * while prioritizing high temperatures, and avoiding discontinuous jumps
	 * when we breach a limit. This is impossible to achieve perfectly with very
	 * differing measurements, but this approximation probably works ok.
	 */
	float selectRepresentative(float min_temp, float max_temp,
				   float landim_low, float lasap_low,
				   float lasap_high, float landim_high) const;

	uint8_t evaluateState(float temperature, float landim_low, float lasap_low,
			      float lasap_high, float landim_high) const;

	void readBattery(aviant_temperature_fc_s *out);
	void readAirspeed(aviant_temperature_fc_s *out);
	void readImu(aviant_temperature_fc_s *out);
	void readBaro(aviant_temperature_fc_s *out);

	static constexpr uint64_t SENSOR_TOUT = 3_s;

	uORB::SubscriptionMultiArray<battery_status_s> _battery_status_subs{ORB_ID::battery_status};
	uORB::SubscriptionMultiArray<differential_pressure_s> _differential_pressure_subs{ORB_ID::differential_pressure};
	uORB::SubscriptionMultiArray<vehicle_imu_status_s> _vehicle_imu_status_subs{ORB_ID::vehicle_imu_status};
	uORB::SubscriptionMultiArray<sensor_baro_s> _sensor_baro_subs{ORB_ID::sensor_baro};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID::parameter_update, 1_s};

	uORB::Publication<aviant_temperature_fc_s> _aviant_temperature_fc_pub{ORB_ID::aviant_temperature_fc};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": temp cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": temp interval")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::AV_T_BAT_LL>) _param_av_t_bat_ll,
		(ParamFloat<px4::params::AV_T_BAT_L>) _param_av_t_bat_l,
		(ParamFloat<px4::params::AV_T_BAT_H>) _param_av_t_bat_h,
		(ParamFloat<px4::params::AV_T_BAT_HH>) _param_av_t_bat_hh,

		(ParamFloat<px4::params::AV_T_ARSPD_LL>) _param_av_t_arspd_ll,
		(ParamFloat<px4::params::AV_T_ARSPD_L>) _param_av_t_arspd_l,
		(ParamFloat<px4::params::AV_T_ARSPD_H>) _param_av_t_arspd_h,
		(ParamFloat<px4::params::AV_T_ARSPD_HH>) _param_av_t_arspd_hh,

		(ParamFloat<px4::params::AV_T_IMU_LL>) _param_av_t_imu_ll,
		(ParamFloat<px4::params::AV_T_IMU_L>) _param_av_t_imu_l,
		(ParamFloat<px4::params::AV_T_IMU_H>) _param_av_t_imu_h,
		(ParamFloat<px4::params::AV_T_IMU_HH>) _param_av_t_imu_hh,

		(ParamFloat<px4::params::AV_T_BARO_LL>) _param_av_t_baro_ll,
		(ParamFloat<px4::params::AV_T_BARO_L>) _param_av_t_baro_l,
		(ParamFloat<px4::params::AV_T_BARO_H>) _param_av_t_baro_h,
		(ParamFloat<px4::params::AV_T_BARO_HH>) _param_av_t_baro_hh
	);
};

} // namespace aviant
