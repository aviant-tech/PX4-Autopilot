#include "Navigation.hpp"
#include "px4_platform_common/defines.h"

#include <mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

namespace aviant
{

using namespace time_literals;

Navigation::Navigation() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

Navigation::~Navigation()
{
	stop();
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool Navigation::start()
{
	parametersUpdate(true);

#ifdef CONFIG_ARCH_BOARD_PX4_SITL
	// For EKF replay testing, we use sensor_combined as timebase
	const char *replay_mode = getenv("replay_mode");

	if (replay_mode && strcmp(replay_mode, "ekf2") == 0) {
		_ekf_replay = true;
		PX4_INFO("Aviant navigation indicator running in EKF2 replay mode");
	}

#endif // CONFIG_ARCH_BOARD_PX4_SITL

	if (_ekf_replay) {
		_sensor_combined_sub = uORB::SubscriptionCallbackWorkItem(this, ORB_ID(sensor_combined));
		_sensor_combined_sub.registerCallback();
		// Schedule for regular execution
		ScheduleOnInterval(_params_av_nav_int.get() * 1_ms);

	} else {
		// Schedule based on sensor_combined timestamps in Run()

	}

	return true;
}

void Navigation::stop()
{
	px4::ScheduledWorkItem::Deinit();
}

void Navigation::Run()
{
	// For ekf2 replay testing, there isn't valid system time or scheduling intervals,
	// so pace the module based on sensor_combined timestamps.

	if (_ekf_replay) {
		sensor_combined_s sensor_combined;

		if (!_sensor_combined_sub.copy(&sensor_combined)) {
			return;
		}

		_now = sensor_combined.timestamp;

		if (_now - _last_run < _params_av_nav_int.get() * 1_ms) {
			// not enough time elapsed
			return;
		}

		_last_run = _now;

	} else {
		_now = hrt_absolute_time();
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	parametersUpdate();

	// Start with a clean state. Default values matches "UNKNOWN" enums.
	_out = {};
	_out.timestamp = _now;

	checkLocalPosition();
	checkGnssInput();

	// Only run estimator check on the active instance, to save computation.
	const int ekf_idx = getEstimatorInstance();

	if (ekf_idx >= 0) {
		checkRtkHeadingUsed(ekf_idx);
		checkBaroHealthy(ekf_idx);

	} else {
		// No valid EKF instance
	}

	// Start out with nominal values, dowgrade based on checks not passing
	_out.redundancy = aviant_navigation_s::REDUNDANCY_REDUNDANT;
	_out.accuracy = aviant_navigation_s::ACCURACY_ACCURATE;

	if (!allChecksPassed(_out.lpos_valid)) {
		setSeverityAtLeast(_out.redundancy, aviant_navigation_s::REDUNDANCY_FAILED);
		setSeverityAtLeast(_out.accuracy, aviant_navigation_s::ACCURACY_FAILED);
	}

	if (!allChecksPassed(_out.gnss_absolute_rtk_float, _out.ekf_gnss_pos_fused_recently)) {
		setSeverityAtLeast(_out.accuracy, aviant_navigation_s::ACCURACY_APPROXIMATE);
	}

	if (!allChecksPassed(_out.gnss_fallback_available, _out.mag_fused_recently, _out.mag_innovations_ok,
			     _out.baro_fused_recently, _out.ekf_rtk_heading_used)) {
		setSeverityAtLeast(_out.redundancy, aviant_navigation_s::REDUNDANCY_SINGLE);
	}

	_aviant_navigation_pub.publish(_out);
	_last_run = _now;
	perf_end(_loop_perf);
}

void Navigation::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
};

void Navigation::parametersUpdate(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		_heading_diff_warn = math::radians(_params_av_nav_hdiff_w_deg.get());
	}
}

void Navigation::checkLocalPosition()
{
	vehicle_local_position_s vehicle_local_position;

	if (!_vehicle_local_position_sub.copy(&vehicle_local_position)
	    || isTimedOut(vehicle_local_position.timestamp, LPOS_TOUT)) {
		return;
	}

	if (!(vehicle_local_position.xy_valid && vehicle_local_position.z_valid)) {
	} else {
		_out.lpos_valid = true;
	}
}

void Navigation::checkGnssInput()
{
	// 1: Check the currently selected GNSS from vehicle_gps_position
	sensor_gps_s selected_gnss{};

	if (!_vehicle_gps_position_sub.copy(&selected_gnss) || isTimedOut(selected_gnss.timestamp, GNSS_TOUT)) {
		return;
	}

	const auto fix_type = selected_gnss.fix_type;

	if (fix_type == sensor_gps_s::FIX_TYPE_RTK_FLOAT || fix_type == sensor_gps_s::FIX_TYPE_RTK_FIXED) {
		_out.gnss_absolute_rtk_float = true;
	}

	// 2: Verify two valid GNSS inputs are available
	sensor_gps_s mb_gnss{};

	if (!_sensor_gps_subs[0].copy(&mb_gnss) || isTimedOut(mb_gnss.timestamp, GNSS_TOUT)) {
		// Primary GNSS not available
		return;
	}

	sensor_gps_s rover_gnss{};

	if (!_sensor_gps_subs[1].copy(&rover_gnss) || isTimedOut(rover_gnss.timestamp, GNSS_TOUT)) {
		// Rover GNSS not available
		return;
	}

	// Only require 3D fix for redundancy purposes
	if (mb_gnss.fix_type >= sensor_gps_s::FIX_TYPE_3D && rover_gnss.fix_type >= sensor_gps_s::FIX_TYPE_3D) {
		_out.gnss_fallback_available = true;
	}
}

int Navigation::getEstimatorInstance()
{
	if (_ekf_replay) {
		// ekf replay only has one instance
		return 0;
	}

	estimator_selector_status_s estimator_selector_status;

	if (!_estimator_selector_status_sub.copy(&estimator_selector_status)
	    || isTimedOut(estimator_selector_status.timestamp, EKF2_TOUT)) {
		return -1;
	}

	const int ekf_idx = estimator_selector_status.primary_instance;

	if (ekf_idx < 0 || ekf_idx >= ORB_MULTI_MAX_INSTANCES) {
		return -1;
	}

	return ekf_idx;
}

void Navigation::checkRtkHeadingUsed(int estimator_instance)
{
	estimator_status_flags_s estimator_status_flags;

	if (!_estimator_status_flags_subs[estimator_instance].copy(&estimator_status_flags)
	    || isTimedOut(estimator_status_flags.timestamp, EKF2_TOUT)) {
		return;
	}

	_out.ekf_rtk_heading_used = estimator_status_flags.cs_gps_yaw;
}

/*

Ater writing this, I realize that it's probably better to just check the innovation check,
we want to know "are we close to EKF not accepting mag/gnss heading", not just "are the headings close".

void Navigation::compareGnssAndMagHeading(int estimator_instance){
	estimator_aid_source3d_s estimator_aid_src_mag;
	if (!_estimator_aid_src_mag_subs[estimator_instance].copy(&estimator_aid_src_mag) || isTimedOut(estimator_aid_src_mag.timestamp, MAG_TOUT)) {
		_out.accuracy_status = math::max(_out.accuracy_status, aviant_navigation_s::NAV_STATUS_WARNING);
		return;
	}

	estimator_aid_source1d_s estimator_aid_src_gnss_yaw;
	if (!_estimator_aid_src_gnss_yaw_subs[estimator_instance].copy(&estimator_aid_src_gnss_yaw) || isTimedOut(estimator_aid_src_gnss_yaw.timestampm, GNSS_HDG_TOUT)) {
		_out.accuracy_status = math::max(_out.accuracy_status, aviant_navigation_s::NAV_STATUS_WARNING);
		return;
	}

	estimator_states_s estimator_states;
	if (!_estimator_states_subs[estimator_instance].copy(&estimator_states) || isTimedOut(estimator_states.timestamp, EKF2_TOUT)) {
		_out.accuracy_status = math::max(_out.accuracy_status, aviant_navigation_s::NAV_STATUS_WARNING);
		return;
	}

	const matrix::Vector3f mag{estimator_aid_src_mag.observation[0], estimator_aid_src_mag.observation[1], estimator_aid_src_mag.observation[2]};
	const float mag_heading = CalculateMagHeading(estimator_states, mag);
	_out.mag_heading = mag_heading;

	// Calculate heading difference
	float heading_diff = matrix::wrap_pi(mag_heading - estimator_aid_src_gnss_yaw.observation);
	_out.heading_difference = heading_diff;
	if (_heading_diff_warn > 0.0f && fabsf(heading_diff) > _heading_diff_warn) {
		_out.accuracy_status = math::max(_out.accuracy_status, aviant_navigation_s::NAV_STATUS_WARNING);
	} else {
		_out.mag_matches_rtk_heading = true;
	}
}
*/

void Navigation::checkBaroHealthy(int estimator_instance)
{
	estimator_aid_source1d_s estimator_aid_src_baro_hgt;

	if (!_estimator_aid_src_baro_hgt_subs[estimator_instance].copy(&estimator_aid_src_baro_hgt)) {
		return;
	}

	// If the barometer has been fused recently, consider it a healthy backup
	_out.baro_fused_recently = !isTimedOut(estimator_aid_src_baro_hgt.time_last_fuse, BARO_TOUT);
}

float Navigation::calculateMagHeading(const estimator_states_s &states, const matrix::Vector3f &mag)
{
	// Modelled after Ekf::resetMagHeading

	// see state.h in EKF for state index definitions
	matrix::Vector3f mag_I{states.states[16], states.states[17], states.states[18]};
	matrix::Vector3f mag_B{states.states[19], states.states[20], states.states[21]};
	matrix::Quaternionf attitude{states.states[0], states.states[1], states.states[2], states.states[3]};

	// rotate the magnetometer measurements into earth frame using a zero yaw angle
	const matrix::Dcmf R_to_earth = math::Utilities::updateYawInRotMat(0.f, matrix::Dcmf(attitude));

	// the angle of the projection onto the horizontal gives the heading angle
	const matrix::Vector3f mag_earth_pred = R_to_earth * (mag - mag_B);

	// note: we use the estimated inclination always, unlike EKF::getMagDeclination()
	// this is for simplicity, since we expect mag to be aligned for most of the fligh
	const float declination = atan2f(mag_I(1), mag_I(0));

	// calculate the observed heading angle
	return -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + declination;
}

bool Navigation::isTimedOut(const hrt_abstime &timestamp, uint64_t timeout_interval) const
{
	if (_now - timestamp > timeout_interval) {
		return true;
	}

	return false;
}

inline void Navigation::setSeverityAtLeast(uint8_t &field, uint8_t severity)
{
	if (severity > field) {
		field = severity;
	}
}
} // namespace aviant
