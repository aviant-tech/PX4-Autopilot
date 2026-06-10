#include "Navigation.hpp"
#include "px4_platform_common/defines.h"

#include <cstdarg>
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

	ScheduleOnInterval(100_ms);
	return true;
}

void Navigation::stop()
{
	px4::ScheduledWorkItem::Deinit();
}

void Navigation::Run()
{
	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	parametersUpdate();

	// Start with a clean state. Default values matches "UNKNOWN" enums.
	aviant_navigation_s out = {};
	out.mag_heading = NAN; // Initialize to NaN, avoid confustion with valid 0 rad heading

	checkLocalPosition(&out);
	checkGnssInput(&out);

	// Only run estimator check on the active instance, to save computation.
	const int ekf_idx = getEstimatorInstance();

	if (ekf_idx >= 0) {
		checkRtkHeadingUsed(&out, ekf_idx);
		checkGnssPosFused(&out, ekf_idx);
		checkMagHealthy(&out, ekf_idx);
		checkBaroHealthy(&out, ekf_idx);

	} else {
		// No valid EKF instance
	}

	// Start out with nominal values, dowgrade based on checks not passing
	out.redundancy = aviant_navigation_s::REDUNDANCY_REDUNDANT;
	out.accuracy = aviant_navigation_s::ACCURACY_ACCURATE;

	if (!out.lpos_valid) {
		out.redundancy = math::max(out.redundancy, aviant_navigation_s::REDUNDANCY_FAILED);
		out.accuracy = math::max(out.accuracy, aviant_navigation_s::ACCURACY_FAILED);
	}

	if (!(out.gnss_absolute_rtk_float && out.ekf_gnss_pos_fused_recently)) {
		out.accuracy = math::max(out.accuracy, aviant_navigation_s::ACCURACY_APPROXIMATE);
	}

	if (!(out.gnss_fallback_available && out.mag_fused_recently && out.mag_innovations_ok &&
	      out.baro_fused_recently && out.ekf_rtk_heading_used)) {
		out.redundancy = math::max(out.redundancy, aviant_navigation_s::REDUNDANCY_SINGLE);
	}

	out.timestamp = hrt_absolute_time();
	_aviant_navigation_pub.publish(out);

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
	}
}

void Navigation::checkLocalPosition(aviant_navigation_s *out)
{
	vehicle_local_position_s vehicle_local_position;

	if (!_vehicle_local_position_sub.copy(&vehicle_local_position)
	    || isTimedOut(vehicle_local_position.timestamp, LPOS_TOUT)) {
		return;
	}

	if (vehicle_local_position.xy_valid && vehicle_local_position.z_valid) {
		out->lpos_valid = true;
	}
}

void Navigation::checkGnssInput(aviant_navigation_s *out)
{
	// 1: Check the currently selected GNSS from vehicle_gps_position
	sensor_gps_s selected_gnss{};

	if (!_vehicle_gps_position_sub.copy(&selected_gnss) || isTimedOut(selected_gnss.timestamp, GNSS_TOUT)) {
		return;
	}

	const auto fix_type = selected_gnss.fix_type;

	if (fix_type == sensor_gps_s::FIX_TYPE_RTK_FLOAT || fix_type == sensor_gps_s::FIX_TYPE_RTK_FIXED) {
		out->gnss_absolute_rtk_float = true;
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
		out->gnss_fallback_available = true;
	}
}

int Navigation::getEstimatorInstance()
{

	estimator_selector_status_s estimator_selector_status;

	if (!_estimator_selector_status_sub.copy(&estimator_selector_status)
	    || isTimedOut(estimator_selector_status.timestamp, EKF2_SLOW_TOUT)) {
		return -1;
	}

	const int ekf_idx = estimator_selector_status.primary_instance;

	if (ekf_idx < 0 || ekf_idx >= ORB_MULTI_MAX_INSTANCES) {
		return -1;
	}

	return ekf_idx;
}

void Navigation::checkRtkHeadingUsed(aviant_navigation_s *out, int estimator_instance)
{
	estimator_status_flags_s estimator_status_flags;

	if (!_estimator_status_flags_subs[estimator_instance].copy(&estimator_status_flags)
	    || isTimedOut(estimator_status_flags.timestamp, EKF2_SLOW_TOUT)) {
		return;
	}

	out->ekf_rtk_heading_used = estimator_status_flags.cs_gps_yaw;
}

void Navigation::checkBaroHealthy(aviant_navigation_s *out, int estimator_instance)
{
	estimator_aid_source1d_s estimator_aid_src_baro_hgt;

	if (!_estimator_aid_src_baro_hgt_subs[estimator_instance].copy(&estimator_aid_src_baro_hgt)) {
		return;
	}

	// If the barometer has been fused recently, consider it a healthy backup
	out->baro_fused_recently = !isTimedOut(estimator_aid_src_baro_hgt.time_last_fuse, BARO_TOUT);
}

void Navigation::checkMagHealthy(aviant_navigation_s *out, int estimator_instance)
{
	estimator_aid_source3d_s estimator_aid_src_mag;

	if (!_estimator_aid_src_mag_subs[estimator_instance].copy(&estimator_aid_src_mag)) {
		return;
	}

	// If the magnetometer has been fused recently, we know that biases are being estimated
	out->mag_fused_recently = !isTimedOut(estimator_aid_src_mag.time_last_fuse, MAG_TOUT);

	// We also want to ensure that innovations aren't close to gate threshold
	// So that the mag won't be rejected immediately after switching over to it
	out->mag_innovations_ok = true;

	if (_params_av_nav_mag_itr.get() >= 0.0f) {
		// Innovation ratio check disabled
		for (auto ratio : estimator_aid_src_mag.test_ratio) {
			if (ratio >= _params_av_nav_mag_itr.get()) {
				out->mag_innovations_ok = false;
				break;
			}
		}
	}

	// For ease of debugging, calculate and publish 2D mag heading with current bias estimates
	estimator_states_s estimator_states;

	if (!_estimator_states_subs[estimator_instance].copy(&estimator_states)
	    || isTimedOut(estimator_states.timestamp, EKF2_TOUT)) {
		return;
	}

	const matrix::Vector3f debiased_mag{estimator_aid_src_mag.observation[0], estimator_aid_src_mag.observation[1], estimator_aid_src_mag.observation[2]};
	out->mag_heading = calculateMagHeading(estimator_states, debiased_mag);
}

void Navigation::checkGnssPosFused(aviant_navigation_s *out, int estimator_instance)
{
	estimator_aid_source3d_s estimator_aid_src_gnss_pos;

	if (!_estimator_aid_src_gnss_pos_subs[estimator_instance].copy(&estimator_aid_src_gnss_pos)) {
		return;
	}

	out->ekf_gnss_pos_fused_recently = !isTimedOut(estimator_aid_src_gnss_pos.time_last_fuse, GNSS_TOUT);
}

float Navigation::calculateMagHeading(const estimator_states_s &states, const matrix::Vector3f &debiased_mag)
{
	// Modelled after Ekf::resetMagHeading
	// see state.h in EKF for state index definitions

	static_assert(sizeof(states.states) / sizeof(states.states[0]) == 24,
		      "estimator_states_s size does not match expected EKF state size");
	matrix::Vector3f mag_I{states.states[16], states.states[17], states.states[18]};
	matrix::Vector3f mag_bias{states.states[19], states.states[20], states.states[21]};
	matrix::Quaternionf attitude{states.states[0], states.states[1], states.states[2], states.states[3]};

	// rotate the magnetometer measurements into earth frame using a zero yaw angle
	const matrix::Dcmf R_to_earth = math::Utilities::updateYawInRotMat(0.f, matrix::Dcmf(attitude));

	// the angle of the projection onto the horizontal gives the heading angle
	//
	// we add back the bias that was previously subtracted. otherwise the
	// estimated bias could hide bad calibrations. the estimated bias
	// correction is not guaranteed to be accurate in all poses.
	const matrix::Vector3f mag_earth_pred = R_to_earth * (debiased_mag + mag_bias);

	// note: we use the estimated inclination always, unlike EKF::getMagDeclination()
	// this is for simplicity, since we expect mag to be aligned for most of the fligh
	const float declination = atan2f(mag_I(1), mag_I(0));

	// calculate the observed heading angle
	return -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + declination;
}

bool Navigation::isTimedOut(const hrt_abstime &timestamp, uint64_t timeout_interval) const
{
	// Note that, due to underflow, this returns false if timestamp is in the future,
	// as long as timeout_interval isn't ridiculously large.
	// This is acceptable behavior for our use case.
	return  hrt_absolute_time() - timestamp > timeout_interval;
}

} // namespace aviant
