#pragma once

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <sys/types.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionMultiArray.hpp>

#include <uORB/topics/aviant_navigation.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/estimator_states.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/estimator_aid_source1d.h>
#include <uORB/topics/estimator_aid_source3d.h>
#include <uORB/topics/sensor_gps.h>

#include <systemlib/mavlink_log.h>

#include "drivers/device/device.h"

namespace aviant
{

using namespace time_literals;

class Navigation : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	Navigation();
	~Navigation() override;

	bool start();
	void stop();

	void print_status();

private:
	void Run() override;
	void parametersUpdate(bool force = false);

	void checkLocalPosition(aviant_navigation_s *out);
	void checkGnssInput(aviant_navigation_s *out);
	void checkRtkHeadingUsed(aviant_navigation_s *out, int estimator_instance);
	void checkGnssPosFused(aviant_navigation_s *out, int estimator_instance);
	void checkMagHealthy(aviant_navigation_s *out, int estimator_instance);
	void checkBaroHealthy(aviant_navigation_s *out, int estimator_instance);

	void checkMagHeadingOffset(aviant_navigation_s *out);

	int getEstimatorInstance();
	bool isTimedOut(const hrt_abstime &timestamp, uint64_t timeout_interval) const;
	static float calculateMagHeading(const estimator_states_s &states, const matrix::Vector3f &debiased_mag);

	static constexpr uint64_t EKF2_TOUT = 100_ms;
	static constexpr uint64_t EKF2_SLOW_TOUT = 1100_ms; // Some EKF2 topics are published at 1 Hz
	static constexpr uint64_t LPOS_TOUT = 100_ms;
	static constexpr uint64_t MAG_TOUT = 500_ms;
	static constexpr uint64_t GNSS_TOUT = 500_ms;
	static constexpr uint64_t BARO_TOUT = 500_ms;
	static constexpr uint64_t GNSS_HDG_TOUT = 500_ms;

	static constexpr float INNOVATION_CHECK_FAIL_THRESHOLD = 0.5f;

	static constexpr float MAG_HDG_WARN_DEG = 20.0f;
	static constexpr float MAG_HDG_FAIL_DEG = 45.0f;

	uORB::Subscription _vehicle_local_position_sub{ORB_ID::vehicle_local_position};
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID::vehicle_gps_position};
	uORB::Subscription _vehicle_status_sub{ORB_ID::vehicle_status};
	uORB::SubscriptionMultiArray<sensor_gps_s, 2> _sensor_gps_subs{ORB_ID::sensor_gps};

	uORB::Subscription _estimator_selector_status_sub{ORB_ID::estimator_selector_status};
	uORB::SubscriptionMultiArray<estimator_states_s> _estimator_states_subs{ORB_ID::estimator_states};
	uORB::SubscriptionMultiArray<estimator_status_s> _estimator_status_subs{ORB_ID::estimator_status};
	uORB::SubscriptionMultiArray<estimator_status_flags_s> _estimator_status_flags_subs{ORB_ID::estimator_status_flags};
	uORB::SubscriptionMultiArray<estimator_aid_source1d_s> _estimator_aid_src_baro_hgt_subs{ORB_ID::estimator_aid_src_baro_hgt};
	uORB::SubscriptionMultiArray<estimator_aid_source3d_s> _estimator_aid_src_mag_subs{ORB_ID::estimator_aid_src_mag};
	uORB::SubscriptionMultiArray<estimator_aid_source1d_s> _estimator_aid_src_gnss_yaw_subs{ORB_ID::estimator_aid_src_gnss_yaw};
	uORB::SubscriptionMultiArray<estimator_aid_source3d_s> _estimator_aid_src_gnss_pos_subs{ORB_ID::estimator_aid_src_gnss_pos};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID::parameter_update, 1_s};

	uORB::Publication<aviant_navigation_s> _aviant_navigation_pub{ORB_ID::aviant_navigation};

	orb_advert_t _mavlink_log_pub{nullptr};

	// Each latches true the first time its threshold is crossed, so the operator is told once.
	bool _mag_hdg_warn_msg_sent{false};
	bool _mag_hdg_fail_msg_sent{false};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": interval")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::AV_NAV_MAG_ITR>) _params_av_nav_mag_itr
	);
};

} // namespace aviant
