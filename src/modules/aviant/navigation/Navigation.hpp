#pragma once

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <sys/types.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/aviant_navigation.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/estimator_states.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/estimator_aid_source1d.h>
#include <uORB/topics/estimator_aid_source3d.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_combined.h>


#include "drivers/device/device.h"

#include <uORB/SubscriptionMultiArray.hpp>
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

	void checkLocalPosition();
	void checkGnssInput();
	void compareGnssAndMagHeading(int estimator_instance);
	void checkRtkHeadingUsed(int estimator_instance);
	void checkBaroHealthy(int estimator_instance);


	int getEstimatorInstance();

	static constexpr uint64_t EKF2_TOUT = 100_ms;
	static constexpr uint64_t LPOS_TOUT = 100_ms;
	static constexpr uint64_t MAG_TOUT = 100_ms;
	static constexpr uint64_t GNSS_TOUT = 500_ms;
	static constexpr uint64_t BARO_TOUT = 500_ms;
	static constexpr uint64_t GNSS_HDG_TOUT = 500_ms;

	static constexpr float INNOVATION_CHECK_FAIL_THRESHOLD = 0.5f;
	bool isTimedOut(const hrt_abstime &timestamp, uint64_t timeout_interval) const;
	static float calculateMagHeading(const estimator_states_s &states, const matrix::Vector3f &mag);
	static inline void setSeverityAtLeast(uint8_t &field, uint8_t severity);

	template<typename... Args>
	static bool allChecksPassed(Args... args)
	{
		// Avoid signed char promotion issues
		bool tmp[] = {(args)... };

		for (bool v : tmp) {
			if (!v) {
				return false;
			}
		}

		return true;
	}

	uORB::Subscription _vehicle_local_position_sub{ORB_ID::vehicle_local_position};
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID::vehicle_gps_position};


	uORB::Subscription _estimator_selector_status_sub{ORB_ID::estimator_selector_status};
	uORB::SubscriptionMultiArray<estimator_states_s> _estimator_states_subs{ORB_ID::estimator_states};
	uORB::SubscriptionMultiArray<estimator_status_s> _estimator_status_subs{ORB_ID::estimator_status};
	uORB::SubscriptionMultiArray<estimator_status_flags_s> _estimator_status_flags_subs{ORB_ID::estimator_status_flags};

	// GNSS checks
	uORB::SubscriptionMultiArray<sensor_gps_s, 2> _sensor_gps_subs{ORB_ID::sensor_gps};

	// Baro height comparison
	uORB::SubscriptionMultiArray<estimator_aid_source1d_s> _estimator_aid_src_baro_hgt_subs{ORB_ID::estimator_aid_src_baro_hgt};

	// Magnetometer / GNSS heading comparison
	uORB::SubscriptionMultiArray<estimator_aid_source3d_s> _estimator_aid_src_mag_subs{ORB_ID::estimator_aid_src_mag};
	uORB::SubscriptionMultiArray<estimator_aid_source1d_s> _estimator_aid_src_gnss_yaw_subs{ORB_ID::estimator_aid_src_gnss_yaw};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID::parameter_update, 1_s};

	uORB::Publication<aviant_navigation_s> _aviant_navigation_pub{ORB_ID::aviant_navigation};

	// For EKF replay testing, we use sensor_combined as timebase
	uORB::SubscriptionCallbackWorkItem _sensor_combined_sub{this, ORB_ID(sensor_combined)};
	bool _ekf_replay{false};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::AV_NAV_INT>) _params_av_nav_int,
		(ParamFloat<px4::params::AV_NAV_HDIFF_W>) _params_av_nav_hdiff_w_deg
	);
	float _heading_diff_warn{0.0f};

	aviant_navigation_s _out{};
	uint64_t _now{0};
	uint64_t _last_run{0};

	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

};
} // namespace aviant
