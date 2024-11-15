#include "../PreFlightCheck.hpp"

#include <drivers/drv_hrt.h>
#include <px4_defines.h>
#include <systemlib/mavlink_log.h>
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>

using namespace time_literals;

bool PreFlightCheck::dynamicActuatorCheck(orb_advert_t *mavlink_log_pub, const uint8_t instance,
		const bool is_mandatory, const bool report_fail)
{
	uORB::SubscriptionData<battery_status_s> battery_status_sub{ORB_ID(battery_status), instance};
	battery_status_sub.update();
	const battery_status_s &battery_status_data = battery_status_sub.get();

	bool valid = (hrt_elapsed_time(&battery_status_data.timestamp) < 5_s);
	bool good = battery_status_data.dynamic_actuator >= 0.f;

	if (!valid && report_fail) {
		mavlink_log_critical(mavlink_log_pub, "Preflight Fail: no valid data from battery %u", instance);

	} else if (!good && report_fail) {
		mavlink_log_critical(mavlink_log_pub, "Preflight Fail: dynamic actuator error from battery %u", instance);
	}

	return (valid && good) || !is_mandatory;
}
