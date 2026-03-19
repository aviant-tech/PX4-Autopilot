#ifndef AVIANT_DETAILED_FC_STATE_HPP
#define AVIANT_DETAILED_FC_STATE_HPP

#include <mavlink.h>
#include <mavlink/mavlink_stream.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_status.h>

class MavlinkStreamAviantDetailedFcState : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamAviantDetailedFcState(mavlink); }

	static constexpr const char *get_name_static() { return "AVIANT_DETAILED_FC_STATE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_AVIANT_DETAILED_FC_STATE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	bool const_rate() override { return true; }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_AVIANT_DETAILED_FC_STATE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamAviantDetailedFcState(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};

	bool send() override
	{
		if (_mavlink->get_free_tx_buf() < get_size()) {
			return false;
		}

		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		actuator_armed_s actuator_armed{};
		_actuator_armed_sub.copy(&actuator_armed);

		timespec tv;
		px4_clock_gettime(CLOCK_REALTIME, &tv);

		const uint32_t time_boot_ms = hrt_absolute_time() / 1000;
		const uint64_t time_unix_usec = (uint64_t)tv.tv_sec * 1000000 + tv.tv_nsec / 1000;

		const uint8_t armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)
				      ? MAV_BOOL_TRUE : MAV_BOOL_FALSE;

		const uint8_t flight_termination = (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_TERMINATION)
						   ? MAV_BOOL_TRUE : MAV_BOOL_FALSE;

		mavlink_msg_aviant_detailed_fc_state_send(
			_mavlink->get_channel(),
			time_boot_ms,
			time_unix_usec,
			armed,
			flight_termination);

		return true;
	}
};

#endif // AVIANT_DETAILED_FC_STATE_HPP
