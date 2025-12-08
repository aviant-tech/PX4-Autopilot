#ifndef AVIANT_NAV_HPP
#define AVIANT_NAV_HPP

#include <mavlink.h>
#include <mavlink/mavlink_stream.h>
#include <uORB/topics/aviant_navigation.h>

class MavlinkStreamAviantNav : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamAviantNav(mavlink); }

	static constexpr const char *get_name_static() { return "AVIANT_NAV"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_AVIANT_NAV; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	bool const_rate() override { return true; }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_AVIANT_NAV_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamAviantNav(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _aviant_navigation_sub{ORB_ID::aviant_navigation};

	bool send() override
	{
		aviant_navigation_s msg;

		if (_mavlink->get_free_tx_buf() >= get_size()  && _aviant_navigation_sub.update(&msg)) {

			mavlink_msg_aviant_nav_send(_mavlink->get_channel(), msg.accuracy, msg.redundancy);

			return true;
		}

		return false;
	}
};

#endif // AVIANT_NAV_HPP
