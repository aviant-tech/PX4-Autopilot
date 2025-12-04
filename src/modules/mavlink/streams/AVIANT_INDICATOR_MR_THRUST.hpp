#ifndef AVIANT_INDICATOR_MR_THRUST_HPP
#define AVIANT_INDICATOR_MR_THRUST_HPP

#include <mavlink.h>
#include <mavlink/mavlink_stream.h>
#include <uORB/topics/aviant_mr_thrust_indicator.h>

class MavlinkStreamAviantIndicatorMrThrust : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamAviantIndicatorMrThrust(mavlink); }

	static constexpr const char *get_name_static() { return "AVIANT_INDICATOR_MR_THRUST"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_AVIANT_INDICATOR_MR_THRUST; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	bool const_rate() override { return true; }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_AVIANT_INDICATOR_MR_THRUST_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamAviantIndicatorMrThrust(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _aviant_mr_thrust_indicator_sub{ORB_ID(aviant_mr_thrust_indicator)};

	bool send() override
	{
		aviant_mr_thrust_indicator_s msg;

		if (_mavlink->get_free_tx_buf() >= get_size()  && _aviant_mr_thrust_indicator_sub.update(&msg)) {

			mavlink_msg_aviant_indicator_mr_thrust_send(_mavlink->get_channel(),
					msg.status,
					msg.thrust_max,
					msg.thrust_max_bomi);

			return true;
		}

		return false;
	}
};

#endif // AVIANT_INDICATOR_MR_THRUST_HPP
