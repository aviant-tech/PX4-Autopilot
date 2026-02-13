#ifndef AVIANT_HEARTBEAT_HPP
#define AVIANT_HEARTBEAT_HPP

#include <mavlink.h>
#include <mavlink/mavlink_stream.h>

class MavlinkStreamAviantHeartbeat : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamAviantHeartbeat(mavlink); }

	static constexpr const char *get_name_static() { return "AVIANT_HEARTBEAT"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_AVIANT_HEARTBEAT; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	bool const_rate() override { return true; }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_AVIANT_HEARTBEAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamAviantHeartbeat(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	bool send() override
	{
		if (_mavlink->get_free_tx_buf() >= get_size()) {
			uint8_t status{0};
			mavlink_msg_aviant_heartbeat_send(_mavlink->get_channel(), status);

			return true;
		}

		return false;
	}
};

#endif // AVIANT_HEARTBEAT_HPP
