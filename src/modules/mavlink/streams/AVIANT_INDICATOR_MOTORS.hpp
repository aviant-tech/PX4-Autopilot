#ifndef AVIANT_INDICATOR_MOTORS_HPP
#define AVIANT_INDICATOR_MOTORS_HPP

#include <mavlink.h>
#include <mavlink/mavlink_stream.h>
#include <uORB/topics/aviant_motors.h>

class MavlinkStreamAviantIndicatorMotors : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamAviantIndicatorMotors(mavlink); }

	static constexpr const char *get_name_static() { return "AVIANT_INDICATOR_MOTORS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_AVIANT_INDICATOR_MOTORS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	bool const_rate() override { return true; }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_AVIANT_INDICATOR_MOTORS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamAviantIndicatorMotors(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _aviant_motors_sub{ORB_ID::aviant_motors};

	bool send() override
	{
		aviant_motors_s msg;

		if (_mavlink->get_free_tx_buf() >= get_size() && _aviant_motors_sub.update(&msg)) {

			mavlink_msg_aviant_indicator_motors_send(
				_mavlink->get_channel(),
				msg.mot_voltage_cv,
				msg.mot_load_status,
				msg.anomalous_current_ca,
				msg.anomalous_current_status
			);

			return true;
		}

		return false;
	}
};

#endif // AVIANT_INDICATOR_MOTORS_HPP
