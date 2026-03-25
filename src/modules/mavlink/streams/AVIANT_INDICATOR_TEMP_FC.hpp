#ifndef AVIANT_INDICATOR_TEMP_FC_HPP
#define AVIANT_INDICATOR_TEMP_FC_HPP

#include <mavlink.h>
#include <mavlink/mavlink_stream.h>
#include <uORB/topics/aviant_temperature_fc.h>

class MavlinkStreamAviantIndicatorTempFc : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamAviantIndicatorTempFc(mavlink); }

	static constexpr const char *get_name_static() { return "AVIANT_INDICATOR_TEMP_FC"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_AVIANT_INDICATOR_TEMP_FC; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	bool const_rate() override { return true; }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_AVIANT_INDICATOR_TEMP_FC_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamAviantIndicatorTempFc(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _aviant_temperature_fc_sub{ORB_ID::aviant_temperature_fc};

	bool send() override
	{
		aviant_temperature_fc_s msg;

		if (_mavlink->get_free_tx_buf() >= get_size() && _aviant_temperature_fc_sub.update(&msg)) {

			mavlink_msg_aviant_indicator_temp_fc_send(
				_mavlink->get_channel(),
				msg.battery_internal_state,
				msg.battery_internal,
				msg.airspeed_internal_state,
				msg.airspeed_internal,
				msg.imu_internal_state,
				msg.imu_internal,
				msg.baro_internal_state,
				msg.baro_internal
			);

			return true;
		}

		return false;
	}
};

#endif // AVIANT_INDICATOR_TEMP_FC_HPP
