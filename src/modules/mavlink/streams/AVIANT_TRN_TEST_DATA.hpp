/****************************************************************************
 *
 *   Copyright (c) 2024 Aviant. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef AVIANT_TRN_TEST_DATA_HPP
#define AVIANT_TRN_TEST_DATA_HPP

#include <uORB/topics/estimator_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>

class MavlinkStreamAviantTrnTestData : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamAviantTrnTestData(mavlink); }

	static constexpr const char *get_name_static() { return "AVIANT_TRN_TEST_DATA"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_AVIANT_TRN_TEST_DATA; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return (_gnss_denied_instance >= 0) ? MAVLINK_MSG_ID_AVIANT_TRN_TEST_DATA_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	static constexpr uint8_t MAX_INSTANCES = 9;

	explicit MavlinkStreamAviantTrnTestData(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	int8_t _gnss_denied_instance{-1};

	uORB::Subscription _local_pos_sub{ORB_ID(estimator_local_position)};
	uORB::Subscription _attitude_sub{ORB_ID(estimator_attitude)};

	void discoverGnssDeniedInstance()
	{
		// Search through estimator_status instances to find a GNSS-denied one
		for (uint8_t i = 0; i < MAX_INSTANCES; i++) {
			uORB::Subscription status_sub{ORB_ID(estimator_status), i};
			estimator_status_s status;

			if (status_sub.copy(&status)) {
				if (status.pos_est_mode == estimator_status_s::POS_EST_MODE_GNSS_DENIED) {
					_gnss_denied_instance = i;

					// Subscribe to this instance's local position and attitude
					_local_pos_sub = uORB::Subscription{ORB_ID(estimator_local_position), i};
					_attitude_sub = uORB::Subscription{ORB_ID(estimator_attitude), i};

					PX4_INFO("AVIANT_TRN_TEST_DATA: Found GNSS-denied EKF instance %d", i);
					return;
				}
			}
		}
	}

	bool send() override
	{
		// Retry discovery until the GNSS-denied instance is found
		if (_gnss_denied_instance < 0) {
			discoverGnssDeniedInstance();

			if (_gnss_denied_instance < 0) {
				return false;
			}
		}

		vehicle_local_position_s local_pos;
		vehicle_attitude_s attitude;

		if (_local_pos_sub.update(&local_pos)) {
			_attitude_sub.copy(&attitude);

			mavlink_aviant_trn_test_data_t msg{};

			msg.time_boot_ms = local_pos.timestamp / 1000;

			// Position
			msg.x = local_pos.x;
			msg.y = local_pos.y;
			msg.z = local_pos.z;

			// Velocity
			msg.vx = local_pos.vx;
			msg.vy = local_pos.vy;
			msg.vz = local_pos.vz;

			// Attitude quaternion
			msg.q1 = attitude.q[0];
			msg.q2 = attitude.q[1];
			msg.q3 = attitude.q[2];
			msg.q4 = attitude.q[3];

			mavlink_msg_aviant_trn_test_data_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // AVIANT_TRN_TEST_DATA_HPP
