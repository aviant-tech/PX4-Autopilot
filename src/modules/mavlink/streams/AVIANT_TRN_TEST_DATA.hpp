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
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>

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
		return (_trn_instance >= 0) ? MAVLINK_MSG_ID_AVIANT_TRN_TEST_DATA_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	static constexpr uint8_t MAX_INSTANCES = 9;

	// EKF2_TRN_PUB values
	static constexpr int32_t TRN_PUB_NONE = 0;
	static constexpr int32_t TRN_PUB_SELECTED = 1;
	static constexpr int32_t TRN_PUB_FIRST_BACKUP = 2;

	explicit MavlinkStreamAviantTrnTestData(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	int8_t _trn_instance{-1};
	int32_t _trn_pub_mode{-1}; // -1 = not yet read

	uORB::Subscription _local_pos_sub{ORB_ID(estimator_local_position)};
	uORB::Subscription _attitude_sub{ORB_ID(estimator_attitude)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _selector_status_sub{ORB_ID(estimator_selector_status)};

	void subscribe_to_instance(uint8_t inst)
	{
		_trn_instance = inst;
		_local_pos_sub = uORB::Subscription{ORB_ID(estimator_local_position), inst};
		_attitude_sub = uORB::Subscription{ORB_ID(estimator_attitude), inst};
	}

	void discover_trn_instance()
	{
		if (_trn_pub_mode < 0) {
			_trn_pub_mode = TRN_PUB_NONE;
			param_get(param_find("EKF2_TRN_PUB"), &_trn_pub_mode);
		}

		if (_trn_pub_mode == TRN_PUB_NONE) {
			return;
		}

		if (_trn_pub_mode == TRN_PUB_SELECTED) {
			estimator_selector_status_s selector_status;

			if (_selector_status_sub.copy(&selector_status)) {
				subscribe_to_instance(selector_status.primary_instance);
			}

		} else if (_trn_pub_mode == TRN_PUB_FIRST_BACKUP) {
			for (uint8_t i = 0; i < MAX_INSTANCES; i++) {
				uORB::Subscription status_sub{ORB_ID(estimator_status), i};
				estimator_status_s status;

				if (status_sub.copy(&status) && status.pos_est_group == estimator_status_s::POS_EST_GROUP_BACKUP) {
					subscribe_to_instance(i);
					PX4_INFO("AVIANT_TRN_TEST_DATA: using backup EKF instance %d", i);
					return;
				}
			}
		}
	}

	bool send() override
	{
		if (_trn_instance < 0) {
			discover_trn_instance();

			if (_trn_instance < 0) {
				return false;
			}
		}

		// For "Selected" mode, track primary instance changes
		if (_trn_pub_mode == TRN_PUB_SELECTED) {
			estimator_selector_status_s selector_status;

			if (_selector_status_sub.update(&selector_status)) {
				if (selector_status.primary_instance != static_cast<uint8_t>(_trn_instance)) {
					subscribe_to_instance(selector_status.primary_instance);
				}
			}
		}

		vehicle_local_position_s local_pos;
		vehicle_attitude_s attitude;
		vehicle_angular_velocity_s angular_velocity;

		if (_local_pos_sub.update(&local_pos)) {
			_attitude_sub.copy(&attitude);
			_angular_velocity_sub.copy(&angular_velocity);

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

			// Angular velocity
			msg.angular_velocity_x = angular_velocity.xyz[0];
			msg.angular_velocity_y = angular_velocity.xyz[1];
			msg.angular_velocity_z = angular_velocity.xyz[2];

			mavlink_msg_aviant_trn_test_data_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // AVIANT_TRN_TEST_DATA_HPP
