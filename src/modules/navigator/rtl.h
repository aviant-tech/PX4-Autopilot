/***************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
/**
 * @file rtl.h
 *
 * Helper class for RTL
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#pragma once

#include <px4_platform_common/module_params.h>

#include "navigator_mode.h"
#include "mission_block.h"

#include <uORB/Subscription.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/rtl_time_estimate.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>
#include <matrix/math.hpp>
#include <lib/geo/geo.h>

class Navigator;

class RTL : public MissionBlock, public ModuleParams
{
public:
	RTL(Navigator *navigator);

	~RTL() = default;

	enum RTLType {
		RTL_TYPE_HOME_OR_RALLY = 0,
		RTL_TYPE_MISSION_LANDING,
		RTL_TYPE_MISSION_LANDING_REVERSED,
		RTL_TYPE_CLOSEST,
	};

	enum RTLDestinationType {
		RTL_DESTINATION_HOME = 0,
		RTL_DESTINATION_MISSION_LANDING,
		RTL_DESTINATION_SAFE_POINT,
	};

	enum RTLHeadingMode {
		RTL_NAVIGATION_HEADING = 0,
		RTL_DESTINATION_HEADING,
		RTL_CURRENT_HEADING,
	};

	void on_inactivation() override;
	void on_inactive() override;
	void on_activation() override;
	void on_active() override;

	void find_RTL_destination();

	void set_return_alt_min(bool min) { _rtl_alt_min = min; }

	int get_rtl_type() const { return _param_rtl_type.get(); }

	void setClimbAndReturnDone(bool done) { _climb_and_return_done = done; }

	bool getClimbAndReturnDone() { return _climb_and_return_done; }

	void get_rtl_xy_z_speed(float &xy, float &z);
	matrix::Vector2f get_wind();

	bool getShouldEngageMissionForLanding() const { return _should_engange_mission_for_landing; }

private:

	void set_rtl_item();

	void advance_rtl();

	float calculate_return_alt_from_cone_half_angle(float cone_half_angle_deg);
	void calc_and_pub_rtl_time_estimate();

	float getCruiseGroundSpeed();

	float getClimbRate();

	float getDescendRate();

	float getCruiseSpeed();

	float getHoverLandSpeed();

	enum RTLState {
		RTL_STATE_NONE = 0,
		RTL_STATE_CLIMB,
		RTL_STATE_RETURN,
		RTL_STATE_DESCEND,
		RTL_STATE_LOITER,
		RTL_STATE_TRANSITION_TO_MC,
		RTL_MOVE_TO_LAND_HOVER_VTOL,
		RTL_STATE_LAND,
		RTL_STATE_LANDED,
		RTL_STATE_HEAD_TO_CENTER,
		RTL_STATE_MC_DESCEND,
	} _rtl_state{RTL_STATE_NONE};

	struct RTLPosition {
		double lat;
		double lon;
		float alt;
		float yaw;
		uint8_t safe_point_index; ///< 0 = home position, 1 = mission landing, >1 = safe landing points (rally points)
		RTLDestinationType type{RTL_DESTINATION_HOME};

		void set(const home_position_s &home_position)
		{
			lat = home_position.lat;
			lon = home_position.lon;
			alt = home_position.alt;
			yaw = home_position.yaw;
			safe_point_index = 0;
			type = RTL_DESTINATION_HOME;
		}
	};

	RTLPosition _destination{}; ///< the RTL position to fly to (typically the home position or a safe point)

	hrt_abstime _destination_check_time{0};

	float _rtl_alt{0.0f};	// AMSL altitude at which the vehicle should return to the home position
	float _rtl_loiter_rad{50.0f};		// radius at which a fixed wing would loiter while descending

	bool _climb_and_return_done{false};	// this flag is set to true if RTL is active and we are past the climb state and return state
	bool _rtl_alt_min{false};
	bool _should_engange_mission_for_landing{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RTL_RETURN_ALT>)  _param_rtl_return_alt,
		(ParamFloat<px4::params::RTL_DESCEND_ALT>) _param_rtl_descend_alt,
		(ParamFloat<px4::params::RTL_DESCEND_MC>)  _param_rtl_descend_mc,
		(ParamFloat<px4::params::RTL_LAND_DELAY>)  _param_rtl_land_delay,
		(ParamFloat<px4::params::RTL_MIN_DIST>)    _param_rtl_min_dist,
		(ParamInt<px4::params::RTL_TYPE>)          _param_rtl_type,
		(ParamInt<px4::params::RTL_CONE_ANG>)      _param_rtl_cone_half_angle_deg,
		(ParamInt<px4::params::RTL_PLD_MD>)        _param_rtl_pld_md,
		(ParamFloat<px4::params::RTL_LOITER_RAD>)  _param_rtl_loiter_rad,
		(ParamInt<px4::params::RTL_HDG_MD>)        _param_rtl_hdg_md,
		(ParamFloat<px4::params::RTL_TIME_FACTOR>) _param_rtl_time_factor,
		(ParamInt<px4::params::RTL_TIME_MARGIN>)   _param_rtl_time_margin
	)

	param_t		_param_mpc_z_v_auto_up{PARAM_INVALID};
	param_t		_param_mpc_z_v_auto_dn{PARAM_INVALID};
	param_t		_param_mpc_land_speed{PARAM_INVALID};
	param_t		_param_fw_climb_rate{PARAM_INVALID};
	param_t		_param_fw_sink_rate{PARAM_INVALID};

	param_t 	_param_fw_airspeed_trim{PARAM_INVALID};
	param_t 	_param_mpc_xy_cruise{PARAM_INVALID};
	param_t 	_param_rover_cruise_speed{PARAM_INVALID};

	uORB::SubscriptionData<wind_s>		_wind_sub{ORB_ID(wind)};
	uORB::Publication<rtl_time_estimate_s> _rtl_time_estimate_pub{ORB_ID(rtl_time_estimate)};
};

float time_to_home(const matrix::Vector3f &to_home_vec,
		   const matrix::Vector2f &wind_velocity, float vehicle_speed_m_s,
		   float vehicle_descent_speed_m_s);
