/****************************************************************************
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
 * @file mission_rtl.cpp
 *
 * Aviant custom
 * Helper class to access Mission RTL as a flight mode
 *
 */

#include "mission_rtl.h"
#include "navigator.h"
#include "mission_block.h"
#include "navigator/mission_base.h"
#include "navigator/navigation.h"
#include "px4_platform_common/defines.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/events.h>
#include <geo/geo.h>

using namespace time_literals;
using namespace math;
using matrix::wrap_pi;

MissionRTL::MissionRTL(Navigator *navigator) :
	NavigatorMode(navigator)
{
	// Does not matter if this is Fast or FastReverse
	// One needs to exist before activation to have a mission
	// We can read the closest waypoint from
	_mission_handle = new MissionRtlMissionFast(_navigator);
}

void MissionRTL::on_inactivation()
{
}

void MissionRTL::on_inactive()
{
	_mission_handle->on_inactive();
}

void MissionRTL::on_activation()
{
	// If we're not already in mission mode,
	// run on_activation once to select the closest mission item
	// before we use it to decide whether to reverse or not.
	if (already_in_mission_mode) {
		PX4_INFO("MissionRTL was activated from mission mode, keeping the selected waypoint");

	} else {
		_mission_handle->on_activation(false);
	}

	const mission_s mission_info = _mission_handle->getMissionInfo();
	float distance_forwards_m{0};
	float distance_backwards_m{0};
	bool any_winch_commands_forwards{false};
	bool any_winch_commands_backwards{false};

	bool failed_to_read_any_mission_item{false};
	mission_item_s previous_item{0};
	mission_item_s current_item{0};

	// Note: Home location is not part of the mission here.
	// That is fine, just means that the distance from takeoff to
	// the first waypoint is not counted
	for (int item_idx = 0; item_idx < mission_info.count; item_idx++) {
		const bool load_success = _mission_handle->getMissionItem(item_idx, &current_item);

		if (!load_success) {
			failed_to_read_any_mission_item = true;
			break;
		}

		if (current_item.nav_cmd == NAV_CMD_DO_WINCH) {
			if (item_idx < mission_info.current_seq) {
				any_winch_commands_backwards = true;

			} else {
				any_winch_commands_forwards = true;
			}
		}

		if (!_mission_handle->item_contains_position(current_item)) { continue; }

		if (previous_item.nav_cmd == 0) {  // Uninitialized
			previous_item = current_item;
			continue;
		}

		float dist_xy_m, dist_z_m;
		get_distance_to_point_global_wgs84(
			previous_item.lat,
			previous_item.lon,
			previous_item.altitude,
			current_item.lat,
			current_item.lon,
			current_item.altitude,
			&dist_xy_m,
			&dist_z_m
		);
		const float distance_m = sqrtf(dist_xy_m * dist_xy_m + dist_z_m * dist_z_m);

		if (item_idx <= mission_info.current_seq) {
			// Note: this estimate is correct if we were not already in mission mode,
			// but if we were already in mission mode, the reverse distance should really
			// be counted up to the previous waypoint instead of the current one.
			// We choose to accept this slight inaccuracy
			// in order to reduce the complexity of the direction logic.
			distance_forwards_m += distance_m;

		} else {
			distance_backwards_m += distance_m;
		}

		previous_item = current_item;
	}

	bool should_reverse = false;

	// We choose to disregard the winch commands if they are both in front and behind us,
	// since it's not immediately obvious which direction is desired.
	if (failed_to_read_any_mission_item) {
		should_reverse = (mission_info.current_seq < mission_info.count / 2);
		PX4_INFO("MissionRTL: backtrack=%d because of waypoint index (%d of %d)",
			 static_cast<int>(should_reverse),
			 static_cast<int>(mission_info.current_seq),
			 static_cast<int>(mission_info.count)
			);

	} else {
		if (any_winch_commands_forwards && !any_winch_commands_backwards) {
			should_reverse = true;
			PX4_INFO("MissionRTl: backtracking because of winch command on the forwards path");

		} else if (any_winch_commands_backwards && !any_winch_commands_forwards) {
			should_reverse = false;
			PX4_INFO("MissionRTl: fast forward because of winch command on the reverse path");

		} else if (distance_forwards_m < distance_backwards_m) {
			should_reverse = true;
			PX4_INFO("MissionRTL: backtracking because backwards=%.3f m < forwards=%.3f m",
				 static_cast<double>(distance_forwards_m),
				 static_cast<double>(distance_backwards_m));

		} else if (distance_forwards_m > distance_backwards_m) {
			should_reverse = false;
			PX4_INFO("MissionRTL: fast forward because backwards=%.3f m > forwards=%.3f m",
				 static_cast<double>(distance_forwards_m),
				 static_cast<double>(distance_backwards_m));

		} else {
			PX4_INFO("MissionRTL: defaulting to fast forward");
			should_reverse = false;
		}
	}

	delete _mission_handle;

	if (should_reverse) {
		_mission_handle = new MissionRtlMissionFastReverse(_navigator);

	} else {
		_mission_handle = new MissionRtlMissionFast(_navigator);
	}

	_mission_handle->on_inactive();  // Tick once to propagate the mission

	_mission_handle->on_activation(already_in_mission_mode);
}

void MissionRTL::on_active()
{
	_mission_handle->on_active();
}

