/***************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file missionrtl_base.h
 *
 * Helper class for MissionRTL modes using the mission
 *
 */

#pragma once

#include "mission_base.h"
#include "navigator/navigation.h"
#include <cstdlib>

class MissionRtlBase : public MissionBase
{
public:
	MissionRtlBase(Navigator *navigator, int32_t dataman_cache_size_signed):
		MissionBase(navigator, dataman_cache_size_signed) {};
	virtual ~MissionRtlBase() = default;

	virtual void on_activation(bool from_mission_mode) = 0;

	mission_s getMissionInfo() const { return _mission; };

	bool getMissionItem(uint32_t idx, mission_item_s *dst)
	{
		if (idx >= _mission.count) { return false; }

		const dm_item_t dm_item = static_cast<dm_item_t>(_mission.mission_dataman_id);
		return _dataman_cache.loadWait(
			       dm_item,
			       idx,
			       reinterpret_cast<uint8_t *>(dst),
			       sizeof(mission_item_s),
			       MAX_DATAMAN_LOAD_WAIT
		       );
	}

};
