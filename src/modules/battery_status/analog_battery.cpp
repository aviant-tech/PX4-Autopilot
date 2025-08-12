/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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

#include <cmath>
#include <stdio.h>
#include <lib/battery/battery.h>
#include "analog_battery.h"
#include "drivers/drv_hrt.h"
#include "px4_platform_common/defines.h"

using namespace time_literals;

// Defaults to use if the parameters are not set
#if BOARD_NUMBER_BRICKS > 0
#if defined(BOARD_BATT_V_LIST) && defined(BOARD_BATT_I_LIST)
static constexpr int   DEFAULT_V_CHANNEL[BOARD_NUMBER_BRICKS] = BOARD_BATT_V_LIST;
static constexpr int   DEFAULT_I_CHANNEL[BOARD_NUMBER_BRICKS] = BOARD_BATT_I_LIST;
#else
#error  "BOARD_BATT_V_LIST and BOARD_BATT_I_LIST need to be defined"
#endif
#else
static constexpr int DEFAULT_V_CHANNEL[1] = {-1};
static constexpr int DEFAULT_I_CHANNEL[1] = {-1};
#endif

AnalogBattery::AnalogBattery(int index, ModuleParams *parent, const int sample_interval_us, const uint8_t source,
			     const uint8_t priority) :
	Battery(index, parent, sample_interval_us, source)
{
	Battery::setPriority(priority);
	char param_name[17];

	_analog_param_handles.v_offs_cur = param_find("BAT_V_OFFS_CURR");

	snprintf(param_name, sizeof(param_name), "BAT%d_V_DIV", index);
	_analog_param_handles.v_div = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_A_PER_V", index);
	_analog_param_handles.a_per_v = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_V_CHANNEL", index);
	_analog_param_handles.v_channel = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_I_CHANNEL", index);
	_analog_param_handles.i_channel = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_T_CHANNEL", index);
	_analog_param_handles.t_channel = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_T_R_PU", index);
	_analog_param_handles.t_r_pu = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_T_R_25C", index);
	_analog_param_handles.t_r_25c = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_T_BETA", index);
	_analog_param_handles.t_beta = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_T_V_SRC", index);
	_analog_param_handles.t_v_src = param_find(param_name);
}


void
AnalogBattery::updateBatteryStatusADC(hrt_abstime timestamp, float voltage_raw, float current_raw,
				      float temperature_raw)
{
	const float voltage_v = voltage_raw * _analog_params.v_div;
	const float current_a = (current_raw - _analog_params.v_offs_cur) * _analog_params.a_per_v;

	static constexpr float zero_c_in_k = 273.15f;
	static constexpr float epsilon = 1e-6f;

	float temperature_c = NAN;

	const float thermistor_pullup_v = _analog_params.t_v_src - temperature_raw;
	const float thermistor_ohm_inf = _analog_params.t_r_25c * expf(-_analog_params.t_beta / (zero_c_in_k + 25.f));

	if (thermistor_pullup_v > epsilon && thermistor_ohm_inf > epsilon) {
		const float thermistor_ohm = _analog_params.t_r_pu * temperature_raw / thermistor_pullup_v;
		const float thermistor_ohm_ratio = thermistor_ohm / thermistor_ohm_inf;

		const float log_thermistor_ohm_ratio = thermistor_ohm_ratio > epsilon ? logf(thermistor_ohm_ratio) : NAN;

		if (PX4_ISFINITE(log_thermistor_ohm_ratio) && log_thermistor_ohm_ratio > epsilon) {
			temperature_c = _analog_params.t_beta / log_thermistor_ohm_ratio - zero_c_in_k;
		}
	}


	const bool connected = voltage_v > BOARD_ADC_OPEN_CIRCUIT_V &&
			       (BOARD_ADC_OPEN_CIRCUIT_V <= BOARD_VALID_UV || is_valid());

	Battery::setConnected(connected);
	Battery::updateVoltage(voltage_v);
	Battery::updateTemperature(temperature_c);
	Battery::updateCurrent(current_a);
	Battery::updateAndPublishBatteryStatus(timestamp);
}

bool AnalogBattery::is_valid()
{
#ifdef BOARD_BRICK_VALID_LIST
	bool valid[BOARD_NUMBER_BRICKS] = BOARD_BRICK_VALID_LIST;
	return valid[_index - 1];
#else
	// TODO: Maybe return false instead?
	return true;
#endif
}

int AnalogBattery::get_voltage_channel()
{
	if (_analog_params.v_channel >= 0) {
		return _analog_params.v_channel;

	} else {
		return DEFAULT_V_CHANNEL[_index - 1];
	}
}

int AnalogBattery::get_current_channel()
{
	if (_analog_params.i_channel >= 0) {
		return _analog_params.i_channel;

	} else {
		return DEFAULT_I_CHANNEL[_index - 1];
	}
}

int AnalogBattery::get_temperature_channel()
{
	if (_analog_params.t_channel >= 0) {
		return _analog_params.t_channel;

	} else {
		return -1; // No temperature channel
	}
}

void
AnalogBattery::updateParams()
{
	param_get(_analog_param_handles.v_div, &_analog_params.v_div);
	param_get(_analog_param_handles.a_per_v, &_analog_params.a_per_v);
	param_get(_analog_param_handles.v_channel, &_analog_params.v_channel);
	param_get(_analog_param_handles.i_channel, &_analog_params.i_channel);
	param_get(_analog_param_handles.t_channel, &_analog_params.t_channel);
	param_get(_analog_param_handles.t_r_pu, &_analog_params.t_r_pu);
	param_get(_analog_param_handles.t_r_25c, &_analog_params.t_r_25c);
	param_get(_analog_param_handles.t_beta, &_analog_params.t_beta);
	param_get(_analog_param_handles.t_v_src, &_analog_params.t_v_src);
	param_get(_analog_param_handles.v_offs_cur, &_analog_params.v_offs_cur);

	Battery::updateParams();
}
