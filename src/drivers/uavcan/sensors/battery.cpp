/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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

#include "battery.hpp"

#include <lib/geo/geo.h>
#include <px4_defines.h>

const char *const UavcanBatteryBridge::NAME = "battery";

UavcanBatteryBridge::UavcanBatteryBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_battery", ORB_ID(battery_status)),
	ModuleParams(nullptr),
	_sub_battery(node),
	_sub_battery_aux(node),
	_pub_battery_info(node),
	_pub_battery_info_aux(node),
	_warning(battery_status_s::BATTERY_WARNING_NONE),
	_last_timestamp(0)
{
}

int UavcanBatteryBridge::init()
{

	int32_t uavcan_sub_bat = 0;
	param_get(param_find("UAVCAN_SUB_BAT"), &uavcan_sub_bat);

	if (uavcan_sub_bat) {
		int res = _sub_battery.start(BatteryInfoCbBinder(this, &UavcanBatteryBridge::battery_sub_cb));

		if (res < 0) {
			PX4_ERR("failed to start uavcan sub: %d", res);
			return res;
		}

		res = _sub_battery_aux.start(BatteryInfoAuxCbBinder(this, &UavcanBatteryBridge::battery_aux_sub_cb));

		if (res < 0) {
			PX4_ERR("failed to start uavcan sub: %d", res);
			return res;
		}
	}

	return 0;
}

void UavcanBatteryBridge::update()
{
	// TODO: Decide if we want to do this on UavcanSensorBridgeBase::update()
	// or have it as a SubscriptionCallbackWorkItem. The latter could allow us
	// to lock onto the exact same rate as the uORB battery_status topic.
	battery_status_s battery;


	if (_uavcan_pub_bat.get() && _sub_battery_uorb.update(&battery)) {
		// Keep track of max current since last publication
		// TODO: Average other values?
		_max_current =  fmax(_max_current, fabs(battery.current_a));

		// Rate limiting
		hrt_abstime interval_us = roundf(1e6f / _uavcan_bat_rate.get());

		if (hrt_absolute_time() < _last_uavcan_pub + interval_us) {
			return;
		}

		ardupilot::equipment::power::BatteryInfoAux battery_info_aux{};
		battery_info_aux.timestamp.usec = hrt_absolute_time();

		for (uint8_t i = 0; i < battery.cell_count; i++) {
			battery_info_aux.voltage_cell.push_back(battery.voltage_cell_v[i]);
		}

		battery_info_aux.cycle_count = battery.cycle_count;
		battery_info_aux.over_discharge_count = battery.over_discharge_count;
		battery_info_aux.max_current = _max_current;
		battery_info_aux.nominal_voltage = battery.nominal_voltage;
		battery_info_aux.is_powering_off = battery.is_powering_off;

		_pub_battery_info_aux.broadcast(battery_info_aux);


		uavcan::equipment::power::BatteryInfo battery_info{};
		battery_info.voltage = battery.voltage_v;
		battery_info.current = fabs(battery.current_a);
		battery_info.temperature = battery.temperature - CONSTANTS_ABSOLUTE_NULL_CELSIUS; // convert from C to K
		battery_info.full_charge_capacity_wh = battery.capacity;
		battery_info.remaining_capacity_wh = battery.remaining * battery.capacity;
		battery_info.state_of_charge_pct = battery.remaining * 100;
		battery_info.state_of_charge_pct_stdev = battery.max_error;
		battery_info.model_instance_id = 0; // TODO: what goes here?
		battery_info.model_name = "Aviant BMS";
		battery_info.battery_id = battery.serial_number;
		battery_info.hours_to_full_charge = 0; // TODO: Read BQ40Z80_TIME_TO_FULL
		battery_info.state_of_health_pct = battery.state_of_health;

		if (battery.current_a > 0.0f) {
			battery_info.status_flags = uavcan::equipment::power::BatteryInfo::STATUS_FLAG_CHARGING;

		} else {
			battery_info.status_flags = uavcan::equipment::power::BatteryInfo::STATUS_FLAG_IN_USE;
		}

		_pub_battery_info.broadcast(battery_info);

		// Reset max current
		_max_current = 0;
	}
}

void
UavcanBatteryBridge::battery_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::power::BatteryInfo> &msg)
{
	uint8_t instance = 0;

	for (instance = 0; instance < battery_status_s::MAX_INSTANCES; instance++) {
		if (battery_status[instance].id == msg.getSrcNodeID().get() || battery_status[instance].id == 0) {
			break;
		}
	}

	if (instance >= battery_status_s::MAX_INSTANCES) {
		return;
	}

	battery_status[instance].timestamp = hrt_absolute_time();
	battery_status[instance].voltage_v = msg.voltage;
	battery_status[instance].voltage_filtered_v = msg.voltage;
	battery_status[instance].current_a = msg.current;
	battery_status[instance].current_filtered_a = msg.current;
	battery_status[instance].current_average_a = msg.current;

	if (battery_aux_support[instance] == false) {
		sumDischarged(battery_status[instance].timestamp, battery_status[instance].current_a);
		battery_status[instance].discharged_mah = _discharged_mah;
	}

	battery_status[instance].remaining = msg.state_of_charge_pct / 100.0f; // between 0 and 1
	// battery_status[instance].scale = msg.; // Power scaling factor, >= 1, or -1 if unknown
	battery_status[instance].temperature = msg.temperature + CONSTANTS_ABSOLUTE_NULL_CELSIUS; // Kelvin to Celcius
	// battery_status[instance].cell_count = msg.;
	battery_status[instance].connected = true;
	battery_status[instance].source = msg.status_flags & uavcan::equipment::power::BatteryInfo::STATUS_FLAG_IN_USE;
	// battery_status[instance].priority = msg.;
	battery_status[instance].capacity = msg.full_charge_capacity_wh;
	battery_status[instance].full_charge_capacity_wh = msg.full_charge_capacity_wh;
	battery_status[instance].remaining_capacity_wh = msg.remaining_capacity_wh;
	// battery_status[instance].cycle_count = msg.;
	battery_status[instance].time_remaining_s = NAN;
	// battery_status[instance].average_time_to_empty = msg.;
	battery_status[instance].serial_number = msg.model_instance_id;
	battery_status[instance].id = msg.getSrcNodeID().get();

	if (battery_aux_support[instance] == false) {
		// Mavlink 2 needs individual cell voltages or cell[0] if cell voltages are not available.
		battery_status[instance].voltage_cell_v[0] = msg.voltage;

		// Set cell count to 1 so the the battery code in mavlink_messages.cpp copies the values correctly (hack?)
		battery_status[instance].cell_count = 1;
	}

	// battery_status[instance].max_cell_voltage_delta = msg.;

	// battery_status[instance].is_powering_off = msg.;

	determineWarning(battery_status[instance].remaining);
	battery_status[instance].warning = _warning;

	if (battery_aux_support[instance] == false) {
		publish(msg.getSrcNodeID().get(), &battery_status[instance]);
	}
}

void
UavcanBatteryBridge::battery_aux_sub_cb(const uavcan::ReceivedDataStructure<ardupilot::equipment::power::BatteryInfoAux>
					&msg)
{
	uint8_t instance = 0;

	for (instance = 0; instance < battery_status_s::MAX_INSTANCES; instance++) {
		if (battery_status[instance].id == msg.getSrcNodeID().get()) {
			break;
		}
	}

	if (instance >= battery_status_s::MAX_INSTANCES) {
		return;
	}

	battery_aux_support[instance] = true;

	battery_status[instance].discharged_mah = (battery_status[instance].full_charge_capacity_wh -
			battery_status[instance].remaining_capacity_wh) / msg.nominal_voltage *
			1000;
	battery_status[instance].cell_count = math::min((uint8_t)msg.voltage_cell.size(), (uint8_t)14);
	battery_status[instance].cycle_count = msg.cycle_count;
	battery_status[instance].over_discharge_count = msg.over_discharge_count;
	battery_status[instance].nominal_voltage = msg.nominal_voltage;
	battery_status[instance].time_remaining_s = math::isZero(battery_status[instance].current_a) ? 0 :
			(battery_status[instance].remaining_capacity_wh /
			 battery_status[instance].nominal_voltage / battery_status[instance].current_a * 3600);
	battery_status[instance].is_powering_off = msg.is_powering_off;

	for (uint8_t i = 0; i < battery_status[instance].cell_count; i++) {
		battery_status[instance].voltage_cell_v[i] = msg.voltage_cell[i];
	}

	publish(msg.getSrcNodeID().get(), &battery_status[instance]);
}

void
UavcanBatteryBridge::sumDischarged(hrt_abstime timestamp, float current_a)
{
	// Not a valid measurement
	if (current_a < 0.f) {
		// Because the measurement was invalid we need to stop integration
		// and re-initialize with the next valid measurement
		_last_timestamp = 0;
		return;
	}

	// Ignore first update because we don't know dt.
	if (_last_timestamp != 0) {
		const float dt = (timestamp - _last_timestamp) / 1e6;
		// mAh since last loop: (current[A] * 1000 = [mA]) * (dt[s] / 3600 = [h])
		_discharged_mah_loop = (current_a * 1e3f) * (dt / 3600.f);
		_discharged_mah += _discharged_mah_loop;
	}

	_last_timestamp = timestamp;
}

void
UavcanBatteryBridge::determineWarning(float remaining)
{
	// propagate warning state only if the state is higher, otherwise remain in current warning state
	if (remaining < _param_bat_emergen_thr.get() || (_warning == battery_status_s::BATTERY_WARNING_EMERGENCY)) {
		_warning = battery_status_s::BATTERY_WARNING_EMERGENCY;

	} else if (remaining < _param_bat_crit_thr.get() || (_warning == battery_status_s::BATTERY_WARNING_CRITICAL)) {
		_warning = battery_status_s::BATTERY_WARNING_CRITICAL;

	} else if (remaining < _param_bat_low_thr.get() || (_warning == battery_status_s::BATTERY_WARNING_LOW)) {
		_warning = battery_status_s::BATTERY_WARNING_LOW;
	}
}
