/****************************************************************************
 *
 *   Copyright (c) 2013-2021 PX4 Development Team. All rights reserved.
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
 * @file VTOL_att_control_main.cpp
 * Implementation of an attitude controller for VTOL airframes. This module receives data
 * from both the fixed wing- and the multicopter attitude controllers and processes it.
 * It computes the correct actuator controls depending on which mode the vehicle is in (hover, forward-
 * flight or transition). It also publishes the resulting controls on the actuator controls topics.
 *
 * @author Roman Bapst 		<bapstr@ethz.ch>
 * @author Lorenz Meier 	<lm@inf.ethz.ch>
 * @author Thomas Gubler	<thomasgubler@gmail.com>
 * @author David Vorsin		<davidvorsin@gmail.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Andreas Antener 	<andreas@uaventure.com>
 *
 */
#include "vtol_att_control_main.h"
#include <px4_platform_common/events.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>

using namespace matrix;
using namespace time_literals;

VtolAttitudeControl::VtolAttitudeControl() :
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, "vtol_att_control: cycle"))
{
	_airspeed_filtered.reset(0.f);
	_vtol_vehicle_status.vtol_in_rw_mode = true;	/* start vtol in rotary wing mode*/

	_params.idle_pwm_mc = PWM_DEFAULT_MIN;
	_params.vtol_motor_id = 0;

	_params_handles.sys_ctrl_alloc = param_find("SYS_CTRL_ALLOC");
	_params.ctrl_alloc = 0;
	param_get(_params_handles.sys_ctrl_alloc, &_params.ctrl_alloc);

	if (_params.ctrl_alloc != 1) {
		// these are not used with dynamic control allocation
		_params_handles.idle_pwm_mc = param_find("VT_IDLE_PWM_MC");
		_params_handles.vtol_motor_id = param_find("VT_MOT_ID");
		_params_handles.vt_mc_on_fmu = param_find("VT_MC_ON_FMU");
		_params_handles.fw_motors_off = param_find("VT_FW_MOT_OFFID");
	}

	_params_handles.vtol_fw_permanent_stab = param_find("VT_FW_PERM_STAB");
	_params_handles.vtol_type = param_find("VT_TYPE");
	_params_handles.elevons_mc_lock = param_find("VT_ELEV_MC_LOCK");
	_params_handles.fw_min_alt = param_find("VT_FW_MIN_ALT");
	_params_handles.fw_alt_err = param_find("VT_FW_ALT_ERR");
	_params_handles.fw_qc_max_pitch = param_find("VT_FW_QC_P");
	_params_handles.fw_qc_max_roll = param_find("VT_FW_QC_R");
	_params_handles.front_trans_time_openloop = param_find("VT_F_TR_OL_TM");
	_params_handles.front_trans_time_min = param_find("VT_TRANS_MIN_TM");

	_params_handles.front_trans_duration = param_find("VT_F_TRANS_DUR");
	_params_handles.back_trans_duration = param_find("VT_B_TRANS_DUR");
	_params_handles.transition_airspeed = param_find("VT_ARSP_TRANS");
	_params_handles.front_trans_throttle = param_find("VT_F_TRANS_THR");
	_params_handles.back_trans_throttle = param_find("VT_B_TRANS_THR");
	_params_handles.airspeed_blend = param_find("VT_ARSP_BLEND");
	_params_handles.airspeed_mode = param_find("FW_ARSP_MODE");
	_params_handles.front_trans_timeout = param_find("VT_TRANS_TIMEOUT");
	_params_handles.mpc_xy_cruise = param_find("MPC_XY_CRUISE");
	_params_handles.diff_thrust = param_find("VT_FW_DIFTHR_EN");
	_params_handles.diff_thrust_scale = param_find("VT_FW_DIFTHR_SC");
	_params_handles.dec_to_pitch_ff = param_find("VT_B_DEC_FF");
	_params_handles.dec_to_pitch_i = param_find("VT_B_DEC_I");
	_params_handles.back_trans_dec_sp = param_find("VT_B_DEC_MSS");

	_params_handles.pitch_min_rad = param_find("VT_PTCH_MIN");
	_params_handles.forward_thrust_scale = param_find("VT_FWD_THRUST_SC");

	_params_handles.vt_forward_thrust_enable_mode = param_find("VT_FWD_THRUST_EN");
	_params_handles.vt_arsp_tau = param_find("VT_ARSP_TAU");
	_params_handles.mpc_land_alt1 = param_find("MPC_LAND_ALT1");
	_params_handles.mpc_land_alt2 = param_find("MPC_LAND_ALT2");

	_params_handles.land_pitch_min_rad = param_find("VT_LND_PTCH_MIN");

	/* fetch initial parameter values */
	parameters_update();

	if (static_cast<vtol_type>(_params.vtol_type) == vtol_type::TAILSITTER) {
		_vtol_type = new Tailsitter(this);

	} else if (static_cast<vtol_type>(_params.vtol_type) == vtol_type::TILTROTOR) {
		_vtol_type = new Tiltrotor(this);

	} else if (static_cast<vtol_type>(_params.vtol_type) == vtol_type::STANDARD) {
		_vtol_type = new Standard(this);

	} else {
		exit_and_cleanup();
	}

	_vtol_vehicle_status_pub.advertise();
	_vehicle_thrust_setpoint0_pub.advertise();
	_vehicle_torque_setpoint0_pub.advertise();
	_vehicle_thrust_setpoint1_pub.advertise();
	_vehicle_torque_setpoint1_pub.advertise();
}

VtolAttitudeControl::~VtolAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
VtolAttitudeControl::init()
{
	if (!_actuator_inputs_mc.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	if (!_actuator_inputs_fw.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void VtolAttitudeControl::action_request_poll()
{
	while (_action_request_sub.updated()) {
		action_request_s action_request;

		if (_action_request_sub.copy(&action_request)) {
			switch (action_request.action) {
			case action_request_s::ACTION_VTOL_TRANSITION_TO_MULTICOPTER:
				_transition_command = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
				_immediate_transition = false;
				break;

			case action_request_s::ACTION_VTOL_TRANSITION_TO_FIXEDWING:
				_transition_command = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;
				_immediate_transition = false;
				break;
			}
		}
	}
}

void VtolAttitudeControl::vehicle_cmd_poll()
{
	vehicle_command_s vehicle_command;

	while (_vehicle_cmd_sub.update(&vehicle_command)) {
		if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION) {
			vehicle_status_s vehicle_status{};
			_vehicle_status_sub.copy(&vehicle_status);

			uint8_t result = vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED;

			const int transition_command_param1 = int(vehicle_command.param1 + 0.5f);

			// deny transition from MC to FW in Takeoff, Land, RTL and Orbit
			if (transition_command_param1 == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW &&
			    (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF
			     || vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND
			     || vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL
			     ||  vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ORBIT)) {

				result = vehicle_command_ack_s::VEHICLE_RESULT_TEMPORARILY_REJECTED;

			} else {
				_transition_command = transition_command_param1;
				_immediate_transition = (PX4_ISFINITE(vehicle_command.param2)) ? int(vehicle_command.param2 + 0.5f) : false;
			}

			if (vehicle_command.from_external) {
				vehicle_command_ack_s command_ack{};
				command_ack.timestamp = hrt_absolute_time();
				command_ack.command = vehicle_command.command;
				command_ack.result = result;
				command_ack.target_system = vehicle_command.source_system;
				command_ack.target_component = vehicle_command.source_component;

				uORB::Publication<vehicle_command_ack_s> command_ack_pub{ORB_ID(vehicle_command_ack)};
				command_ack_pub.publish(command_ack);
			}

		} else if (_vtol_vehicle_status.vtol_transition_failsafe &&
			   (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_SET_MODE ||
			    vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_REPOSITION)) {
			/* Reset _transition_command if the vehicle failed to do transition and external command arrive.
			   This will result in the exiting failsafe mode. */
			_transition_command = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
		}
	}
}

void
VtolAttitudeControl::quadchute(QuadchuteReason reason)
{
	if (!_vtol_vehicle_status.vtol_transition_failsafe) {
		switch (reason) {
		case QuadchuteReason::TransitionTimeout:
			mavlink_log_critical(&_mavlink_log_pub, "Quadchute: transition timeout\t");
			events::send(events::ID("vtol_att_ctrl_quadchute_tout"), events::Log::Critical,
				     "Quadchute triggered, due to transition timeout");
			break;

		case QuadchuteReason::ExternalCommand:
			mavlink_log_critical(&_mavlink_log_pub, "Quadchute: external command\t");
			events::send(events::ID("vtol_att_ctrl_quadchute_ext_cmd"), events::Log::Critical,
				     "Quadchute triggered, due to external command");
			break;

		case QuadchuteReason::MinimumAltBreached:
			mavlink_log_critical(&_mavlink_log_pub, "Quadchute: minimum altitude breached\t");
			events::send(events::ID("vtol_att_ctrl_quadchute_min_alt"), events::Log::Critical,
				     "Quadchute triggered, due to minimum altitude breach");
			break;

		case QuadchuteReason::LossOfAlt:
			mavlink_log_critical(&_mavlink_log_pub, "Quadchute: loss of altitude\t");
			events::send(events::ID("vtol_att_ctrl_quadchute_alt_loss"), events::Log::Critical,
				     "Quadchute triggered, due to loss of altitude");
			break;

		case QuadchuteReason::LargeAltError:
			mavlink_log_critical(&_mavlink_log_pub, "Quadchute: large altitude error\t");
			events::send(events::ID("vtol_att_ctrl_quadchute_alt_err"), events::Log::Critical,
				     "Quadchute triggered, due to large altitude error");
			break;

		case QuadchuteReason::MaximumPitchExceeded:
			mavlink_log_critical(&_mavlink_log_pub, "Quadchute: maximum pitch exceeded\t");
			events::send(events::ID("vtol_att_ctrl_quadchute_max_pitch"), events::Log::Critical,
				     "Quadchute triggered, due to maximum pitch angle exceeded");
			break;

		case QuadchuteReason::MaximumRollExceeded:
			mavlink_log_critical(&_mavlink_log_pub, "Quadchute: maximum roll exceeded\t");
			events::send(events::ID("vtol_att_ctrl_quadchute_max_roll"), events::Log::Critical,
				     "Quadchute triggered, due to maximum roll angle exceeded");
			break;
		}

		_quadchute_requested = true;
	}
}

int
VtolAttitudeControl::parameters_update()
{
	float v;
	int32_t l;

	if (_params.ctrl_alloc != 1) {
		/* idle pwm for mc mode */
		param_get(_params_handles.idle_pwm_mc, &_params.idle_pwm_mc);
		param_get(_params_handles.vtol_motor_id, &_params.vtol_motor_id);
		param_get(_params_handles.vt_mc_on_fmu, &l);
		_params.vt_mc_on_fmu = l;

		/* vtol motor count */
		param_get(_params_handles.fw_motors_off, &_params.fw_motors_off);
	}

	/* vtol fw permanent stabilization */
	param_get(_params_handles.vtol_fw_permanent_stab, &l);
	_vtol_vehicle_status.fw_permanent_stab = (l == 1);

	param_get(_params_handles.vtol_type, &l);
	_params.vtol_type = l;

	/* vtol lock elevons in multicopter */
	param_get(_params_handles.elevons_mc_lock, &l);
	_params.elevons_mc_lock = (l == 1);

	/* minimum relative altitude for FW mode (QuadChute) */
	param_get(_params_handles.fw_min_alt, &v);
	_params.fw_min_alt = v;

	/* maximum negative altitude error for FW mode (Adaptive QuadChute) */
	param_get(_params_handles.fw_alt_err, &v);
	_params.fw_alt_err = v;

	/* maximum pitch angle (QuadChute) */
	param_get(_params_handles.fw_qc_max_pitch, &l);
	_params.fw_qc_max_pitch = l;

	/* maximum roll angle (QuadChute) */
	param_get(_params_handles.fw_qc_max_roll, &l);
	_params.fw_qc_max_roll = l;

	param_get(_params_handles.front_trans_time_openloop, &_params.front_trans_time_openloop);

	param_get(_params_handles.front_trans_time_min, &_params.front_trans_time_min);

	param_get(_params_handles.vt_arsp_tau, &_params.vt_arsp_tau);

	/*
	 * Open loop transition time needs to be larger than minimum transition time,
	 * anything else makes no sense and can potentially lead to numerical problems.
	 */
	if (_params.front_trans_time_openloop < _params.front_trans_time_min * 1.1f) {
		_params.front_trans_time_openloop = _params.front_trans_time_min * 1.1f;
		param_set_no_notification(_params_handles.front_trans_time_openloop, &_params.front_trans_time_openloop);
		mavlink_log_critical(&_mavlink_log_pub, "OL transition time set larger than min transition time\t");
		/* EVENT
		 * @description <param>VT_F_TR_OL_TM</param> set to {1:.1}.
		 */
		events::send<float>(events::ID("vtol_att_ctrl_ol_trans_too_large"), events::Log::Warning,
				    "Open loop transition time set larger than minimum transition time", _params.front_trans_time_openloop);
	}

	param_get(_params_handles.front_trans_duration, &_params.front_trans_duration);
	param_get(_params_handles.back_trans_duration, &_params.back_trans_duration);
	param_get(_params_handles.transition_airspeed, &_params.transition_airspeed);
	param_get(_params_handles.front_trans_throttle, &_params.front_trans_throttle);
	param_get(_params_handles.back_trans_throttle, &_params.back_trans_throttle);
	param_get(_params_handles.airspeed_blend, &_params.airspeed_blend);
	param_get(_params_handles.airspeed_mode, &l);
	_params.airspeed_disabled = l != 0;
	param_get(_params_handles.front_trans_timeout, &_params.front_trans_timeout);
	param_get(_params_handles.mpc_xy_cruise, &_params.mpc_xy_cruise);
	param_get(_params_handles.diff_thrust, &_params.diff_thrust);

	param_get(_params_handles.diff_thrust_scale, &v);
	_params.diff_thrust_scale = math::constrain(v, -1.0f, 1.0f);

	/* maximum down pitch allowed */
	param_get(_params_handles.pitch_min_rad, &v);
	_params.pitch_min_rad = math::radians(v);

	/* maximum down pitch allowed during landing*/
	param_get(_params_handles.land_pitch_min_rad, &v);
	_params.land_pitch_min_rad = math::radians(v);

	/* scale for fixed wing thrust used for forward acceleration in multirotor mode */
	param_get(_params_handles.forward_thrust_scale, &_params.forward_thrust_scale);

	// make sure parameters are feasible, require at least 1 m/s difference between transition and blend airspeed
	_params.airspeed_blend = math::min(_params.airspeed_blend, _params.transition_airspeed - 1.0f);

	param_get(_params_handles.back_trans_dec_sp, &v);
	// increase the target deceleration setpoint provided to the controller by 20%
	// to make overshooting the transition waypoint less likely in the presence of tracking errors
	_params.back_trans_dec_sp = 1.2f * v;

	param_get(_params_handles.dec_to_pitch_ff, &_params.dec_to_pitch_ff);
	param_get(_params_handles.dec_to_pitch_i, &_params.dec_to_pitch_i);

	param_get(_params_handles.vt_forward_thrust_enable_mode, &_params.vt_forward_thrust_enable_mode);
	param_get(_params_handles.mpc_land_alt1, &_params.mpc_land_alt1);
	param_get(_params_handles.mpc_land_alt2, &_params.mpc_land_alt2);

	// update the parameters of the instances of base VtolType
	if (_vtol_type != nullptr) {
		_vtol_type->parameters_update();
	}

	return OK;
}

void
VtolAttitudeControl::Run()
{
	if (should_exit()) {
		_actuator_inputs_fw.unregisterCallback();
		_actuator_inputs_mc.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

#if !defined(ENABLE_LOCKSTEP_SCHEDULER)

	// prevent excessive scheduling (> 500 Hz)
	if (now - _last_run_timestamp < 2_ms) {
		return;
	}

#endif // !ENABLE_LOCKSTEP_SCHEDULER
	_last_run_timestamp = now;

	if (!_initialized) {
		parameters_update();  // initialize parameter cache

		if (_vtol_type->init()) {
			_initialized = true;

		} else {
			exit_and_cleanup();
			return;
		}
	}

	perf_begin(_loop_perf);

	const bool updated_fw_in = _actuator_inputs_fw.update(&_actuators_fw_in);
	const bool updated_mc_in = _actuator_inputs_mc.update(&_actuators_mc_in);

	// run on actuator publications corresponding to VTOL mode
	bool should_run = false;

	switch (_vtol_type->get_mode()) {
	case mode::TRANSITION_TO_FW:
	case mode::TRANSITION_TO_MC:
		should_run = updated_fw_in || updated_mc_in;
		break;

	case mode::ROTARY_WING:
		should_run = updated_mc_in;
		break;

	case mode::FIXED_WING:
		should_run = updated_fw_in;
		break;
	}

	if (should_run) {
		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			parameters_update();
		}

		_v_control_mode_sub.update(&_v_control_mode);
		_v_att_sub.update(&_v_att);
		_local_pos_sub.update(&_local_pos);
		_local_pos_sp_sub.update(&_local_pos_sp);
		_pos_sp_triplet_sub.update(&_pos_sp_triplet);

		if (_airspeed_validated_sub.update(&_airspeed_validated)) {
			update_filtered_airspeed(_airspeed_validated);
		}

		_tecs_status_sub.update(&_tecs_status);
		_land_detected_sub.update(&_land_detected);
		action_request_poll();
		vehicle_cmd_poll();

		vehicle_air_data_s air_data;

		if (_vehicle_air_data_sub.update(&air_data)) {
			_air_density = air_data.rho;
		}

		// check if mc and fw sp were updated
		bool mc_att_sp_updated = _mc_virtual_att_sp_sub.update(&_mc_virtual_att_sp);
		bool fw_att_sp_updated = _fw_virtual_att_sp_sub.update(&_fw_virtual_att_sp);

		// Set vtol_transition_failsafe flag if quadchute was requested the previous iteration
		// We do this before _vtol_type->update_vtol_state() so that
		// failsafe flag and mr flag is updated at the same iteration
		if (_quadchute_requested) {
			_vtol_vehicle_status.vtol_transition_failsafe = true;
			_quadchute_requested = false;
		}

		// update the vtol state machine which decides which mode we are in
		_vtol_type->update_vtol_state();

		// check in which mode we are in and call mode specific functions
		switch (_vtol_type->get_mode()) {
		case mode::TRANSITION_TO_FW:
		case mode::TRANSITION_TO_MC:
			// vehicle is doing a transition
			_vtol_vehicle_status.vtol_in_trans_mode = true;
			_vtol_vehicle_status.vtol_in_rw_mode = true; // making mc attitude controller work during transition
			_vtol_vehicle_status.in_transition_to_fw = (_vtol_type->get_mode() == mode::TRANSITION_TO_FW);

			_fw_virtual_att_sp_sub.update(&_fw_virtual_att_sp);

			if (!_vtol_type->was_in_trans_mode() || mc_att_sp_updated || fw_att_sp_updated) {
				_vtol_type->update_transition_state();
				_v_att_sp_pub.publish(_v_att_sp);
			}

			break;

		case mode::ROTARY_WING:
			// vehicle is in rotary wing mode
			_vtol_vehicle_status.vtol_in_rw_mode = true;
			_vtol_vehicle_status.vtol_in_trans_mode = false;
			_vtol_vehicle_status.in_transition_to_fw = false;

			_vtol_type->update_mc_state();
			_v_att_sp_pub.publish(_v_att_sp);

			break;

		case mode::FIXED_WING:
			// vehicle is in fw mode
			_vtol_vehicle_status.vtol_in_rw_mode = false;
			_vtol_vehicle_status.vtol_in_trans_mode = false;
			_vtol_vehicle_status.in_transition_to_fw = false;

			if (fw_att_sp_updated) {
				_vtol_type->update_fw_state();
				_v_att_sp_pub.publish(_v_att_sp);
			}

			break;
		}

		_vtol_type->fill_actuator_outputs();
		_actuators_0_pub.publish(_actuators_out_0);
		_actuators_1_pub.publish(_actuators_out_1);

		_vehicle_torque_setpoint0_pub.publish(_torque_setpoint_0);
		_vehicle_torque_setpoint1_pub.publish(_torque_setpoint_1);
		_vehicle_thrust_setpoint0_pub.publish(_thrust_setpoint_0);
		_vehicle_thrust_setpoint1_pub.publish(_thrust_setpoint_1);

		// Advertise/Publish vtol vehicle status
		_vtol_vehicle_status.timestamp = hrt_absolute_time();
		_vtol_vehicle_status.airspeed_filtered = get_filtered_airspeed();
		_vtol_vehicle_status.mc_weights[0] = _vtol_type->_mc_roll_weight;
		_vtol_vehicle_status.mc_weights[1] = _vtol_type->_mc_pitch_weight;
		_vtol_vehicle_status.mc_weights[2] = _vtol_type->_mc_yaw_weight;
		_vtol_vehicle_status.mc_weights[3] = _vtol_type->_mc_throttle_weight;
		_vtol_vehicle_status_pub.publish(_vtol_vehicle_status);
	}

	perf_end(_loop_perf);
}

void VtolAttitudeControl::update_filtered_airspeed(const struct airspeed_validated_s &sample)
{
	if (!sample.airspeed_sensor_measurement_valid || !PX4_ISFINITE(sample.calibrated_airspeed_m_s)) {
		if (_airspeed_filtered_valid) {
			PX4_WARN("Received invalid calibrated airspeed");
		}

		_airspeed_filtered_valid = false;
		return;
	}

	if (sample.timestamp <= _airspeed_filtered_last_timestamp) {
		if (_airspeed_filtered_valid) {
			PX4_WARN("Airspeed timestamp not monotonic");
		}

		_airspeed_filtered_valid = false;
		return;
	}

	// We must update the filter parameters, since we don't know the sensor sample rate
	float dt_s = static_cast<float>(sample.timestamp - _airspeed_filtered_last_timestamp) / 1e6f;
	_airspeed_filtered.setParameters(dt_s, _params.vt_arsp_tau);
	_airspeed_filtered.update(sample.calibrated_airspeed_m_s);
	_airspeed_filtered_last_timestamp = sample.timestamp;
	_airspeed_filtered_valid = true;
}

float VtolAttitudeControl::get_filtered_airspeed()
{
	if (!_airspeed_filtered_valid) {
		return NAN;
	}

	return _airspeed_filtered.getState();
}

int
VtolAttitudeControl::task_spawn(int argc, char *argv[])
{
	VtolAttitudeControl *instance = new VtolAttitudeControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int
VtolAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
VtolAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_att_control is the fixed wing attitude controller.
)DESCR_STR");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_NAME("vtol_att_control", "controller");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int vtol_att_control_main(int argc, char *argv[])
{
	return VtolAttitudeControl::main(argc, argv);
}
