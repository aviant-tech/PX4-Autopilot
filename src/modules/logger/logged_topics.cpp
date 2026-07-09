/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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

#include "logged_topics.h"
#include "messages.h"

#include <parameters/param.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/topics/uORBTopics.hpp>

#include <string.h>

using namespace px4::logger;

/*
 * `python Tools/aviant/logger_budget.py`
 * Subscriptions: 234 non-optional, 63 optional, 297 total (max 255)
 * Bandwidth: 57.625 kbps (non-optional), 151.186 kbps (all)
 */
void LoggedTopics::add_default_topics()
{
	add_topic("adc_report", VERY_FAST); // For voltage spikes
	add_topic("aviant_motors", MODERATE);
	add_topic("aviant_navigation", MODERATE);
	add_topic("aviant_temperature_fc", SLOW);
	add_topic("aviant_navigation", MODERATE);
	add_topic("action_request", AUTO);  // 0 Hz or on change
	add_topic("actuator_armed", AUTO);  // 2 Hz or on change
	add_topic("actuator_controls_status_0", MODERATE);
	add_topic("airspeed", SLOW);
	add_topic("airspeed_validated", FAST);
	add_optional_topic("autotune_attitude_control_status", FAST);
	add_optional_topic("camera_capture", AUTO);  // 0 Hz or on change
	add_optional_topic("camera_trigger", AUTO);  // 0 Hz or on change
	add_topic("cellular_status", MODERATE);
	// add_topic("commander_state", AUTO);  // removed in a previous commit
	add_topic("config_overrides", AUTO);  // 2 Hz or on change
	add_topic("cpuload", AUTO);  // 2 Hz
	add_optional_topic("differential_drive_control_output", FAST);
	add_optional_topic("differential_drive_setpoint", FAST);
	add_optional_topic("external_ins_attitude", AUTO);  // 200 Hz
	add_optional_topic("external_ins_global_position", AUTO);  // 200 Hz
	add_optional_topic("external_ins_local_position", AUTO);  // 200 Hz
	add_optional_topic("esc_status", MODERATE);
	add_topic("failure_detector_status", FAST);
	add_topic("failsafe_flags", AUTO);  // 2 Hz or on change
	add_optional_topic("follow_target", MODERATE);
	add_optional_topic("follow_target_estimator", MODERATE);
	add_optional_topic("follow_target_status", MODERATE);
	add_optional_topic("flaps_setpoint", SLOW);
	add_topic("flight_phase_estimation", MODERATE);
	add_optional_topic("gimbal_manager_set_attitude", MODERATE);
	add_optional_topic("generator_status", AUTO);  // ~1 Hz
	add_optional_topic("gps_dump", AUTO);  // ~200 Hz
	add_optional_topic("gimbal_controls", MODERATE);
	add_optional_topic("gripper", AUTO);  // 0 Hz or on change
	add_optional_topic("heater_status", AUTO);  // ~1 Hz, MAVLink trigger
	add_topic("home_position", AUTO);  // 0 Hz or on change
	add_topic("hover_thrust_estimate", SLOW);
	add_topic("input_rc", MODERATE);
	add_optional_topic("internal_combustion_engine_status", VERY_FAST);
	add_optional_topic("iridiumsbd_status", SLOW);
	add_optional_topic("irlock_report", SLOW);
	add_optional_topic("landing_gear", MODERATE);
	add_optional_topic("landing_gear_wheel", FAST);
	add_optional_topic("landing_target_pose", SLOW);
	add_optional_topic("launch_detection_status", MODERATE);
	add_optional_topic("magnetometer_bias_estimate", MODERATE);
	add_topic("manual_control_setpoint", MODERATE);
	add_topic("manual_control_switches", AUTO);  // 1 Hz or on change
	add_topic("mission_result", AUTO);  // 0 Hz or on change
	add_topic("navigator_mission_item", AUTO);  // 0 Hz or on change
	add_topic("npfg_status", FAST);
	add_optional_topic("offboard_control_mode", FAST);
	add_topic("onboard_computer_status", SLOW);
	add_topic("parameter_update", AUTO);  // 0 Hz or on change
	add_topic("position_controller_status", MODERATE);
	//add_topic("position_controller_landing_status", FAST);  // We don't do FW landings
	add_topic("goto_setpoint", SLOW);
	add_topic("position_setpoint_triplet", MODERATE);
	add_topic("px4io_status", AUTO);  // 1 Hz
	add_topic("radio_status", AUTO);  // ~1 Hz, MAVLink trigger
	add_topic("rtl_time_estimate", SLOW);
	add_topic("rtl_status", SLOW);
	add_optional_topic("sensor_airflow", FAST);
	add_topic("sensor_combined", FAST);
	add_optional_topic("sensor_correction", AUTO);  // 0 Hz or on change
	add_topic("sensor_gyro_fft", MODERATE);
	add_topic("sensor_selection", AUTO);  // 0 Hz or on change
	add_topic("sensors_status_imu", MODERATE);
	add_optional_topic("spoilers_setpoint", SLOW);
	add_topic("system_power", MODERATE);
	add_topic("task_stack_info", AUTO);  // 2 Hz
	add_topic("takeoff_status", SLOW);
	add_topic("tecs_status", FAST);
	add_optional_topic("tiltrotor_extra_controls", FAST);
	add_topic("trajectory_setpoint", MODERATE);
	add_topic("transponder_report", AUTO);  // ~5 Hz or on change
	add_topic("vehicle_acceleration", VERY_FAST);
	add_topic("vehicle_air_data", MODERATE);
	add_topic("vehicle_angular_velocity", VERY_FAST);
	add_topic("vehicle_attitude", FAST);
	add_topic("vehicle_attitude_setpoint", FAST);
	add_topic("vehicle_command", AUTO);  // 0 Hz or on change
	add_topic("vehicle_command_ack", AUTO);  // 0 Hz or on change
	add_topic("vehicle_constraints", SLOW);
	add_topic("vehicle_control_mode", AUTO);  // 2 Hz or on change
	add_topic("vehicle_global_position", MODERATE);
	add_topic("vehicle_gps_position", MODERATE);
	add_topic("vehicle_gnss_heading", MODERATE);
	add_topic("vehicle_land_detected", AUTO);  // 1 Hz or on change
	add_topic("vehicle_local_position", MODERATE);
	add_topic("vehicle_local_position_setpoint", MODERATE);
	add_topic("vehicle_magnetometer", MODERATE);
	add_topic("vehicle_rates_setpoint", VERY_FAST);
	// add_topic("vehicle_roi", SLOW);  // We don't use "Region of interest" stuff
	add_topic("vehicle_status", AUTO);  // 2 Hz or on change
	add_topic("vtol_vehicle_status", MODERATE);
	add_topic("wind", SLOW);

	// multi topics
	add_topic_multi("actuator_outputs", MODERATE, 5);  // Low rate on all outputs
	add_topic("actuator_outputs", VERY_FAST, 0);  // Higher rate for PWM MAIN
	add_topic("actuator_outputs", VERY_FAST, 1);  // Higher rate for PWM AUX
	add_topic_multi("airspeed_wind", SLOW, 4);
	add_topic_multi("control_allocator_status", VERY_FAST, 2);
	add_topic_multi("rate_ctrl_status", MODERATE, 2);
	add_optional_topic_multi("sensor_hygrometer", MODERATE, 4);
	add_optional_topic_multi("rpm", MODERATE);
	add_topic_multi("timesync_status", SLOW, 3);
	add_topic_multi("telemetry_status", SLOW, 4);

	// EKF multi topics (currently max 9 estimators)
#if CONSTRAINED_MEMORY
	static constexpr uint8_t MAX_ESTIMATOR_INSTANCES = 1;
#else
	static constexpr uint8_t MAX_ESTIMATOR_INSTANCES = 6; // artificially limited until PlotJuggler fixed
	add_topic("estimator_selector_status", AUTO);  // 1 Hz or on change
	add_topic_multi("estimator_attitude", MODERATE, MAX_ESTIMATOR_INSTANCES);
	add_topic_multi("estimator_global_position", SLOW, MAX_ESTIMATOR_INSTANCES);
	add_topic_multi("estimator_local_position", MODERATE, MAX_ESTIMATOR_INSTANCES);
	add_topic_multi("estimator_wind", SLOW, MAX_ESTIMATOR_INSTANCES);
#endif

	// always add the first instance
	add_topic("estimator_baro_bias", MODERATE);
	add_topic("estimator_gnss_hgt_bias", MODERATE);
	add_topic("estimator_rng_hgt_bias", MODERATE);
	add_topic("estimator_ev_pos_bias", MODERATE);
	add_topic("estimator_event_flags", AUTO);	 // 1 Hz or on change
	add_topic("estimator_gps_status", SLOW);
	add_topic("estimator_innovation_test_ratios", MODERATE);
	add_topic("estimator_innovation_variances", MODERATE);
	add_topic("estimator_innovations", MODERATE);
	add_topic("estimator_optical_flow_vel", MODERATE);
	add_topic("estimator_sensor_bias", AUTO);  // 1 Hz or on change
	add_topic("estimator_states", SLOW);
	add_topic("estimator_status", MODERATE);
	add_topic("estimator_status_flags", AUTO);  // ~1 Hz or on change
	add_topic("yaw_estimator_status", SLOW);

	add_topic_multi("estimator_baro_bias", MODERATE, MAX_ESTIMATOR_INSTANCES);
	add_topic_multi("estimator_gnss_hgt_bias", MODERATE, MAX_ESTIMATOR_INSTANCES);
	add_topic_multi("estimator_rng_hgt_bias", MODERATE, MAX_ESTIMATOR_INSTANCES);
	add_topic_multi("estimator_ev_pos_bias", MODERATE, MAX_ESTIMATOR_INSTANCES);
	add_topic_multi("estimator_event_flags", AUTO, MAX_ESTIMATOR_INSTANCES);  // 1 Hz or on change
	add_topic_multi("estimator_gps_status", SLOW, MAX_ESTIMATOR_INSTANCES);
	add_topic_multi("estimator_innovation_test_ratios", MODERATE, MAX_ESTIMATOR_INSTANCES);
	add_topic_multi("estimator_innovation_variances", MODERATE, MAX_ESTIMATOR_INSTANCES);
	add_topic_multi("estimator_innovations", MODERATE, MAX_ESTIMATOR_INSTANCES);
	add_topic_multi("estimator_optical_flow_vel", MODERATE, MAX_ESTIMATOR_INSTANCES);
	add_topic_multi("estimator_sensor_bias", AUTO, MAX_ESTIMATOR_INSTANCES);  // 1 Hz or on change
	add_topic_multi("estimator_states", SLOW, MAX_ESTIMATOR_INSTANCES);
	add_topic_multi("estimator_status", MODERATE, MAX_ESTIMATOR_INSTANCES);
	add_topic_multi("estimator_status_flags", AUTO, MAX_ESTIMATOR_INSTANCES);  // ~1 Hz or on change
	add_topic_multi("yaw_estimator_status", SLOW, MAX_ESTIMATOR_INSTANCES);

	// add_optional_topic_multi("estimator_aid_src_airspeed", FAST, MAX_ESTIMATOR_INSTANCES);
	// add_optional_topic_multi("estimator_aid_src_baro_hgt", FAST, MAX_ESTIMATOR_INSTANCES);
	// add_optional_topic_multi("estimator_aid_src_ev_pos", FAST, MAX_ESTIMATOR_INSTANCES);
	// add_optional_topic_multi("estimator_aid_src_ev_vel", FAST, MAX_ESTIMATOR_INSTANCES);
	// add_optional_topic_multi("estimator_aid_src_ev_yaw", FAST, MAX_ESTIMATOR_INSTANCES);
	// add_optional_topic_multi("estimator_aid_src_gravity", FAST, MAX_ESTIMATOR_INSTANCES);
	// add_optional_topic_multi("estimator_aid_src_rng_hgt", FAST, MAX_ESTIMATOR_INSTANCES);
	// add_optional_topic_multi("estimator_aid_src_fake_hgt", FAST, MAX_ESTIMATOR_INSTANCES);
	// add_optional_topic_multi("estimator_aid_src_fake_pos", FAST, MAX_ESTIMATOR_INSTANCES);
	// add_optional_topic_multi("estimator_aid_src_gnss_yaw", FAST, MAX_ESTIMATOR_INSTANCES);
	// add_optional_topic_multi("estimator_aid_src_gnss_vel", FAST, MAX_ESTIMATOR_INSTANCES);
	// add_optional_topic_multi("estimator_aid_src_gnss_pos", FAST, MAX_ESTIMATOR_INSTANCES);
	// add_optional_topic_multi("estimator_aid_src_mag", FAST, MAX_ESTIMATOR_INSTANCES);
	// add_optional_topic_multi("estimator_aid_src_optical_flow", FAST, MAX_ESTIMATOR_INSTANCES);
	// add_optional_topic_multi("estimator_aid_src_terrain_optical_flow", FAST, MAX_ESTIMATOR_INSTANCES);
	// add_optional_topic_multi("estimator_aid_src_ev_yaw", FAST, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_aid_src_aux_global_position", FAST, MAX_ESTIMATOR_INSTANCES);

	// log all raw sensors at minimal rate (at least 1 Hz)
	add_topic("battery_status", MODERATE, 0);  // Use adc_report for high-rate
	add_optional_topic("battery_status", SLOW, 1);
	add_topic("differential_pressure", MODERATE, 0);
	add_optional_topic("differential_pressure", MODERATE, 1);
	add_topic("distance_sensor", MODERATE, 0);
	add_optional_topic("distance_sensor", MODERATE, 1);
	add_topic_multi("sensor_accel", SLOW, 4);
	add_topic_multi("sensor_baro", SLOW, 4);
	add_topic_multi("sensor_gps", SLOW, 2);
	add_topic_multi("sensor_gnss_relative", SLOW, 1);
	add_topic_multi("sensor_gyro", SLOW, 4);
	add_topic_multi("sensor_mag", SLOW, 4);
	add_optional_topic_multi("sensor_optical_flow", SLOW, 2);

	add_topic_multi("vehicle_imu", MODERATE, 4);
	add_topic("vehicle_imu_status", FAST, 2);  // IMU 2 for vibration metric
	add_optional_topic_multi("vehicle_imu_status", SLOW, 4);
	add_topic_multi("vehicle_magnetometer", MODERATE, 4);
	add_optional_topic("vehicle_optical_flow", MODERATE);
	add_optional_topic("aux_global_position", MODERATE);
	//add_optional_topic("vehicle_optical_flow_vel", FAST);
	add_optional_topic("pps_capture", AUTO);  // ~1 Hz, pulse from GPS

	// additional control allocation logging
	add_topic("actuator_motors", FAST);
	add_topic("actuator_servos", FAST);
	add_topic("vehicle_thrust_setpoint", VERY_FAST, 0);  // Motor thrust
	// add_topic("vehicle_thrust_setpoint", VERY_FAST, 1);  // Servo thrust, N/A
	add_topic("vehicle_torque_setpoint", VERY_FAST, 0);  // Motor torque
	add_topic("vehicle_torque_setpoint", VERY_FAST, 1);  // Servo torque

	// SYS_HITL: default ground truth logging for simulation
	int32_t sys_hitl = 0;
	param_get(param_find("SYS_HITL"), &sys_hitl);

	if (sys_hitl >= 1) {
		add_topic("vehicle_angular_velocity_groundtruth", VERY_FAST);
		add_topic("vehicle_attitude_groundtruth", VERY_FAST);
		add_topic("vehicle_global_position_groundtruth", FAST);
		add_topic("vehicle_local_position_groundtruth", VERY_FAST);
	}

#ifdef CONFIG_ARCH_BOARD_PX4_SITL
	add_topic("fw_virtual_attitude_setpoint", AUTO);
	add_topic("mc_virtual_attitude_setpoint", AUTO);
	add_optional_topic("vehicle_torque_setpoint_virtual_mc", AUTO);
	add_optional_topic("vehicle_torque_setpoint_virtual_fw", AUTO);
	add_optional_topic("vehicle_thrust_setpoint_virtual_mc", AUTO);
	add_optional_topic("vehicle_thrust_setpoint_virtual_fw", AUTO);
	add_topic("time_offset", AUTO);
	add_topic("vehicle_angular_velocity", VERY_FAST);
	add_topic("vehicle_angular_velocity_groundtruth", VERY_FAST);
	add_topic("vehicle_attitude_groundtruth", VERY_FAST);
	add_topic("vehicle_global_position_groundtruth", FAST);
	add_topic("vehicle_local_position_groundtruth", VERY_FAST);

	// EKF replay
	add_topic("estimator_baro_bias", AUTO);
	add_topic("estimator_gnss_hgt_bias", AUTO);
	add_topic("estimator_rng_hgt_bias", AUTO);
	add_topic("estimator_ev_pos_bias", AUTO);
	add_topic("estimator_event_flags", AUTO);
	add_topic("estimator_gps_status", AUTO);
	add_topic("estimator_innovation_test_ratios", AUTO);
	add_topic("estimator_innovation_variances", AUTO);
	add_topic("estimator_innovations", AUTO);
	add_topic("estimator_optical_flow_vel", AUTO);
	add_topic("estimator_sensor_bias", AUTO);
	add_topic("estimator_states", AUTO);
	add_topic("estimator_status", AUTO);
	add_topic("estimator_status_flags", AUTO);
	add_topic("vehicle_attitude", AUTO);
	add_topic("vehicle_global_position", AUTO);
	add_topic("vehicle_local_position", AUTO);
	add_topic("wind", AUTO);
	add_topic("yaw_estimator_status", AUTO);

	add_optional_topic_multi("estimator_aid_src_airspeed", AUTO, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_aid_src_baro_hgt", AUTO, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_aid_src_rng_hgt", AUTO, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_aid_src_fake_hgt", AUTO, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_aid_src_fake_pos", AUTO, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_aid_src_ev_hgt", AUTO, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_aid_src_ev_pos", AUTO, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_aid_src_ev_vel", AUTO, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_aid_src_ev_yaw", AUTO, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_aid_src_gnss_hgt", AUTO, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_aid_src_gnss_pos", AUTO, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_aid_src_gnss_vel", AUTO, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_aid_src_gnss_yaw", AUTO, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_aid_src_gravity", AUTO, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_aid_src_mag", AUTO, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_aid_src_optical_flow", AUTO, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_aid_src_terrain_optical_flow", AUTO, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_aid_src_terrain_range_finder", AUTO, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_aid_src_sideslip", AUTO, MAX_ESTIMATOR_INSTANCES);

#endif /* CONFIG_ARCH_BOARD_PX4_SITL */

#ifdef CONFIG_BOARD_UAVCAN_INTERFACES
	add_topic_multi("can_interface_status", FAST, CONFIG_BOARD_UAVCAN_INTERFACES);
#endif
}

/*
 * `python Tools/aviant/logger_budget.py --high-rate`
 * Subscriptions: 234 non-optional, 64 optional, 298 total (max 255)
 * Bandwidth: 149.472 kbps (non-optional), 254.914 kbps (all)
 */
void LoggedTopics::add_high_rate_topics()
{
	// maximum rate to analyze fast maneuvers (e.g. for racing)
	add_topic("manual_control_setpoint", AUTO);  // ~50 Hz, RC input rate
	add_topic_multi("rate_ctrl_status", VERY_FAST, 2);
	add_topic("sensor_combined", AUTO);  // 200 Hz
	add_topic("vehicle_angular_velocity", AUTO);  // 400 Hz, IMU_GYRO_RATEMAX
	add_topic("vehicle_attitude", AUTO);  // 200 Hz
	add_topic("vehicle_attitude_setpoint", AUTO);  // 200 Hz
	add_topic("vehicle_rates_setpoint", AUTO);  // 200 Hz
	add_topic("vehicle_local_position", VERY_FAST);  // 200 Hz
	add_topic("vehicle_local_position_setpoint", VERY_FAST);  // 200 Hz

	add_optional_topic("esc_status", VERY_FAST);  // 200 Hz, ESC status rate
	//add_topic("actuator_motors", AUTO);  // 400 Hz, IMU_GYRO_RATEMAX
	add_optional_topic("actuator_outputs_debug", AUTO);  // ~50 Hz, ESC status rate
	//add_topic("actuator_servos", AUTO);  // 400 Hz, IMU_GYRO_RATEMAX
	add_topic("vehicle_thrust_setpoint", AUTO, 0);  // 400 Hz, motor thrust, IMU_GYRO_RATEMAX
	// add_topic("vehicle_thrust_setpoint", 0, 1);  // 400 Hz, servo thrust, N/A, IMU_GYRO_RATEMAX
	add_topic("vehicle_torque_setpoint", AUTO, 0);  // 400 Hz, motor torque, IMU_GYRO_RATEMAX
	add_topic("vehicle_torque_setpoint", AUTO, 1);  // 400 Hz, servo torque, IMU_GYRO_RATEMAX
}

void LoggedTopics::add_debug_topics()
{
	add_topic("debug_array");
	add_topic("debug_key_value");
	add_topic("debug_value");
	add_topic("debug_vect");
	add_topic_multi("satellite_info", 1000, 2);
	add_topic("mag_worker_data");
	add_topic("sensor_preflight_mag", 500);
	add_topic("actuator_test", 500);
}

/*
 * `python Tools/aviant/logger_budget.py --ekf-replay`
 * Subscriptions: 239 non-optional, 60 optional, 299 total (max 255)
 * Bandwidth: 76.571 kbps (non-optional), 169.583 kbps (all)
 */
void LoggedTopics::add_estimator_replay_topics()
{
	// for estimator replay (need to be at full rate)
	add_topic("ekf2_timestamps", AUTO);  // 200 Hz, IMU rate

	// current EKF2 subscriptions
	add_topic("airspeed", AUTO);  // 20 Hz, sensor rate
	add_topic("vehicle_optical_flow", AUTO);  // ~0 Hz, we dont have a sensor for this
	add_topic("sensor_combined", AUTO);	 // 200 Hz, IMU rate
	add_topic("sensor_selection", AUTO);  // 0 Hz or on change
	add_topic("vehicle_air_data", AUTO);  // 20 Hz, sensor rate
	add_topic("vehicle_gps_position", AUTO);  // 10 Hz, GPS rate
	add_topic("vehicle_gnss_heading", AUTO);  // 10 Hz, GPS rate
	add_topic("vehicle_land_detected", AUTO);  // 1 Hz or on change
	add_topic("vehicle_magnetometer", AUTO);  // 20 Hz, mag rate
	add_topic("vehicle_status", AUTO);  // 2 Hz or on change
	add_topic("vehicle_visual_odometry", AUTO);  // ~0 Hz, we dont have a sensor for this
	add_topic("aux_global_position", AUTO);  // ~0 Hz, we dont have a sensor for this
	add_topic("distance_sensor", AUTO, 0);  // 50 Hz, downward lidar
	add_topic("distance_sensor", AUTO, 1);  // ~0 Hz, we dont have a forward lidar
}

void LoggedTopics::add_thermal_calibration_topics()
{
	add_topic_multi("sensor_accel", 100, 4);
	add_topic_multi("sensor_baro", 100, 4);
	add_topic_multi("sensor_gyro", 100, 4);
	add_topic_multi("sensor_mag", 100, 4);
}

void LoggedTopics::add_sensor_comparison_topics()
{
	add_topic_multi("sensor_accel", 100, 4);
	add_topic_multi("sensor_baro", 100, 4);
	add_topic_multi("sensor_gyro", 100, 4);
	add_topic_multi("sensor_mag", 100, 4);
}

void LoggedTopics::add_vision_and_avoidance_topics()
{
	add_topic("collision_constraints");
	add_topic("obstacle_distance_fused");
	add_topic("vehicle_mocap_odometry", 30);
	add_topic("vehicle_trajectory_waypoint", 200);
	add_topic("vehicle_trajectory_waypoint_desired", 200);
	add_topic("vehicle_visual_odometry", 30);
}

void LoggedTopics::add_raw_imu_gyro_fifo()
{
	add_topic("sensor_gyro_fifo");
}

void LoggedTopics::add_raw_imu_accel_fifo()
{
	add_topic("sensor_accel_fifo");
}

void LoggedTopics::add_system_identification_topics()
{
	// for system id need to log imu and controls at full rate
	add_topic("sensor_combined");
	add_topic("vehicle_angular_velocity");
	add_topic("vehicle_torque_setpoint");
}

void LoggedTopics::add_mavlink_tunnel()
{
	add_topic("mavlink_tunnel");
}

int LoggedTopics::add_topics_from_file(const char *fname)
{
	int ntopics = 0;

	/* open the topic list file */
	FILE *fp = fopen(fname, "r");

	if (fp == nullptr) {
		return -1;
	}

	/* call add_topic for each topic line in the file */
	for (;;) {
		/* get a line, bail on error/EOF */
		char line[80];
		line[0] = '\0';

		if (fgets(line, sizeof(line), fp) == nullptr) {
			break;
		}

		/* skip comment lines */
		if ((strlen(line) < 2) || (line[0] == '#')) {
			continue;
		}

		// read line with format: <topic_name>[ <interval>[ <instance>]]
		char topic_name[80];
		uint32_t interval_ms = 0;
		uint32_t instance = 0;
		int nfields = sscanf(line, "%s %" PRIu32 " %" PRIu32, topic_name, &interval_ms, &instance);

		if (nfields > 0) {
			int name_len = strlen(topic_name);

			if (name_len > 0 && topic_name[name_len - 1] == ',') {
				topic_name[name_len - 1] = '\0';
			}

			/* add topic with specified interval_ms */
			if ((nfields > 2 && add_topic(topic_name, interval_ms, instance))
			    || add_topic_multi(topic_name, interval_ms)) {
				ntopics++;

			} else {
				PX4_ERR("Failed to add topic %s", topic_name);
			}
		}
	}

	fclose(fp);
	return ntopics;
}

void LoggedTopics::initialize_mission_topics(MissionLogType mission_log_type)
{
	if (mission_log_type == MissionLogType::Complete) {
		add_mission_topic("camera_capture");
		add_mission_topic("mission_result");
		add_mission_topic("vehicle_global_position", 1000);
		add_mission_topic("vehicle_status", 1000);

	} else if (mission_log_type == MissionLogType::Geotagging) {
		add_mission_topic("camera_capture");
	}
}

void LoggedTopics::add_mission_topic(const char *name, uint16_t interval_ms)
{
	if (add_topic(name, interval_ms)) {
		++_num_mission_subs;
	}
}

bool LoggedTopics::add_topic(const orb_metadata *topic, uint16_t interval_ms, uint8_t instance, bool optional)
{
	if (_subscriptions.count >= MAX_TOPICS_NUM) {
		PX4_WARN("Too many subscriptions, failed to add: %s %" PRIu8, topic->o_name, instance);
		return false;
	}

	if (optional && orb_exists(topic, instance) != 0) {
		PX4_DEBUG("Not adding non-existing optional topic %s %i", topic->o_name, instance);

		if (instance == 0 && _subscriptions.num_excluded_optional_topic_ids < MAX_EXCLUDED_OPTIONAL_TOPICS_NUM) {
			_subscriptions.excluded_optional_topic_ids[_subscriptions.num_excluded_optional_topic_ids++] = topic->o_id;
		}

		return false;
	}

	RequestedSubscription &sub = _subscriptions.sub[_subscriptions.count++];
	sub.interval_ms = interval_ms;
	sub.instance = instance;
	sub.id = static_cast<ORB_ID>(topic->o_id);
	return true;
}

bool LoggedTopics::add_topic(const char *name, uint16_t interval_ms, uint8_t instance, bool optional)
{
	interval_ms /= _rate_factor;

	const orb_metadata *const *topics = orb_get_topics();
	bool success = false;

	for (size_t i = 0; i < orb_topics_count(); i++) {
		if (strcmp(name, topics[i]->o_name) == 0) {
			bool already_added = false;

			// check if already added: if so, only update the interval
			for (int j = 0; j < _subscriptions.count; ++j) {
				if (_subscriptions.sub[j].id == static_cast<ORB_ID>(topics[i]->o_id) &&
				    _subscriptions.sub[j].instance == instance) {

					PX4_DEBUG("logging topic %s(%" PRIu8 "), interval: %" PRIu16 ", already added, only setting interval",
						  topics[i]->o_name, instance, interval_ms);

					_subscriptions.sub[j].interval_ms = interval_ms;
					success = true;
					already_added = true;
					break;
				}
			}

			if (!already_added) {
				success = add_topic(topics[i], interval_ms, instance, optional);

				if (success) {
					PX4_DEBUG("logging topic: %s(%" PRIu8 "), interval: %" PRIu16, topics[i]->o_name, instance, interval_ms);
				}

				break;
			}
		}
	}

	return success;
}

bool LoggedTopics::add_topic_multi(const char *name, uint16_t interval_ms, uint8_t max_num_instances, bool optional)
{
	// add all possible instances
	for (uint8_t instance = 0; instance < max_num_instances; instance++) {
		add_topic(name, interval_ms, instance, optional);
	}

	return true;
}

bool LoggedTopics::initialize_logged_topics(SDLogProfileMask profile)
{
	int ntopics = add_topics_from_file(PX4_STORAGEDIR "/etc/logging/logger_topics.txt");

	if (ntopics > 0) {
		PX4_INFO("logging %d topics from logger_topics.txt", ntopics);

	} else {
		initialize_configured_topics(profile);
	}

	return _subscriptions.count > 0;
}

void LoggedTopics::initialize_configured_topics(SDLogProfileMask profile)
{
	// load appropriate topics for profile
	// the order matters: if several profiles add the same topic, the logging rate of the last one will be used
	if (profile & SDLogProfileMask::DEFAULT) {
		add_default_topics();
	}

	if (profile & SDLogProfileMask::ESTIMATOR_REPLAY) {
		add_estimator_replay_topics();
	}

	if (profile & SDLogProfileMask::THERMAL_CALIBRATION) {
		add_thermal_calibration_topics();
	}

	if (profile & SDLogProfileMask::SYSTEM_IDENTIFICATION) {
		add_system_identification_topics();
	}

	if (profile & SDLogProfileMask::HIGH_RATE) {
		add_high_rate_topics();
	}

	if (profile & SDLogProfileMask::DEBUG_TOPICS) {
		add_debug_topics();
	}

	if (profile & SDLogProfileMask::SENSOR_COMPARISON) {
		add_sensor_comparison_topics();
	}

	if (profile & SDLogProfileMask::VISION_AND_AVOIDANCE) {
		add_vision_and_avoidance_topics();
	}

	if (profile & SDLogProfileMask::RAW_IMU_GYRO_FIFO) {
		add_raw_imu_gyro_fifo();
	}

	if (profile & SDLogProfileMask::RAW_IMU_ACCEL_FIFO) {
		add_raw_imu_accel_fifo();
	}

	if (profile & SDLogProfileMask::MAVLINK_TUNNEL) {
		add_mavlink_tunnel();
	}
}
