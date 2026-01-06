# VEHICLE_VTOL_STATE, should match 1:1 MAVLinks's MAV_VTOL_STATE
uint8 VEHICLE_VTOL_STATE_UNDEFINED = 0
uint8 VEHICLE_VTOL_STATE_TRANSITION_TO_FW = 1
uint8 VEHICLE_VTOL_STATE_TRANSITION_TO_MC = 2
uint8 VEHICLE_VTOL_STATE_MC = 3
uint8 VEHICLE_VTOL_STATE_FW = 4

uint64 timestamp			# time since system start (microseconds)

bool vtol_in_rw_mode			# true: vtol vehicle is in rotating wing mode
bool vtol_in_trans_mode
bool in_transition_to_fw		# True if VTOL is doing a transition from MC to FW
bool vtol_transition_failsafe		# vtol in transition failsafe mode
bool fw_permanent_stab			# In fw mode stabilize attitude even if in manual mode

float32 airspeed_filtered		# Filtered airspeed used during transition. NAN if invalid
float32[4] mc_weights			# Multicopter blending weights for [roll, pitch, yaw, throttle]

# Euler-based lookahead (old method with gimbal lock)
float32 lookahead_roll_euler		# Predicted roll angle using Euler rates [rad]
float32 lookahead_pitch_euler		# Predicted pitch angle using Euler rates [rad]

# Quaternion-based lookahead (new method, singularity-free)
float32 lookahead_roll_quat		# Predicted roll angle using quaternion integration [rad]
float32 lookahead_pitch_quat		# Predicted pitch angle using quaternion integration [rad]
