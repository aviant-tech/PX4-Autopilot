uint64 timestamp

# Mirrors MAVLink enum AVIANT_INDICATOR_STATE
uint8 STATE_INACTIVE = 0
uint8 STATE_NOMINAL = 1
uint8 STATE_WARNING = 2
uint8 STATE_CRITICAL = 3

uint8 NUM_MOTORS = 12

int16[12] mot_voltage_filtered_cv	# [cV] effective voltage per motor
uint8[12] mot_load_status		# per-motor load state

int16 anomalous_current_filtered_ca	# [cA] filtered current prediction error
float32 predicted_current_a		# [A] instantaneous total predicted bus current
uint8 anomalous_current_status
