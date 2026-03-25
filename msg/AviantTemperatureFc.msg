uint64 timestamp

# Mirrors MAVLink enum AVIANT_INDICATOR_STATE
uint8 STATE_INACTIVE = 0
uint8 STATE_NOMINAL = 1
uint8 STATE_WARNING = 2
uint8 STATE_CRITICAL = 3

uint8 battery_internal_state
int8 battery_internal		# [degC]

uint8 airspeed_internal_state
int8 airspeed_internal		# [degC]

uint8 imu_internal_state
int8 imu_internal		# [degC]

uint8 baro_internal_state
int8 baro_internal		# [degC]
