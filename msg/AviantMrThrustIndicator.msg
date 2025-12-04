uint64 timestamp
float32 thrust_max # Estimated required commanded thrust on the heaviest loaded motor
float32 thrust_max_bomi # Estimated required commanded thrust on the heaviest loaded motor, in case one motor becomes inoperative.

# Mirrors MAVLink message AVIANT_INDICATOR_STATE
uint8 THRUST_STATUS_INACTIVE = 0
uint8 THRUST_STATUS_NOMINAL = 1
uint8 THRUST_STATUS_WARNING = 2
uint8 THRUST_STATUS_CRITICAL = 3
uint8 status
