uint64 timestamp

# Mirrors MAVLink message AVIANT_NAV_REDUNDANCY
uint8 REDUNDANCY_UNKNOWN = 0
uint8 REDUNDANCY_REDUNDANT = 1
uint8 REDUNDANCY_SINGLE = 2
uint8 REDUNDANCY_FAILED = 3
uint8 redundancy

# Mirrors MAVLink message AVIANT_NAV_ACCURACY
uint8 ACCURACY_UNKNOWN = 0
uint8 ACCURACY_ACCURATE = 1
uint8 ACCURACY_APPROXIMATE = 2
uint8 ACCURACY_FAILED = 3
uint8 accuracy

# Diagnostic data showing which checks have passed
bool lpos_valid
bool gnss_absolute_rtk_float
bool gnss_fallback_available
bool mag_fused_recently
bool mag_innovations_ok
bool baro_fused_recently
bool ekf_rtk_heading_used
bool ekf_gnss_pos_fused_recently
