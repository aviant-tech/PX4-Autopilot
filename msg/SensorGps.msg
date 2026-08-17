# GPS position in WGS84 coordinates.
# the field 'timestamp' is for the position & velocity (microseconds)
uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id                # unique device ID for the sensor that does not change between power cycles

float64 latitude_deg		# Latitude in degrees, allows centimeter level RTK precision
float64 longitude_deg		# Longitude in degrees, allows centimeter level RTK precision
float64 altitude_msl_m		# Altitude above MSL, meters
float64 altitude_ellipsoid_m	# Altitude above Ellipsoid, meters

float32 s_variance_m_s		# GPS speed accuracy estimate, (metres/sec)
float32 c_variance_rad		# GPS course accuracy estimate, (radians)
uint8 FIX_TYPE_NONE                   = 1       # Value 0 is also valid to represent no fix.
uint8 FIX_TYPE_2D                     = 2
uint8 FIX_TYPE_3D                     = 3
uint8 FIX_TYPE_RTCM_CODE_DIFFERENTIAL = 4
uint8 FIX_TYPE_RTK_FLOAT              = 5
uint8 FIX_TYPE_RTK_FIXED              = 6
uint8 FIX_TYPE_EXTRAPOLATED           = 8
uint8 fix_type                  # Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.

float32 eph			# GPS horizontal position accuracy (metres)
float32 epv			# GPS vertical position accuracy (metres)

float32 hdop			# Horizontal dilution of precision
float32 vdop			# Vertical dilution of precision

int32 noise_per_ms		# GPS noise per millisecond
uint16 automatic_gain_control   # Automatic gain control monitor

uint8 JAMMING_STATE_UNKNOWN  = 0
uint8 JAMMING_STATE_OK       = 1
uint8 JAMMING_STATE_WARNING  = 2
uint8 JAMMING_STATE_CRITICAL = 3
uint8 jamming_state		# indicates whether jamming has been detected or suspected by the receivers. O: Unknown, 1: OK, 2: Warning, 3: Critical
int32 jamming_indicator		# indicates jamming is occurring

uint8 SPOOFING_STATE_UNKNOWN   = 0
uint8 SPOOFING_STATE_NONE      = 1
uint8 SPOOFING_STATE_INDICATED = 2
uint8 SPOOFING_STATE_MULTIPLE  = 3
uint8 spoofing_state		# indicates whether spoofing has been detected or suspected by the receivers. O: Unknown, 1: OK, 2: Warning, 3: Critical

float32 vel_m_s			# GPS ground speed, (metres/sec)
float32 vel_n_m_s		# GPS North velocity, (metres/sec)
float32 vel_e_m_s		# GPS East velocity, (metres/sec)
float32 vel_d_m_s		# GPS Down velocity, (metres/sec)
float32 cog_rad			# Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)
bool vel_ned_valid		# True if NED velocity is valid

int32 timestamp_time_relative	# timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)
uint64 time_utc_usec		# Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0

uint8 satellites_used		# Number of satellites used

float32 heading			# heading angle of XYZ body frame rel to NED. Set to NaN if not available and updated (used for dual antenna GPS), (rad, [-PI, PI])
float32 heading_offset		# heading offset of dual antenna array in body frame. Set to NaN if not applicable. (rad, [-PI, PI])
float32 heading_accuracy	# heading accuracy (rad, [0, 2PI])

float32 rtcm_injection_rate	# RTCM message injection rate Hz
uint8 selected_rtcm_instance	# uorb instance that is being used for RTCM corrections

bool rtcm_crc_failed		# RTCM message CRC failure detected

uint8 RTCM_MSG_USED_UNKNOWN = 0
uint8 RTCM_MSG_USED_NOT_USED = 1
uint8 RTCM_MSG_USED_USED = 2
uint8 rtcm_msg_used		# Indicates if the RTCM message was used successfully by the receiver

# --- GNSS debug instrumentation -------------------------------------------------
# Filled by the u-blox driver (PX4-GPSDrivers ubx.cpp) and by the gps module.
# Everything below is diagnostic only, nothing in the flight stack consumes it.

uint32 receiver_msss		# UBX-NAV-STATUS msss: milliseconds since the receiver started. A DECREASE means the receiver rebooted by itself. 0 = not reported
uint32 receiver_ttff		# UBX-NAV-STATUS ttff: time to first fix of the current receiver run (ms)
uint32 rf_post_status		# UBX-MON-RF postStatus: receiver power-on self test status word
uint32 driver_rx_bytes		# bytes read from this receiver by the autopilot since the driver last (re)configured it

uint16 driver_msg_gap_ms	# time between the last two complete position messages, measured by the driver (ms)
uint16 driver_ck_errors		# cumulative UBX checksum failures since the driver last (re)configured the receiver

uint8 antenna_status		# UBX-MON-RF antStatus: 0=init, 1=unknown, 2=ok, 3=short, 4=open
uint8 antenna_power		# UBX-MON-RF antPower: 0=off, 1=on, 2=unknown
uint8 rf_mag_i			# UBX-MON-RF magI: magnitude of I-part of complex signal (0=no signal, 255=max)
uint8 rf_mag_q			# UBX-MON-RF magQ: magnitude of Q-part of complex signal
int8 rf_ofs_i			# UBX-MON-RF ofsI: imbalance of I-part of complex signal
int8 rf_ofs_q			# UBX-MON-RF ofsQ: imbalance of Q-part of complex signal

# TOPICS sensor_gps vehicle_gps_position
