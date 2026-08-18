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

# --- serial (UART) monitoring ---------------------------------------------------
# Filled by the gps module from device::Serial and by the u-blox parser. The peak and
# the three counters are per RATE_MEASUREMENT_PERIOD (5 s) window; uart_rx_dropped and
# driver_resync_bytes are monotonic since the driver last (re)configured the receiver.

uint32 uart_rx_dropped		# bytes discarded by the kernel because the serial RX buffer was full. Needs the NuttX TIOCGRXDROP patch, otherwise always 0
uint32 driver_resync_bytes	# bytes the UBX parser threw away while hunting for the next 0xB5 0x62 sync pair. Non-zero means the byte stream was damaged

uint16 uart_rx_buf_peak		# high-water mark of the kernel serial RX ring buffer (bytes). Compare against CONFIG_USARTn_RXBUFSIZE, 600 on most boards
uint16 uart_reads_saturated	# read() calls that completely filled the driver's 150 byte buffer
uint16 uart_poll_timeouts	# readAtLeast() calls that returned no data at all
uint16 uart_read_errors		# read() calls that returned an error
uint16 driver_sync_errors	# UBX 0xB5 not followed by 0x62, i.e. a truncated or corrupted message header


# --- moving-baseline cross-link monitoring --------------------------------------
# The moving base sends RTCM straight to the rover over a direct receiver-to-receiver UART2
# link that never passes through the autopilot, so none of the counters above can see it.
# These fields watch that link from both ends: what the rover reports receiving
# (UBX-RXM-RTCM) and what each receiver reports moving through its own UART2 port
# (UBX-MON-COMMS). Comparing the base's crosslink_tx_bytes against the rover's
# crosslink_rx_bytes measures loss on the wire itself.

uint32 rtcm_msgs_received	# monotonic count of RTCM frames this receiver decoded. On the moving-baseline rover these are the frames arriving from the moving base
uint32 rtcm_crc_errors		# monotonic count of RTCM frames that failed CRC. rtcm_crc_failed above is a single bool that is overwritten several times a second and can never be sampled reliably
uint32 crosslink_tx_bytes	# UBX-MON-COMMS txBytes for this receiver's UART2 port: on the moving base, RTCM actually pushed towards the rover
uint32 crosslink_rx_bytes	# UBX-MON-COMMS rxBytes for the same port: on the rover, bytes actually arriving from the moving base
uint32 crosslink_skipped_bytes	# UBX-MON-COMMS skipped: bytes received on that port and thrown away because they matched no enabled protocol, i.e. corrupted RTCM framing

uint16 rtcm_age_ms		# time since the last RTCM frame was decoded (ms), i.e. age of the moving-base corrections. Nominally below 125 ms at the 8 Hz moving-baseline rate. Counted from the end of configuration while no frame has arrived yet
uint16 rtcm_age_peak_ms		# worst rtcm_age_ms since boot: one number per flight for the longest cross-link outage
uint16 crosslink_overrun_errors	# UBX-MON-COMMS overrunErrs for that port: the receiver's own UART2 receive buffer overflowed
uint16 crosslink_port_id	# UBX-MON-COMMS portId the crosslink_* fields came from. 0 = MON-COMMS never arrived, 65535 = it did but reported no UART2 port

uint8 rtcm_types_seen		# bitmask of the RTCM types decoded in the last second, see RTCM_SEEN_* below. A bit that stops being set means the base stopped sending that type
uint8 crosslink_tx_peak_usage	# UBX-MON-COMMS txPeakUsage (percent) for that port
uint8 crosslink_rx_peak_usage	# UBX-MON-COMMS rxPeakUsage (percent) for that port

uint8 RTCM_SEEN_1074 = 1	# GPS MSM4
uint8 RTCM_SEEN_1084 = 2	# GLONASS MSM4
uint8 RTCM_SEEN_1094 = 4	# Galileo MSM4
uint8 RTCM_SEEN_1124 = 8	# BeiDou MSM4
uint8 RTCM_SEEN_1230 = 16	# GLONASS code-phase biases
uint8 RTCM_SEEN_4072 = 32	# u-blox proprietary reference station message. The moving-baseline solution depends on this one
uint8 RTCM_SEEN_OTHER = 64	# any other type, e.g. a static base stream injected by the autopilot

# TOPICS sensor_gps vehicle_gps_position
