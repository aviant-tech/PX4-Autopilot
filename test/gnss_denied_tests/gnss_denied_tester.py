"""Helper class for GNSS-denied SITL tests using pymavlink.

Thin wrapper around a pymavlink connection for receiving
AVIANT_TRN_TEST_DATA, sending external position estimates,
and injecting GPS failures.

Requires the ``aviant`` MAVLink dialect (set MAVLINK_DIALECT=aviant and
MDEF pointing at the in-tree message_definitions before importing
pymavlink).  See conftest.py.
"""

from __future__ import annotations

import math
import time

from pymavlink import mavutil

mavlink = mavutil.mavlink

MAV_CMD_EXTERNAL_POSITION_ESTIMATE = 43003


class GnssDeniedTester:
    """Drives GNSS-denied test scenarios over MAVLink."""

    def __init__(self, connection: mavutil.mavlink_connection):
        self.conn = connection
        self._target_sys = 1
        self._target_comp = 1

    # ------------------------------------------------------------------
    # TRN test data (AVIANT_TRN_TEST_DATA)
    # ------------------------------------------------------------------

    def get_trn_test_data(self, timeout_s: float = 5.0):
        """Receive a fresh AVIANT_TRN_TEST_DATA message."""
        while self.conn.recv_match(
                type='AVIANT_TRN_TEST_DATA', blocking=False) is not None:
            pass
        return self.conn.recv_match(
            type='AVIANT_TRN_TEST_DATA', blocking=True, timeout=timeout_s)

    def get_gnss_denied_position(self, timeout_s: float = 5.0):
        """Return (x, y, z) from AVIANT_TRN_TEST_DATA."""
        msg = self.get_trn_test_data(timeout_s=timeout_s)
        if msg is None:
            return (math.nan, math.nan, math.nan)
        return (msg.x, msg.y, msg.z)

    def collect_trn_data(self, duration_s: float = 5.0) -> list:
        """Collect AVIANT_TRN_TEST_DATA messages for a fixed duration."""
        # Drain stale messages.
        while self.conn.recv_match(
                type='AVIANT_TRN_TEST_DATA', blocking=False) is not None:
            pass

        messages = []
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            msg = self.conn.recv_match(
                type='AVIANT_TRN_TEST_DATA', blocking=True, timeout=0.5)
            if msg is not None:
                messages.append(msg)
        return messages

    # ------------------------------------------------------------------
    # Local position (normal EKF instance)
    # ------------------------------------------------------------------

    def get_local_position(self, timeout_s: float = 3.0):
        """Return (x, y, z) from LOCAL_POSITION_NED."""
        msg = self.conn.recv_match(
            type='LOCAL_POSITION_NED', blocking=True, timeout=timeout_s)
        if msg is None:
            return (math.nan, math.nan, math.nan)
        return (msg.x, msg.y, msg.z)

    def get_global_position(self, timeout_s: float = 3.0):
        """Return (lat_deg, lon_deg) from GLOBAL_POSITION_INT."""
        msg = self.conn.recv_match(
            type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout_s)
        if msg is None:
            return (math.nan, math.nan)
        return (msg.lat / 1e7, msg.lon / 1e7)

    # ------------------------------------------------------------------
    # External position estimate
    # ------------------------------------------------------------------

    def send_external_position_estimate(self, lat: float, lon: float,
                                        accuracy: float = 10.0):
        """Send MAV_CMD_EXTERNAL_POSITION_ESTIMATE via COMMAND_INT for lat/lon precision."""
        att = self.conn.recv_match(type='ATTITUDE', blocking=True, timeout=3.0)
        if att is None:
            raise TimeoutError('No ATTITUDE message for timestamp')

        self.conn.mav.command_int_send(
            self._target_sys, self._target_comp,
            0,                                  # frame (unused)
            MAV_CMD_EXTERNAL_POSITION_ESTIMATE,
            0, 0,                               # current, autocontinue
            att.time_boot_ms / 1000.0,          # param1: FC timestamp
            0.0,                                # param2: processing_time
            accuracy,                           # param3: accuracy
            0.0,                                # param4: empty
            int(lat * 1e7),                     # x: latitude (degE7)
            int(lon * 1e7),                     # y: longitude (degE7)
            float('nan'),                       # z: altitude (unused)
        )

    # ------------------------------------------------------------------
    # Home position
    # ------------------------------------------------------------------

    def get_home(self, timeout_s: float = 10.0):
        """Return (lat_deg, lon_deg) from HOME_POSITION."""
        msg = self.conn.recv_match(
            type='HOME_POSITION', blocking=True, timeout=timeout_s)
        if msg is None:
            raise TimeoutError('Did not receive HOME_POSITION')
        return (msg.latitude / 1e7, msg.longitude / 1e7)

    # ------------------------------------------------------------------
    # GPS failure injection
    # ------------------------------------------------------------------

    def inject_gps_failure(self):
        """Disable GPS via MAV_CMD_INJECT_FAILURE."""
        self.conn.mav.command_long_send(
            self._target_sys, self._target_comp,
            mavlink.MAV_CMD_INJECT_FAILURE, 0,
            4.0,  # FAILURE_UNIT_SENSOR_GPS
            1.0,  # FAILURE_TYPE_OFF
            0.0,  # instance (0 = all)
            0, 0, 0, 0,
        )

    # ------------------------------------------------------------------
    # Arming / takeoff / disarm detection
    # ------------------------------------------------------------------

    def takeoff(self, alt: float = 10.0):
        """Arm and takeoff to altitude via MAV_CMD_NAV_TAKEOFF."""
        self.conn.mav.command_long_send(
            self._target_sys, self._target_comp,
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1.0, 0, 0, 0, 0, 0, 0,
        )
        self._wait_ack(mavlink.MAV_CMD_COMPONENT_ARM_DISARM)

        self.conn.mav.command_long_send(
            self._target_sys, self._target_comp,
            mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, alt,
        )
        self._wait_ack(mavlink.MAV_CMD_NAV_TAKEOFF)

    def wait_until_disarmed(self, timeout_s: float = 60.0):
        """Block until the vehicle disarms."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            msg = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
            if msg and not (msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return
        raise TimeoutError(f'Vehicle did not disarm within {timeout_s}s')

    def set_param_int(self, param_id: str, value: int):
        """Set a PX4 integer parameter."""
        self.conn.mav.param_set_send(
            self._target_sys, self._target_comp,
            param_id.encode('utf-8'),
            float(value),
            mavlink.MAV_PARAM_TYPE_INT32,
        )

    def land(self):
        """Command the vehicle to land at the current position."""
        self.conn.mav.command_long_send(
            self._target_sys, self._target_comp,
            mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0, 0, 0, 0,
        )
        self._wait_ack(mavlink.MAV_CMD_NAV_LAND)

    def _wait_ack(self, command: int, timeout_s: float = 5.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            msg = self.conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.5)
            if msg and msg.command == command:
                return msg
        return None

    def shutdown(self):
        pass
