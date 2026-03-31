"""SITL integration tests for GNSS-denied EKF instances.

Each test starts a fresh PX4 SITL with GNSS-denied mode enabled
(PX4_EKF2_GNSS_DENIED=1).  SIH provides simulated sensors.

Tests 1, 2, 4 need only a booted PX4 with EKF2 running (no flight).
Test 3 needs armed flight to verify GPS-loss failsafe behavior.
"""

import logging
import math
import time

import pytest
from gnss_denied_tester import GnssDeniedTester

log = logging.getLogger(__name__)


def _ts():
    """Monotonic timestamp for relative timing."""
    return time.monotonic()


def _elapsed(start: float) -> str:
    return f"{time.monotonic() - start:.2f}s"


def test_gnss_denied_instance_exists(tester: GnssDeniedTester):
    """GNSS-denied EKF instance runs and produces finite position data."""
    t0 = _ts()
    log.info("Waiting for AVIANT_TRN_TEST_DATA...")
    trn = tester.get_trn_test_data(timeout_s=5.0)
    log.info("TRN data received after %s: %s",
             _elapsed(t0), trn is not None)
    assert trn is not None, \
        "No AVIANT_TRN_TEST_DATA -- GNSS-denied instance not running"

    log.info("TRN pos: x=%.2f y=%.2f z=%.2f", trn.x, trn.y, trn.z)
    assert math.isfinite(trn.x)
    assert math.isfinite(trn.y)
    assert math.isfinite(trn.z)

    log.info("Waiting for LOCAL_POSITION_NED...")
    t1 = _ts()
    pos = tester.get_local_position()
    log.info("Normal pos after %s: x=%.2f y=%.2f z=%.2f",
             _elapsed(t1), *pos)
    assert math.isfinite(pos[0])
    assert math.isfinite(pos[1])
    assert math.isfinite(pos[2])

    log.info("Test complete in %s", _elapsed(t0))


def test_external_position_resets_only_gnss_denied(tester: GnssDeniedTester):
    """External position estimate resets GNSS-denied instance, not normal."""
    t0 = _ts()

    log.info("Getting GNSS-denied position before reset...")
    t1 = _ts()
    pos_before = tester.get_gnss_denied_position()
    log.info("GNSS-denied pos (before) after %s: x=%.2f y=%.2f z=%.2f",
             _elapsed(t1), *pos_before)
    assert math.isfinite(pos_before[0]), "Pre-reset position not finite"

    log.info("Getting normal position before reset...")
    t1 = _ts()
    normal_before = tester.get_local_position()
    log.info("Normal pos (before) after %s: x=%.2f y=%.2f z=%.2f",
             _elapsed(t1), *normal_before)

    log.info("Getting home position...")
    t1 = _ts()
    home_lat, home_lon = tester.get_home()
    log.info("Home after %s: lat=%.6f lon=%.6f", _elapsed(t1), home_lat, home_lon)

    log.info("Sending EXTERNAL_POSITION_ESTIMATE (~50m north)...")
    tester.send_external_position_estimate(home_lat + 0.00045, home_lon)

    log.info("Waiting 2s for reset to take effect...")
    time.sleep(2)

    log.info("Getting GNSS-denied position after reset...")
    t1 = _ts()
    pos_after = tester.get_gnss_denied_position()
    log.info("GNSS-denied pos (after) after %s: x=%.2f y=%.2f z=%.2f",
             _elapsed(t1), *pos_after)

    dx = pos_after[0] - pos_before[0]
    dy = pos_after[1] - pos_before[1]
    change = math.sqrt(dx * dx + dy * dy)
    log.info("GNSS-denied horizontal change: %.1fm", change)
    assert change > 10.0, \
        f"GNSS-denied position should jump >10m, got {change:.1f}m"

    log.info("Getting normal position after reset...")
    t1 = _ts()
    normal_after = tester.get_local_position()
    log.info("Normal pos (after) after %s: x=%.2f y=%.2f z=%.2f",
             _elapsed(t1), *normal_after)

    drift = math.sqrt(
        (normal_after[0] - normal_before[0]) ** 2
        + (normal_after[1] - normal_before[1]) ** 2)
    log.info("Normal instance drift: %.2fm", drift)
    assert drift < 2.0, \
        f"Normal instance drifted {drift:.1f}m -- should be stationary"

    log.info("Test complete in %s", _elapsed(t0))


def test_selector_ignores_gnss_denied_on_gps_loss(tester: GnssDeniedTester):
    """On GPS loss the selector uses a normal instance (vehicle failsafes)."""
    t0 = _ts()

    log.info("Setting SYS_FAILURE_EN=1...")
    tester.set_param_int('SYS_FAILURE_EN', 1)

    log.info("Commanding takeoff to 10m...")
    t1 = _ts()
    tester.takeoff(alt=10.0)
    log.info("Takeoff command sent after %s", _elapsed(t1))

    log.info("Waiting 5s for climb...")
    time.sleep(5)

    log.info("Injecting GPS failure...")
    t1 = _ts()
    tester.inject_gps_failure()
    log.info("GPS failure injected at %s", _elapsed(t0))

    log.info("Waiting for disarm (failsafe land)...")
    t1 = _ts()
    tester.wait_until_disarmed(timeout_s=300)
    log.info("Disarmed after %s (total: %s)", _elapsed(t1), _elapsed(t0))


def test_trn_data_stream_valid(tester: GnssDeniedTester):
    """AVIANT_TRN_TEST_DATA stream has valid fields and unit quaternion."""
    t0 = _ts()

    log.info("Collecting TRN data for 5s...")
    messages = tester.collect_trn_data(duration_s=5.0)
    log.info("Collected %d messages in %s", len(messages), _elapsed(t0))
    assert len(messages) >= 5, \
        f"Expected >= 5 TRN messages in 5s, got {len(messages)}"

    data = messages[-1]
    log.info("Last message: time_boot_ms=%d pos=(%.2f, %.2f, %.2f) "
             "vel=(%.2f, %.2f, %.2f) quat=(%.3f, %.3f, %.3f, %.3f)",
             data.time_boot_ms, data.x, data.y, data.z,
             data.vx, data.vy, data.vz,
             data.q1, data.q2, data.q3, data.q4)

    assert data.time_boot_ms > 0
    for field in ('x', 'y', 'z', 'vx', 'vy', 'vz', 'q1', 'q2', 'q3', 'q4'):
        assert math.isfinite(getattr(data, field)), f"{field} is not finite"

    quat_norm = math.sqrt(data.q1**2 + data.q2**2 + data.q3**2 + data.q4**2)
    log.info("Quaternion norm: %.4f", quat_norm)
    assert abs(quat_norm - 1.0) < 0.01, \
        f"Quaternion norm should be ~1.0, got {quat_norm:.4f}"

    log.info("Test complete in %s", _elapsed(t0))
