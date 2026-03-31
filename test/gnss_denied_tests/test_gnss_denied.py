"""SITL integration tests for GNSS-denied EKF instances.

Each test starts a fresh PX4 SITL with GNSS-denied mode enabled
(PX4_EKF2_GNSS_DENIED=1).  SIH provides simulated sensors.

Tests 1, 4 need only a booted PX4 with EKF2 running (no flight).
Tests 2, 3 need armed flight.
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
    """External position estimate resets GNSS-denied instance, not normal.

    1. Takeoff and hold — the GNSS-denied instance stops fusing GPS on
       arming and enters dead reckoning within ~1 s sim time.
    2. Send an external position reset to current position + 100 m east.
    3. Assert LOCAL_POSITION does not jump (< 1 m) and TRN position jumps
       significantly (> 80 m).

    Note: SIH provides near-perfect IMU data so the GNSS-denied instance
    drifts back toward reality after a reset.  A 100 m offset ensures the
    jump is still clearly visible when TRN is read ~1 s sim-time later.
    """
    t0 = _ts()

    # Drain any pending ACKs so they don't interfere with arm ACK
    while tester.conn.recv_match(type='COMMAND_ACK', blocking=False) is not None:
        pass

    # -- 1. Takeoff and hold ------------------------------------------------
    log.info("Commanding takeoff to 10m...")
    tester.takeoff(alt=10.0)

    # At 10× speed the vehicle reaches altitude quickly — wait 1 s wall
    # (= 10 s sim) to stay well within the armed window.
    log.info("Waiting 1s for climb...")
    time.sleep(1)

    # Verify the vehicle is actually armed
    hb = tester.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=3.0)
    assert hb and (hb.base_mode & 128), \
        "Vehicle not armed — cannot test external position reset"

    # -- 2. External position reset -----------------------------------------
    #    Reset target = current position + 100 m east (in lat/lon).
    trn_before = tester.get_gnss_denied_position()
    log.info("TRN before reset: x=%.2f y=%.2f z=%.2f", *trn_before)
    assert math.isfinite(trn_before[0]), "TRN position not finite"

    local_before = tester.get_local_position()
    log.info("LOCAL before reset: x=%.2f y=%.2f z=%.2f", *local_before)

    cur_lat, cur_lon = tester.get_global_position()
    # 111320 m/deg is Earth's circumference / 360; cos(lat) corrects for longitude convergence
    lon_offset = 100.0 / (111320.0 * math.cos(math.radians(cur_lat)))
    tester.send_external_position_estimate(cur_lat, cur_lon + lon_offset)

    log.info("Waiting 0.1s for reset to take effect...")
    time.sleep(0.1)

    # -- 3. Assertions ------------------------------------------------------
    # LOCAL_POSITION should not jump.
    local_after = tester.get_local_position()
    local_jump = math.sqrt(
        (local_after[0] - local_before[0]) ** 2
        + (local_after[1] - local_before[1]) ** 2)
    log.info("LOCAL after reset: x=%.2f y=%.2f z=%.2f  jump=%.2fm",
             *local_after, local_jump)
    assert local_jump < 1.0, \
        f"LOCAL_POSITION jumped {local_jump:.2f}m -- reset leaked into normal instance"

    # TRN position should jump toward the reset target.
    trn_after = tester.get_gnss_denied_position()
    trn_jump = math.sqrt(
        (trn_after[0] - trn_before[0]) ** 2
        + (trn_after[1] - trn_before[1]) ** 2)
    log.info("TRN after reset: x=%.2f y=%.2f z=%.2f  jump=%.2fm",
             *trn_after, trn_jump)
    assert trn_jump > 80.0, \
        f"TRN position should jump significantly, got {trn_jump:.2f}m -- reset not accepted"

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
