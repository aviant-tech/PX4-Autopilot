"""pytest fixtures for GNSS-denied SITL tests.

Each test gets a fresh PX4 SITL instance with GNSS-denied EKF instances
enabled (PX4_EKF2_GNSS_DENIED=1).  The SIH simulator provides sensor
data without needing Gazebo.

Parameters are passed as environment variables to the PX4 process.
The rcS startup script reads PX4_EKF2_GNSS_DENIED and applies
``param set SENS_IMU_MODE 0`` and ``param set EKF2_GNSS_DENIED 1``.
"""

from __future__ import annotations

import os

WORKSPACE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
DEFAULT_BUILD_DIR = os.path.join(WORKSPACE, 'build', 'px4_sitl_default')

# Build the aviant dialect from the in-tree mavlink XML definitions.
# These must be set before pymavlink is imported.
os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'aviant'
os.environ['MDEF'] = os.path.join(
    WORKSPACE, 'src', 'modules', 'mavlink', 'mavlink', 'message_definitions')

import logging
import shutil
import signal
import subprocess
import threading
import time

import psutil
import pytest
from pymavlink import mavutil

from gnss_denied_tester import GnssDeniedTester

log = logging.getLogger(__name__)

MAV_PORT = 14540


def pytest_addoption(parser):
    parser.addoption(
        '--build-dir', action='store', default=DEFAULT_BUILD_DIR,
        help='Path to the PX4 SITL build directory',
    )


# ------------------------------------------------------------------
# PX4 lifecycle helpers
# ------------------------------------------------------------------

def _kill_existing_px4():
    for proc in psutil.process_iter(['name']):
        if proc.info['name'] == 'px4':
            proc.kill()
            proc.wait(timeout=5)


class PX4Instance:
    """Wraps a running PX4 SITL process with stdout-based boot detection."""

    def __init__(self, proc: subprocess.Popen):
        self.proc = proc
        self._lines: list[str] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _read_loop(self):
        assert self.proc.stdout is not None
        while not self._stop.is_set():
            line = self.proc.stdout.readline()
            if not line:
                if self.proc.poll() is not None:
                    break
                time.sleep(0.01)
                continue
            with self._lock:
                self._lines.append(line)

    def wait_for_line(self, needle: str, timeout_s: float = 30.0) -> bool:
        """Wait until a line containing *needle* (case-insensitive) appears."""
        deadline = time.monotonic() + timeout_s
        seen = 0
        while time.monotonic() < deadline:
            with self._lock:
                while seen < len(self._lines):
                    if needle.lower() in self._lines[seen].lower():
                        return True
                    seen += 1
            time.sleep(0.05)
        return False

    def stop(self):
        self._stop.set()
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGTERM)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=3)
        self._reader.join(timeout=3)


def _start_px4(build_dir: str,
               extra_env: dict[str, str] | None = None) -> PX4Instance:
    rootfs = os.path.join(build_dir, 'tmp_gnss_denied_tests', 'rootfs')

    if os.path.isdir(rootfs):
        for item in os.listdir(rootfs):
            if item == 'log':
                continue
            path = os.path.join(rootfs, item)
            if os.path.isfile(path) or os.path.islink(path):
                os.remove(path)
            else:
                shutil.rmtree(path)
    os.makedirs(rootfs, exist_ok=True)

    env = os.environ.copy()
    env['PX4_SIM_MODEL'] = 'sihsim_quadx'
    env['PX4_SIM_SPEED_FACTOR'] = '10'
    env['PX4_EKF2_GNSS_DENIED'] = '1'
    env['SYS_FAILURE_EN'] = '1'

    if extra_env:
        env.update(extra_env)

    px4_bin = os.path.join(build_dir, 'bin', 'px4')
    etc_dir = os.path.join(build_dir, 'etc')

    proc = subprocess.Popen(
        [
            px4_bin,
            etc_dir,
            '-s', 'etc/init.d-posix/rcS',
            '-d',
        ],
        cwd=rootfs,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
    )

    instance = PX4Instance(proc)

    if not instance.wait_for_line('startup script returned', timeout_s=30):
        instance.stop()
        raise TimeoutError('PX4 did not complete boot within timeout')

    return instance


# ------------------------------------------------------------------
# Fixtures
# ------------------------------------------------------------------

@pytest.fixture()
def px4(request):
    """Start a fresh PX4 SITL for each test with GNSS-denied mode enabled.

    Tests can provide extra env vars via ``pytest.mark.parametrize``
    on the indirect ``px4`` fixture.
    """
    build_dir = os.path.abspath(request.config.getoption('--build-dir'))

    if not os.path.isfile(os.path.join(build_dir, 'bin', 'px4')):
        pytest.skip(
            f'PX4 binary not found at {build_dir}/bin/px4 -- '
            'run `DONT_RUN=1 make px4_sitl` first')

    params = getattr(request, 'param', None)

    t0 = time.monotonic()
    log.info("Killing existing PX4 instances...")
    _kill_existing_px4()
    log.info("Starting PX4 SITL (build_dir=%s, params=%s)...", build_dir, params)
    t1 = time.monotonic()
    instance = _start_px4(build_dir, extra_env=params)
    log.info("PX4 booted in %.2fs", time.monotonic() - t1)

    yield instance

    log.info("Stopping PX4 (fixture was alive for %.2fs)...", time.monotonic() - t0)
    instance.stop()
    _kill_existing_px4()


@pytest.fixture()
def tester(px4):
    """Provide a GnssDeniedTester connected to the running PX4 instance.

    Blocks until EKF2 achieves tilt alignment (publishes ATTITUDE)
    so that offboard control and GNSS-denied instances are ready.
    """
    t0 = time.monotonic()
    time.sleep(1)

    log.info("Connecting pymavlink to udpin:0.0.0.0:%d...", MAV_PORT)
    conn = mavutil.mavlink_connection(
        f'udpin:0.0.0.0:{MAV_PORT}',
        source_system=255,
        source_component=0,
    )

    log.info("Waiting for HEARTBEAT...")
    t1 = time.monotonic()
    conn.wait_heartbeat(timeout=15)
    log.info("HEARTBEAT received after %.2fs", time.monotonic() - t1)

    # Wait for EKF2 tilt alignment.
    log.info("Waiting for EKF2 tilt alignment (ATTITUDE message)...")
    t1 = time.monotonic()
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        msg = conn.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
        if msg is not None:
            break
    else:
        raise TimeoutError('EKF2 did not achieve tilt alignment within 60 s')
    log.info("EKF2 tilt aligned after %.2fs", time.monotonic() - t1)

    t = GnssDeniedTester(conn)

    # Let GNSS-denied instances initialize (1s wall = 10s sim at speed factor 10).
    log.info("Waiting 1s for GNSS-denied instances to initialize...")
    time.sleep(1.0)

    log.info("Tester ready (fixture setup took %.2fs)", time.monotonic() - t0)

    yield t

    t.shutdown()
    conn.close()
