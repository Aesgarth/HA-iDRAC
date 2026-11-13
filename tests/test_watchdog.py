import json
import os
import sys
import types
from unittest.mock import MagicMock

import pytest

for mod in ["flask", "markupsafe", "paho", "paho.mqtt", "paho.mqtt.client"]:
    try:
        __import__(mod)
    except ModuleNotFoundError:
        if mod not in sys.modules:
            sys.modules[mod] = MagicMock()

dev_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "ha-idrac-controller-dev"))
if dev_dir not in sys.path:
    sys.path.insert(0, dev_dir)

from app import main, watchdog  # noqa: E402


# ---------------------------------------------------------------------------
# Watchdog unit tests
# ---------------------------------------------------------------------------

def _sample_servers():
    return [
        {"id": "r720", "host": "10.0.0.10", "user": "root", "password": "secret-pw"},
    ]


def test_fresh_heartbeat_not_stale(tmp_path):
    hb = str(tmp_path / "hb.json")
    watchdog.write_heartbeat(_sample_servers(), path=hb, now=1000.0)
    assert watchdog.stale_servers(path=hb, now=1100.0) == []


def test_stale_heartbeat_returns_servers(tmp_path):
    hb = str(tmp_path / "hb.json")
    servers = _sample_servers()
    watchdog.write_heartbeat(servers, path=hb, now=1000.0)
    assert watchdog.stale_servers(path=hb, now=1200.0) == servers


def test_empty_servers_list_returns_empty_even_when_stale(tmp_path):
    hb = str(tmp_path / "hb.json")
    watchdog.write_heartbeat([], path=hb, now=1000.0)
    assert watchdog.stale_servers(path=hb, now=2000.0) == []


def test_missing_or_corrupt_heartbeat_returns_empty(tmp_path):
    missing = str(tmp_path / "does_not_exist.json")
    assert watchdog.stale_servers(path=missing, now=1000.0) == []

    corrupt = tmp_path / "corrupt.json"
    corrupt.write_text("{not valid json")
    assert watchdog.stale_servers(path=str(corrupt), now=1000.0) == []


def test_hand_back_passes_password_in_env_not_argv():
    calls = []

    def fake_runner(cmd, env=None, capture_output=True, timeout=20):
        calls.append((cmd, env))
        return types.SimpleNamespace(returncode=0)

    server = {"id": "r720", "host": "10.0.0.10", "user": "root", "password": "secret-pw"}
    assert watchdog.hand_back(server, runner=fake_runner) is True
    assert len(calls) == 1
    cmd, env = calls[0]
    assert cmd[-5:] == watchdog.AUTO_ARGS
    assert "secret-pw" not in cmd
    assert env["IPMI_PASSWORD"] == "secret-pw"

    def fail_runner(cmd, env=None, capture_output=True, timeout=20):
        return types.SimpleNamespace(returncode=1)

    assert watchdog.hand_back(server, runner=fail_runner) is False


def test_run_forever_once_clears_only_when_all_succeed(tmp_path):
    hb = str(tmp_path / "hb.json")
    servers = [
        {"id": "s1", "host": "10.0.0.1", "user": "u", "password": "p1"},
        {"id": "s2", "host": "10.0.0.2", "user": "u", "password": "p2"},
    ]
    watchdog.write_heartbeat(servers, path=hb, now=0.0)

    # One server fails -> do not clear heartbeat
    def partial_fail(cmd, env=None, capture_output=True, timeout=20):
        rc = 0 if "10.0.0.1" in cmd else 1
        return types.SimpleNamespace(returncode=rc)

    watchdog.run_forever(path=hb, runner=partial_fail, once=True)
    with open(hb) as f:
        assert len(json.load(f)["servers"]) == 2

    # All succeed -> clear heartbeat
    def all_ok(cmd, env=None, capture_output=True, timeout=20):
        return types.SimpleNamespace(returncode=0)

    watchdog.run_forever(path=hb, runner=all_ok, once=True)
    with open(hb) as f:
        assert json.load(f)["servers"] == []


# ---------------------------------------------------------------------------
# Integration-style test: main stops updating heartbeat, time advances >180s
# ---------------------------------------------------------------------------

def test_watchdog_integration_retries_once_per_check_until_success(tmp_path, monkeypatch):
    hb = str(tmp_path / "hb.json")
    now = [1000.0]
    monkeypatch.setattr(watchdog.time, "time", lambda: now[0])

    worker = types.SimpleNamespace(
        alias="r720",
        in_manual_control=True,
        ipmi=types.SimpleNamespace(ip="10.0.0.10", user="root", password="pw"),
    )
    monkeypatch.setattr(watchdog, "HEARTBEAT_FILE", hb)
    main.update_watchdog_heartbeat(workers=[worker], now=now[0])

    # Main stops updating; advance time > 180 s
    now[0] += 185.0
    attempts = []

    def fail_first_then_succeed(cmd, env=None, capture_output=True, timeout=20):
        attempts.append(list(cmd))
        rc = 1 if len(attempts) == 1 else 0
        return types.SimpleNamespace(returncode=rc)

    # First check fails: attempted once, heartbeat still present
    watchdog.run_forever(path=hb, runner=fail_first_then_succeed, once=True)
    assert len(attempts) == 1
    assert len(watchdog.stale_servers(path=hb, now=now[0])) == 1

    # Second check succeeds: attempted once more, heartbeat cleared
    watchdog.run_forever(path=hb, runner=fail_first_then_succeed, once=True)
    assert len(attempts) == 2
    assert watchdog.stale_servers(path=hb, now=now[0]) == []


# ---------------------------------------------------------------------------
# Layer 1 tests with fake ipmitool (subprocess.run)
# ---------------------------------------------------------------------------

_GOOD_TEMP_SDR = "\n".join([
    "Inlet Temp       | 04h | ok  |  7.1 | 22 degrees C",
    "Exhaust Temp     | 01h | ok  |  7.1 | 30 degrees C",
    "CPU1 Temp        | 0Eh | ok  |  3.1 | 40 degrees C",
])
_GOOD_FAN_SDR = "Fan1A Tach       | 30h | ok  |  7.1 | 3600 RPM"
_GOOD_PWR_SDR = "Pwr Consumption  | 77h | ok  | 21.1 | 120 Watts"


def _make_worker(tmp_path, monkeypatch, extra_conf=None):
    hb = str(tmp_path / "hb.json")
    monkeypatch.setattr(watchdog, "HEARTBEAT_FILE", hb)
    monkeypatch.setattr(main, "PID_STATE_FILE", str(tmp_path / "pid_state.json"))

    conf = {
        "alias": "test-r720",
        "idrac_ip": "10.0.0.50",
        "idrac_username": "root",
        "idrac_password": "pw",
        "fan_mode": "target",
        "fan_control_enabled": True,
        "base_fan_speed_percent": 20,
        "low_temp_threshold": 45,
        "high_temp_fan_speed_percent": 50,
        "critical_temp_threshold": 80,
        "pid_config": {"target_temp": 40, "min_speed": 15, "max_speed": 90},
    }
    if extra_conf:
        conf.update(extra_conf)

    worker = main.ServerWorker(conf, {"log_level": "warning", "check_interval_seconds": 10})
    worker.mqtt = MagicMock()
    worker.mqtt.is_connected = True
    worker.mqtt.availability_topic = "idrac/test-r720/availability"
    monkeypatch.setattr(main, "worker_instances", [worker])
    return worker, hb


def test_layer1_three_failed_reads_and_reconnect_with_fake_ipmitool(tmp_path, monkeypatch):
    worker, _ = _make_worker(tmp_path, monkeypatch)

    temp_should_fail = [False]
    recorded_raw_cmds = []

    def fake_subprocess_run(cmd, env=None, capture_output=True, text=True, timeout=15, **kwargs):
        cmd_lower = [c.lower() for c in cmd]
        if "sdr" in cmd_lower:
            if "temperature" in cmd_lower:
                if temp_should_fail[0]:
                    return types.SimpleNamespace(returncode=1, stdout="", stderr="Error: timeout")
                return types.SimpleNamespace(returncode=0, stdout=_GOOD_TEMP_SDR, stderr="")
            if "fan" in cmd_lower:
                return types.SimpleNamespace(returncode=0, stdout=_GOOD_FAN_SDR, stderr="")
            return types.SimpleNamespace(returncode=0, stdout=_GOOD_PWR_SDR, stderr="")
        if "raw" in cmd:
            idx = cmd.index("raw")
            recorded_raw_cmds.append(cmd[idx:])
            return types.SimpleNamespace(returncode=0, stdout="", stderr="")
        return types.SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr("app.ipmi_manager.subprocess.run", fake_subprocess_run)

    # Normal cycle puts server in manual control and advances pid state
    assert worker.step_once() is None
    assert worker.in_manual_control is True
    worker.pid.speed = 42.0
    worker.pid.last_time = 12345.0
    worker.fan_state.in_auto = True
    worker.fan_state.bucket = "high"
    recorded_raw_cmds.clear()

    # 3 consecutive failed temperature reads -> Dell auto every 60 s + idrac_reachable OFF
    temp_should_fail[0] = True
    assert worker.step_once() == 60
    assert worker._consecutive_read_failures == 1
    assert recorded_raw_cmds == []

    assert worker.step_once() == 60
    assert worker._consecutive_read_failures == 2
    assert recorded_raw_cmds == []

    assert worker.step_once() == 60
    assert worker._consecutive_read_failures == 3
    assert worker._idrac_unreachable is True
    assert ["raw", "0x30", "0x30", "0x01", "0x01"] in recorded_raw_cmds
    assert worker.in_manual_control is False
    worker.mqtt.publish_state.assert_any_call("binary_sensor", "idrac_reachable", "OFF", retain=True)
    # Availability topic stays online so idrac_reachable is OFF rather than unavailable
    worker.mqtt.publish.assert_any_call(worker.mqtt.availability_topic, "online", retain=True)

    # 4th failed read keeps trying Dell auto every 60 s
    recorded_raw_cmds.clear()
    assert worker.step_once() == 60
    assert ["raw", "0x30", "0x30", "0x01", "0x01"] in recorded_raw_cmds

    # Reconnect: Dell auto must be sent FIRST, slew state reset, idrac_reachable ON
    recorded_raw_cmds.clear()
    temp_should_fail[0] = False
    assert worker.step_once() is None
    assert recorded_raw_cmds[0] == ["raw", "0x30", "0x30", "0x01", "0x01"]
    assert worker._idrac_unreachable is False
    assert worker.fan_state.in_auto is False
    assert worker.fan_state.bucket is None
    worker.mqtt.publish_state.assert_any_call("binary_sensor", "idrac_reachable", "ON", retain=True)


def test_min_safe_fan_speed_floor_enforced(tmp_path, monkeypatch):
    worker, _ = _make_worker(tmp_path, monkeypatch, extra_conf={
        "fan_mode": "simple",
        "base_fan_speed_percent": 10,
        "min_safe_fan_speed": 25,
    })
    applied = []
    monkeypatch.setattr(worker.ipmi, "apply_user_fan_control_profile", lambda spd: applied.append(spd) or spd)

    speed = worker._apply_fan_policy(30, {"inlet_temp": 20, "cpu_temps": [30]}, 100)
    assert speed == 25
    assert applied == [25]
    assert worker.in_manual_control is True
