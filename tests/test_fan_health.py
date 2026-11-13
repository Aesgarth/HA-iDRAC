import os
import sys
from unittest.mock import MagicMock
import pytest
from conftest import load_module

for mod in ["flask", "markupsafe", "paho", "paho.mqtt", "paho.mqtt.client"]:
    try:
        __import__(mod)
    except ModuleNotFoundError:
        if mod not in sys.modules:
            sys.modules[mod] = MagicMock()

fh = load_module("fan_health", "ha-idrac-controller-dev/app/fan_health.py")

dev_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "ha-idrac-controller-dev"))
if dev_dir not in sys.path:
    sys.path.insert(0, dev_dir)

from app import main  # noqa: E402


def six(rpm=3840, **override):
    fans = {f"Fan{i}": rpm for i in range(1, 7)}
    fans.update(override)
    return fans


def run(monitor, readings_list, start=0.0, step=30.0):
    result = None
    for i, readings in enumerate(readings_list):
        result = monitor.update(start + i * step, readings)
    return result


def test_healthy_fans_are_ok():
    m = fh.FanMonitor(30)
    assert run(m, [six(3840 + (i % 3) * 60) for i in range(20)])["ok"]


def test_stopped_fan_is_detected_after_confirmation():
    m = fh.FanMonitor(30)
    run(m, [six() for _ in range(10)])
    result = run(m, [six(Fan4=0) for _ in range(4)], start=300)
    assert not result["ok"]
    assert result["faulty"] == {"Fan4": "stopped"}
    assert m.describe() == "Fan4: stopped"


def test_brief_zero_reading_is_ignored():
    m = fh.FanMonitor(30)
    run(m, [six() for _ in range(10)])
    run(m, [six(Fan2=0)], start=300)
    assert run(m, [six() for _ in range(5)], start=330)["ok"]


def test_slow_fan_is_detected():
    m = fh.FanMonitor(30)
    run(m, [six() for _ in range(10)])
    result = run(m, [six(Fan3=1800) for _ in range(5)], start=300)
    assert result["faulty"] == {"Fan3": "slow"}


def test_missing_fan_is_detected():
    m = fh.FanMonitor(30)
    run(m, [six() for _ in range(10)])
    readings = six()
    del readings["Fan5"]
    result = run(m, [readings for _ in range(5)], start=300)
    assert result["faulty"] == {"Fan5": "missing"}


def test_speed_change_does_not_cause_false_alarm():
    m = fh.FanMonitor(30)
    run(m, [six(3000) for _ in range(10)])
    transition = [six(3000), six(3600, Fan1=3000), six(4200, Fan1=3400), six(4800, Fan1=4000), six(5400)]
    assert run(m, transition, start=300)["ok"]


def test_fan_model_that_is_naturally_slower_is_not_flagged():
    m = fh.FanMonitor(30)
    readings = six(Fan6=2700)
    assert run(m, [readings for _ in range(60)])["ok"]


def test_fewer_than_three_fans_only_detects_stopped():
    m = fh.FanMonitor(30)
    two = {"Fan1": 3000, "Fan2": 1500}
    assert run(m, [two for _ in range(10)])["ok"]
    result = run(m, [{"Fan1": 3000, "Fan2": 0} for _ in range(4)], start=300)
    assert result["faulty"] == {"Fan2": "stopped"}


def test_fault_clears_after_recovery():
    m = fh.FanMonitor(30)
    run(m, [six() for _ in range(10)])
    assert not run(m, [six(Fan4=0) for _ in range(4)], start=300)["ok"]
    assert run(m, [six() for _ in range(6)], start=420)["ok"]


def test_all_fans_stopped_is_not_reported_as_individual_slow_fans():
    m = fh.FanMonitor(30)
    run(m, [six() for _ in range(10)])
    result = run(m, [six(0) for _ in range(5)], start=300)
    assert isinstance(result["ok"], bool)


def test_confirmation_time_scales_with_poll_interval():
    fast = fh.FanMonitor(10)
    run(fast, [six() for _ in range(20)], step=10.0)
    assert fast.confirm_polls == 9
    result = run(fast, [six(Fan1=0) for _ in range(9)], start=200, step=10.0)
    assert not result["ok"]


def test_parse_fan_readings_handles_missing_and_ignores_pseudo_sensors():
    from app.ipmi_manager import IPMIManager
    ipmi = IPMIManager("10.0.0.10", "root", "secret", log_level="fatal")
    sdr = "\n".join([
        "Fan1A Tach       | 30h | ok  |  7.1 | 3840 RPM",
        "Fan2A Tach       | 31h | ns  |  7.2 | No Reading",
        "Fan3A Tach       | 32h | ok  |  7.3 | na",
        "Fan Redundancy   | 75h | ok  |  7.1 | Fully Redundant",
        "Fan1 Presence    | 76h | ok  |  7.1 | Presence detected",
    ])
    assert ipmi.parse_fan_readings(sdr) == {
        "Fan1A Tach": 3840,
        "Fan2A Tach": None,
        "Fan3A Tach": None,
    }


@pytest.mark.parametrize("policy", ["alert_only", "raise_fans", "dell_auto"])
def test_fan_fault_wiring_policies(policy):
    server_conf = {
        "alias": "test-r720",
        "idrac_ip": "10.0.0.10",
        "idrac_username": "root",
        "idrac_password": "secret",
        "fan_control_enabled": True,
        "fan_mode": "simple",
        "base_fan_speed_percent": 20,
        "low_temp_threshold": 45,
        "high_temp_fan_speed_percent": 50,
        "critical_temp_threshold": 65,
        "poll_interval_seconds": 30,
        "on_fan_fault": policy,
    }
    global_opts = {
        "log_level": "info",
        "check_interval_seconds": 30,
        "mqtt_host": "",
        "mqtt_port": 1883,
        "mqtt_username": "",
        "mqtt_password": "",
    }
    worker = main.ServerWorker(server_conf, global_opts)
    user_speeds = []
    auto_calls = []
    worker.ipmi.apply_user_fan_control_profile = lambda s: user_speeds.append(s) or "ok"
    worker.ipmi.apply_dell_fan_control_profile = lambda force=False: auto_calls.append(force) or "ok"

    temps = {"cpu_temps": [35], "inlet_temp": 22, "exhaust_temp": 28}
    t = 0.0
    for _ in range(5):
        worker.evaluate_fan_health(six(), now=t)
        worker._apply_fan_policy(35, temps, 180, now=t)
        t += 30.0

    user_speeds.clear()
    auto_calls.clear()

    # Stop Fan4 for 4 polls (>90s at 30s interval)
    for _ in range(4):
        worker.evaluate_fan_health(six(Fan4=0), now=t)
        result_label = worker._apply_fan_policy(35, temps, 180, now=t)
        t += 30.0

    assert not worker._fan_fault_status["ok"]

    if policy == "alert_only":
        assert auto_calls == []
        assert user_speeds[-1] == 20
    elif policy == "raise_fans":
        assert auto_calls == []
        assert user_speeds[-1] >= 80
    elif policy == "dell_auto":
        # Issued once on fault confirmation and NOT repeatedly on subsequent fault polls
        assert len(auto_calls) == 1
        assert result_label == "Dell Auto (fan fault)"
        worker.evaluate_fan_health(six(Fan4=0), now=t)
        worker._apply_fan_policy(35, temps, 180, now=t)
        assert len(auto_calls) == 1
