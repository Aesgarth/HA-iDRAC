import copy
import json
import os
import sys
import threading
import time
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


def _base_server_conf(alias="r720"):
    return {
        "alias": alias,
        "idrac_ip": "10.0.0.50",
        "idrac_username": "root",
        "idrac_password": "pw1",
        "enabled": True,
        "fan_control_enabled": True,
        "disable_pcie_cooling_response": False,
        "on_fan_fault": "dell_auto",
        "min_safe_fan_speed": 20,
        "snmp_enabled": False,
        "snmp_community": "",
        "snmp_port": 161,
        "disk_poll_minutes": 10,
        "fan_mode": "target",
        "base_fan_speed_percent": 20,
        "low_temp_threshold": 45,
        "high_temp_fan_speed_percent": 50,
        "critical_temp_threshold": 80,
        "poll_interval_seconds": 30,
        "pid_config": {
            "target_temp": 55,
            "min_speed": 20,
            "max_speed": 80,
            "deadband": 0.75,
            "ti_seconds": 300.0,
            "ramp_down_percent_per_min": 4,
        },
        "fan_curve": [],
    }


def _make_manager(tmp_path, monkeypatch, initial_configs=None):
    cfg_file = str(tmp_path / "servers_config.json")
    with open(cfg_file, "w") as f:
        json.dump(initial_configs or [], f)
    monkeypatch.setattr(watchdog, "HEARTBEAT_FILE", str(tmp_path / "hb.json"))
    monkeypatch.setattr(main, "PID_STATE_FILE", str(tmp_path / "pid_state.json"))
    workers = []
    threads = []
    mgr = main.ServerManager(
        {"log_level": "warning", "check_interval_seconds": 30},
        config_file=cfg_file,
        workers=workers,
        threads_list=threads,
    )
    return mgr, cfg_file


def test_apply_settings_updates_pid_and_poll_interval_preserving_learned_state(tmp_path, monkeypatch):
    conf = _base_server_conf()
    mgr, cfg_file = _make_manager(tmp_path, monkeypatch, [conf])
    worker = main.ServerWorker(copy.deepcopy(conf), mgr.global_opts)
    mgr.workers.append(worker)

    # Populate learned state
    worker.pid.theta = [0.123, 0.045]
    worker.pid.power_floor = {2: (110.0, 25), 6: (160.0, 25)}
    worker.pid.samples = 12
    before_theta = list(worker.pid.theta)
    before_floor = dict(worker.pid.power_floor)

    # Baseline output at temp=50, setpoint=55
    s_before = worker.pid.update(50.0, 22.0, 150.0, now=100.0)
    assert s_before == 20  # at min_speed
    state_before_apply = worker.pid.get_state()

    # Change setpoint down to 35, raise min_speed to 30, max_speed to 90, ramp-down to 12, critical to 75, poll to 10
    new_conf = copy.deepcopy(conf)
    new_conf["critical_temp_threshold"] = 75
    new_conf["poll_interval_seconds"] = 10
    new_conf["pid_config"]["target_temp"] = 35
    new_conf["pid_config"]["min_speed"] = 30
    new_conf["pid_config"]["max_speed"] = 90
    new_conf["pid_config"]["ramp_down_percent_per_min"] = 12

    mgr.apply_settings("r720", new_conf)
    assert worker.pid.get_state() == state_before_apply

    # Output changes on next update while learned state is preserved
    s_after = worker.pid.update(50.0, 22.0, 150.0, now=110.0)
    assert s_after > s_before
    assert worker.pid.theta == before_theta
    assert worker.pid.critical == 75.0
    assert worker.pid.slew_down_per_s == pytest.approx(12.0 / 60.0)
    assert worker.fan_monitor.poll_seconds == 10.0

    with open(cfg_file) as f:
        saved = json.load(f)
    assert saved[0]["pid_config"]["target_temp"] == 35


def test_add_and_remove_server_lifecycle(tmp_path, monkeypatch):
    mgr, cfg_file = _make_manager(tmp_path, monkeypatch, [])
    auto_calls = []

    def fake_run(self):
        self.ipmi.apply_dell_fan_control_profile = lambda force=False: auto_calls.append(self.alias) or ""
        self.mqtt = MagicMock()
        self.mqtt.is_connected = True
        self.discovered_sensors = {"hottest_cpu_temp"}
        self.discovered_components = {"hottest_cpu_temp": "sensor"}
        while self.running:
            time.sleep(0.02)
        self.cleanup()

    monkeypatch.setattr(main.ServerWorker, "run", fake_run)

    conf = _base_server_conf("r730")
    mgr.add_server(conf)
    assert len(mgr.workers) == 1
    assert len(mgr.threads) == 1
    thread = mgr.threads[0]
    time.sleep(0.05)
    assert thread.is_alive()

    mgr.remove_server("r730")
    assert len(mgr.workers) == 0
    assert len(mgr.threads) == 0
    assert not thread.is_alive()
    assert "r730" in auto_calls

    with open(cfg_file) as f:
        assert json.load(f) == []


def test_update_connection_hands_back_restarts_and_keeps_learned_state(tmp_path, monkeypatch):
    conf = _base_server_conf("r720")
    mgr, cfg_file = _make_manager(tmp_path, monkeypatch, [conf])
    auto_calls = []

    def fake_run(self):
        self.ipmi.apply_dell_fan_control_profile = lambda force=False: auto_calls.append(self.ipmi.ip) or ""
        self.mqtt = MagicMock()
        self.mqtt.is_connected = False
        while self.running:
            time.sleep(0.02)
        self.cleanup()

    monkeypatch.setattr(main.ServerWorker, "run", fake_run)

    mgr.start_all([conf])
    old_worker = mgr.get_worker("r720")
    old_worker.pid.theta = [0.111, 0.033]
    old_worker.pid.power_floor = {3: (130.0, 22)}
    old_thread = mgr.threads[0]
    time.sleep(0.05)

    mgr.update_connection("r720", host="10.0.0.99", user="admin", password="newpw")
    assert not old_thread.is_alive()
    assert "10.0.0.50" in auto_calls

    new_worker = mgr.get_worker("r720")
    assert new_worker is not None
    assert new_worker is not old_worker
    assert new_worker.ipmi.ip == "10.0.0.99"
    assert new_worker.ipmi.user == "admin"
    assert new_worker.ipmi.password == "newpw"
    assert new_worker.pid.theta == [0.111, 0.033]
    assert new_worker.pid.power_floor == {3: (130.0, 22)}

    mgr.remove_server("r720")


def test_invalid_settings_rejected_without_mutating_worker_or_disk(tmp_path, monkeypatch):
    conf = _base_server_conf("r720")
    mgr, cfg_file = _make_manager(tmp_path, monkeypatch, [conf])
    worker = main.ServerWorker(copy.deepcopy(conf), mgr.global_opts)
    mgr.workers.append(worker)

    bad_conf = copy.deepcopy(conf)
    bad_conf["pid_config"]["min_speed"] = 90
    bad_conf["pid_config"]["max_speed"] = 40

    with pytest.raises(ValueError, match="minimum fan speed cannot exceed maximum"):
        mgr.apply_settings("r720", bad_conf)

    assert worker.pid.min_speed == 20.0
    assert worker.pid.max_speed == 80.0
    with open(cfg_file) as f:
        saved = json.load(f)
    assert saved[0]["pid_config"]["min_speed"] == 20
    assert saved[0]["pid_config"]["max_speed"] == 80
