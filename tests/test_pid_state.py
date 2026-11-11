"""Tests for concurrent PID state file handling and atomic writes."""
import json
import os
import sys
import threading
from unittest.mock import MagicMock

# Mock optional dependencies so tests run in plain environments without Flask or Paho
for mod in ["flask", "markupsafe", "paho", "paho.mqtt", "paho.mqtt.client"]:
    try:
        __import__(mod)
    except ModuleNotFoundError:
        if mod not in sys.modules:
            sys.modules[mod] = MagicMock()

dev_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "ha-idrac-controller-dev"))
if dev_dir not in sys.path:
    sys.path.insert(0, dev_dir)

from app import main


def test_concurrent_pid_state_saves(tmp_path):
    state_file = str(tmp_path / "pid_states.json")
    aliases = [f"server_{i}" for i in range(15)]
    errors = []

    def worker_save(alias, idx):
        try:
            main.save_pid_state(
                alias=alias,
                pid_state={
                    "theta": [0.05 + idx * 0.001, 0.008],
                    "cov": [[1.0, 0.0], [0.0, 1.0]],
                    "samples": idx,
                    "trim": 0.5,
                    "last_output": 25.0,
                },
                filepath=state_file,
            )
        except Exception as e:
            errors.append(e)

    threads = [
        threading.Thread(target=worker_save, args=(alias, i))
        for i, alias in enumerate(aliases)
    ]

    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert not errors, f"Errors occurred during concurrent saves: {errors}"
    assert os.path.exists(state_file), "PID state file was not created"

    with open(state_file, "r") as f:
        data = json.load(f)

    assert isinstance(data, dict), "PID state file content is not a JSON object"
    for i, alias in enumerate(aliases):
        assert alias in data, f"Alias {alias} missing from PID states"
        assert data[alias]["samples"] == i
        assert data[alias]["theta"] == [0.05 + i * 0.001, 0.008]
        assert data[alias]["cov"] == [[1.0, 0.0], [0.0, 1.0]]

    temp_files = [f for f in os.listdir(tmp_path) if f.startswith("pid_states.json.tmp")]
    assert not temp_files, f"Found leftover temp files: {temp_files}"


def test_concurrent_worker_cleanup_saves(tmp_path, monkeypatch):
    state_file = str(tmp_path / "pid_states.json")
    monkeypatch.setattr(main, "PID_STATE_FILE", state_file)

    global_opts = {
        "log_level": "info",
        "mqtt_host": "",
        "mqtt_port": 1883,
        "mqtt_username": "",
        "mqtt_password": "",
    }

    workers = []
    aliases = [f"worker_{i}" for i in range(10)]
    for i, alias in enumerate(aliases):
        config = {
            "alias": alias,
            "idrac_ip": "127.0.0.1",
            "idrac_username": "root",
            "idrac_password": "password",
            "fan_control_enabled": False,
        }
        w = main.ServerWorker(config, global_opts)
        w.pid.samples = i * 5
        w.pid.theta = [0.05 + i * 0.01, 0.008]
        workers.append(w)

    threads = [threading.Thread(target=w.cleanup) for w in workers]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    with open(state_file, "r") as f:
        data = json.load(f)

    for i, alias in enumerate(aliases):
        assert alias in data
        assert data[alias]["samples"] == i * 5
        assert data[alias]["theta"] == [0.05 + i * 0.01, 0.008]


def test_pid_state_roundtrip_and_delete(tmp_path):
    from app.adaptive_pid import AdaptiveFanController

    state_file = str(tmp_path / "pid_states.json")
    ctl = AdaptiveFanController(setpoint=50)
    ctl.theta = [0.062, 0.0075]
    ctl.cov = [[0.8, 0.1], [0.1, 0.9]]
    ctl.samples = 12
    ctl.trim = 1.25

    main.save_pid_state("srv1", ctl.get_state(), filepath=state_file)
    main.save_pid_state("srv2", {"samples": 3, "theta": [0.05, 0.008], "cov": [[1.0, 0.0], [0.0, 1.0]], "trim": 0.0, "last_output": 20.0}, filepath=state_file)

    ctl2 = AdaptiveFanController(setpoint=50)
    with open(state_file, "r") as f:
        data = json.load(f)
    ctl2.load_state(data["srv1"])

    assert ctl2.samples == 12
    assert ctl2.theta == [0.062, 0.0075]
    assert ctl2.cov == [[0.8, 0.1], [0.1, 0.9]]
    assert ctl2.trim == 1.25

    # Test delete_pid_state removes srv1 and leaves srv2
    main.delete_pid_state("srv1", filepath=state_file)
    with open(state_file, "r") as f:
        data_after = json.load(f)

    assert "srv1" not in data_after
    assert "srv2" in data_after


def test_server_worker_disk_poller_lifecycle(monkeypatch):
    server_conf = {
        "alias": "srv_snmp",
        "idrac_ip": "192.168.1.200",
        "idrac_username": "root",
        "idrac_password": "calvin",
        "snmp_enabled": True,
        "snmp_community": "public",
        "snmp_port": 161,
        "disk_poll_minutes": 10,
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
    monkeypatch.setattr(worker.ipmi, "get_server_model_info", lambda: {"model": "R720"})
    monkeypatch.setattr(worker.ipmi, "get_firmware_version", lambda: "2.41")
    monkeypatch.setattr(worker.ipmi, "apply_dell_fan_control_profile", lambda: None)
    monkeypatch.setattr(main, "save_pid_state", lambda *a, **k: None)

    started = []
    stopped = []
    monkeypatch.setattr(main.DiskHealthPoller, "start", lambda self: started.append(True))
    monkeypatch.setattr(main.DiskHealthPoller, "stop", lambda self: stopped.append(True))
    monkeypatch.setattr(main.DiskHealthPoller, "join", lambda self, timeout=None: None)

    worker._initialize()
    assert worker.disk_poller is not None
    assert len(started) == 1

    worker.stop()
    assert len(stopped) == 1

    worker.cleanup()


