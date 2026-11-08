"""Tests for concurrent PID state file handling and atomic writes."""
import json
import os
import sys
import threading
from unittest.mock import MagicMock

# Mock optional dependencies so tests run in plain environments without Flask or Paho
for mod in ["flask", "markupsafe", "paho", "paho.mqtt", "paho.mqtt.client"]:
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
                pid_state={"integral": float(idx), "last_error": float(idx * 2)},
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
        assert data[alias]["integral"] == float(i)
        assert data[alias]["last_error"] == float(i * 2)

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
        w.pid.integral = float(i * 5)
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
        assert data[alias]["integral"] == float(i * 5)

