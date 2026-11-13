import os
import sys
from unittest.mock import MagicMock

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


class FakeMqtt:
    def __init__(self, base_topic="ha_idrac_controller/r720"):
        self.base_topic = base_topic
        self.availability_topic = f"{base_topic}/status"
        self.is_connected = True
        self.discoveries = []
        self.removals = []
        self.states = []
        self.published = []
        self.subscribed = []

    def subscribe(self, topic):
        self.subscribed.append(topic)

    def publish(self, topic, payload, retain=False, qos=0):
        self.published.append((topic, payload, retain))
        return True

    def publish_discovery(self, component, slug, name, *args, **kwargs):
        self.discoveries.append((component, slug, name, kwargs))

    def remove_discovery(self, component, slug):
        self.removals.append((component, slug))

    def publish_state(self, component, slug, state, attributes=None, retain=False):
        self.states.append((component, slug, state, attributes, retain))


def test_no_fan_status_discovery_and_fan_fault_has_status_attribute(tmp_path, monkeypatch):
    monkeypatch.setattr(watchdog, "HEARTBEAT_FILE", str(tmp_path / "hb.json"))
    monkeypatch.setattr(main, "PID_STATE_FILE", str(tmp_path / "pid_state.json"))

    worker = main.ServerWorker({
        "alias": "r720",
        "idrac_ip": "10.0.0.10",
        "idrac_username": "root",
        "idrac_password": "pw",
        "fan_mode": "simple",
        "fan_control_enabled": True,
    }, {"log_level": "warning", "check_interval_seconds": 30})
    fake_mqtt = FakeMqtt()
    worker.mqtt = fake_mqtt

    worker._fan_fault_status = {"faulty": {"Fan4": "stopped"}, "ok": False}
    worker._fan_fault_description = "Fan4: stopped"

    status_data = {
        "hottest_cpu_temp": 45,
        "inlet_temp": 22,
        "exhaust_temp": 30,
        "power": 150,
        "target_fan_speed": 30,
        "cpus": [45],
        "fans": [{"name": "Fan1", "rpm": 3600}],
        "psus": [],
        "fan_ok": False,
        "fan_status": "Fan4: stopped",
        "fan_readings": {"Fan1": 3600, "Fan4": 0},
    }
    worker._publish_mqtt_data(status_data)

    discovered_slugs = [d[1] for d in fake_mqtt.discoveries]
    assert "fan_status" not in discovered_slugs
    assert "fan_fault" in discovered_slugs

    fan_fault_states = [s for s in fake_mqtt.states if s[1] == "fan_fault"]
    assert len(fan_fault_states) == 1
    comp, slug, state, attrs, retain = fan_fault_states[0]
    assert comp == "binary_sensor"
    assert state == "ON"
    assert retain is True
    assert attrs["status"] == "Fan4: stopped"
    assert attrs["faulty_fans"] == ["Fan4"]
    assert attrs["Fan1"] == 3600
    assert "state" not in attrs


def test_startup_cleanup_removes_deprecated_entities(tmp_path, monkeypatch):
    monkeypatch.setattr(watchdog, "HEARTBEAT_FILE", str(tmp_path / "hb.json"))
    monkeypatch.setattr(main, "PID_STATE_FILE", str(tmp_path / "pid_state.json"))

    worker = main.ServerWorker({
        "alias": "r720",
        "idrac_ip": "10.0.0.10",
        "idrac_username": "root",
        "idrac_password": "pw",
    }, {"log_level": "warning", "check_interval_seconds": 30})
    fake_mqtt = FakeMqtt()
    worker.mqtt = fake_mqtt

    worker._on_mqtt_connected()

    assert ("sensor", "fan_status") in fake_mqtt.removals
    assert ("sensor", "disks_summary") in fake_mqtt.removals
    for slug in ("fan_status", "disks_summary"):
        assert (f"{fake_mqtt.base_topic}/sensor/{slug}", "", True) in fake_mqtt.published
        assert (f"{fake_mqtt.base_topic}/sensor/{slug}/attributes", "", True) in fake_mqtt.published
        assert (f"{fake_mqtt.base_topic}/sensor/{slug}/availability", "", True) in fake_mqtt.published

