import os
import sys
from unittest.mock import MagicMock

for mod in ["flask", "markupsafe", "paho", "paho.mqtt", "paho.mqtt.client"]:
    try:
        __import__(mod)
    except ModuleNotFoundError:
        if mod not in sys.modules:
            sys.modules[mod] = MagicMock()

import conftest  # noqa: F401
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


def test_mqtt_client_id_comes_from_addon_id_env(tmp_path, monkeypatch):
    from conftest import load_module
    mqtt_mod = load_module("mqtt_client_env_test", "ha-idrac-controller-dev/app/mqtt_client.py")

    monkeypatch.delenv("ADDON_ID", raising=False)
    c_default = mqtt_mod.MqttClient()
    assert c_default.client_id == "ha_idrac_controller"

    monkeypatch.setenv("ADDON_ID", "ha_idrac_standard")
    c_std = mqtt_mod.MqttClient()
    assert c_std.client_id == "ha_idrac_standard"
    c_std.set_device_info("r720", "DELL", "PowerEdge R720", "10.0.0.10")
    assert c_std.base_topic == "ha_idrac_controller/r720"
    assert c_std.device_info_dict["identifiers"] == ["idrac_controller_r720"]

    monkeypatch.setattr(watchdog, "HEARTBEAT_FILE", str(tmp_path / "hb.json"))
    monkeypatch.setattr(main, "PID_STATE_FILE", str(tmp_path / "pid_state.json"))
    worker = main.ServerWorker({
        "alias": "r720",
        "idrac_ip": "10.0.0.10",
        "idrac_username": "root",
        "idrac_password": "pw",
    }, {"log_level": "warning", "check_interval_seconds": 30})
    assert worker.mqtt.client_id == "ha_idrac_standard_r720"


def test_psu_electrical_mqtt_publish_and_disappear_goes_offline(tmp_path, monkeypatch):
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

    worker._publish_psu_electrical_mqtt({
        "PSU1": {"voltage": 242.0, "current": 0.8},
        "PSU2": {"voltage": 244.0, "current": 0.2},
    })
    disc_slugs = [d[1] for d in fake_mqtt.discoveries]
    assert "psu1_voltage" in disc_slugs
    assert "psu1_current" in disc_slugs
    assert "psu2_voltage" in disc_slugs
    assert "psu2_current" in disc_slugs
    assert ("sensor", "psu1_voltage", 242.0, None, False) in fake_mqtt.states
    assert ("sensor", "psu1_current", 0.8, None, False) in fake_mqtt.states

    # PSU2 disappears -> availability goes offline
    fake_mqtt.published.clear()
    worker._publish_psu_electrical_mqtt({"PSU1": {"voltage": 241.0, "current": 0.75}})
    assert (f"{fake_mqtt.base_topic}/sensor/psu2_voltage/availability", "offline", True) in fake_mqtt.published
    assert (f"{fake_mqtt.base_topic}/sensor/psu2_current/availability", "offline", True) in fake_mqtt.published
    assert (f"{fake_mqtt.base_topic}/sensor/psu1_voltage/availability", "online", True) in fake_mqtt.published


def test_redundancy_mqtt_publish_and_disappear_goes_offline(tmp_path, monkeypatch):
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

    worker._publish_redundancy_mqtt({
        "ps": {"reading": "Redundancy Lost", "ok": False},
        "fan": {"reading": "Fully Redundant", "ok": True},
    })
    disc_slugs = [d[1] for d in fake_mqtt.discoveries]
    assert "ps_redundancy" in disc_slugs
    assert "fan_redundancy" in disc_slugs
    assert ("binary_sensor", "ps_redundancy", "ON", {"reading": "Redundancy Lost"}, False) in fake_mqtt.states
    assert ("binary_sensor", "fan_redundancy", "OFF", {"reading": "Fully Redundant"}, False) in fake_mqtt.states

    fake_mqtt.published.clear()
    worker._publish_redundancy_mqtt({"ps": None, "fan": {"reading": "Fully Redundant", "ok": True}})
    assert (f"{fake_mqtt.base_topic}/binary_sensor/ps_redundancy/availability", "offline", True) in fake_mqtt.published
    assert (f"{fake_mqtt.base_topic}/binary_sensor/fan_redundancy/availability", "online", True) in fake_mqtt.published


def test_last_successful_poll_sensor_rate_limit_and_failure_recovery(tmp_path, monkeypatch):
    monkeypatch.setattr(watchdog, "HEARTBEAT_FILE", str(tmp_path / "hb.json"))
    monkeypatch.setattr(main, "PID_STATE_FILE", str(tmp_path / "pid_state.json"))

    mono_now = [1000.0]
    monkeypatch.setattr(main.time, "monotonic", lambda: mono_now[0])

    worker = main.ServerWorker({
        "alias": "r720",
        "idrac_ip": "10.0.0.10",
        "idrac_username": "root",
        "idrac_password": "pw",
        "fan_mode": "simple",
        "fan_control_enabled": False,
        "poll_interval_seconds": 5,
    }, {"log_level": "warning", "check_interval_seconds": 5})
    fake_mqtt = FakeMqtt()
    worker.mqtt = fake_mqtt

    temp_fail = [False]
    monkeypatch.setattr(
        worker.ipmi,
        "retrieve_temperatures_raw",
        lambda: None if temp_fail[0] else "CPU1 Temp | 01h | ok | 3.1 | 40 degrees C",
    )
    monkeypatch.setattr(worker.ipmi, "retrieve_fan_rpms_raw", lambda: "Fan1A Tach | 30h | ok | 7.1 | 3000 RPM")
    monkeypatch.setattr(worker.ipmi, "retrieve_power_sdr_raw", lambda: "")

    # 1. First successful read: discovers with timestamp/diagnostic and publishes with retain=True
    assert worker.step_once() is None
    disc = [d for d in fake_mqtt.discoveries if d[1] == "last_successful_poll"]
    assert len(disc) == 1
    assert disc[0][0] == "sensor"
    assert disc[0][3]["entity_category"] == "diagnostic"
    poll_states = [s for s in fake_mqtt.states if s[1] == "last_successful_poll"]
    assert len(poll_states) == 1
    assert poll_states[0][4] is True
    assert poll_states[0][2].endswith("+00:00")
    first_iso = worker._last_successful_poll_iso

    # 2. Poll 5s later (< 10s rate limit): _last_successful_poll_iso updates, MQTT publish is rate-limited
    mono_now[0] += 5.0
    fake_mqtt.states.clear()
    assert worker.step_once() is None
    assert [s for s in fake_mqtt.states if s[1] == "last_successful_poll"] == []

    # 3. Poll 10s after first publish: MQTT publish occurs
    mono_now[0] += 6.0
    fake_mqtt.states.clear()
    assert worker.step_once() is None
    assert len([s for s in fake_mqtt.states if s[1] == "last_successful_poll"]) == 1

    # 4. Failed read: does NOT update timestamp and does NOT mark entity unavailable
    saved_iso = worker._last_successful_poll_iso
    temp_fail[0] = True
    mono_now[0] += 5.0
    fake_mqtt.states.clear()
    fake_mqtt.published.clear()
    assert worker.step_once() == 60
    assert worker._last_successful_poll_iso == saved_iso
    assert [s for s in fake_mqtt.states if s[1] == "last_successful_poll"] == []
    assert not any("last_successful_poll" in p[0] and p[1] == "offline" for p in fake_mqtt.published)

    # 5. First success after failure publishes immediately even if < 10s since last publish
    temp_fail[0] = False
    mono_now[0] += 2.0
    fake_mqtt.states.clear()
    assert worker.step_once() is None
    assert len([s for s in fake_mqtt.states if s[1] == "last_successful_poll"]) == 1
    assert first_iso is not None

