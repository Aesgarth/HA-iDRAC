import json
from pathlib import Path
import pytest
from conftest import load_module

dh = load_module("disk_health", "ha-idrac-controller-dev/app/disk_health.py")
BASE = "iso.3.6.1.4.1.674.10892.5.5.1.20.130.4.1"


def make_disk(index, **over):
    disk = {
        "name": f"Physical Disk 0:1:{index - 1}", "manufacturer": "ACME", "state": 3, "model": "TESTDISK1000",
        "serial": f"SER{index:04d}", "firmware": "A1", "size_mb": 953869, "status": 3, "smart": 0,
        "week": 12, "year": 2020, "media": 2, "endurance": 255,
        "fqdn": f"Disk.Bay.{index - 1}:Enclosure.Internal.0-1:RAID.Integrated.1-1",
        "display": f"Disk {index - 1} in Backplane 1 of Integrated RAID Controller 1",
    }
    disk.update(over)
    return disk


def make_walk(disks, style="iso"):
    columns = [(2, "name", "STRING"), (3, "manufacturer", "STRING"), (4, "state", "INTEGER"),
               (6, "model", "STRING"), (7, "serial", "STRING"), (8, "firmware", "STRING"),
               (11, "size_mb", "INTEGER"), (24, "status", "INTEGER"), (31, "smart", "INTEGER"),
               (33, "week", "STRING"), (34, "year", "STRING"), (35, "media", "INTEGER"),
               (49, "endurance", "INTEGER"), (54, "fqdn", "STRING"), (55, "display", "STRING")]
    prefix = BASE if style == "iso" else "." + BASE.replace("iso.", "1.", 1)
    lines = []
    for column, key, kind in columns:
        for index, disk in disks.items():
            value = disk[key]
            text = f'"{value}"' if kind == "STRING" else str(value)
            lines.append(f"{prefix}.{column}.{index} = {kind}: {text}")
    return "\n".join(lines) + "\n"


def disks_of(count, start=1):
    return {i: make_disk(i) for i in range(start, start + count)}


@pytest.mark.parametrize("count", [1, 2, 8, 24])
def test_autodetects_any_number_of_disks(count):
    parsed = dh.parse_walk(make_walk(disks_of(count)))
    assert len(parsed) == count
    assert dh.summarize(parsed)["count"] == count


def test_non_contiguous_indexes_are_found():
    disks = {1: make_disk(1), 5: make_disk(5), 9: make_disk(9)}
    assert sorted(dh.parse_walk(make_walk(disks))) == [1, 5, 9]


def test_fields_are_parsed():
    disk = dh.parse_walk(make_walk(disks_of(1)))[1]
    assert disk["slot"] == "Physical Disk 0:1:0"
    assert disk["name"].startswith("Disk 0 in Backplane 1")
    assert disk["model"] == "TESTDISK1000"
    assert disk["serial"] == "SER0001"
    assert disk["manufacturer"] == "ACME"
    assert disk["firmware"] == "A1"
    assert disk["size_gb"] == 932
    assert disk["media_type"] == "HDD"
    assert disk["raid_state"] == "online"
    assert disk["component_status"] == "ok"
    assert disk["smart_alert"] == "none"
    assert disk["manufactured"] == "2020-W12"
    assert disk["endurance_remaining"] is None
    assert disk["health"] == "ok"


def test_healthy_array_has_no_problems():
    summary = dh.summarize(dh.parse_walk(make_walk(disks_of(8))))
    assert summary["problem_count"] == 0
    assert summary["has_problem"] is False
    assert summary["worst"] == "ok"


def test_accepts_numeric_oid_format():
    assert len(dh.parse_walk(make_walk(disks_of(4), style="numeric"))) == 4


def test_smart_alert_is_detected():
    disks = disks_of(4)
    disks[3] = make_disk(3, smart=1)
    parsed = dh.parse_walk(make_walk(disks))
    assert parsed[3]["health"] == "predicted_failure"
    summary = dh.summarize(parsed)
    assert summary["has_problem"] and summary["problem_count"] == 1
    assert summary["worst"] == "predicted_failure"
    assert "Predicted failure" in summary["problems"][0]


def test_failed_state_outranks_smart_alert():
    disks = disks_of(5)
    disks[5] = make_disk(5, state=7, smart=1)
    parsed = dh.parse_walk(make_walk(disks))
    assert parsed[5]["health"] == "failed"
    assert dh.summarize(parsed)["worst"] == "failed"


def test_non_critical_status_is_a_warning():
    disks = disks_of(3)
    disks[2] = make_disk(2, status=4)
    assert dh.parse_walk(make_walk(disks))[2]["health"] == "warning"


def test_offline_disk_is_a_problem():
    disks = disks_of(4)
    disks[4] = make_disk(4, state=5)
    assert dh.parse_walk(make_walk(disks))[4]["health"] == "offline"


def test_ssd_endurance_is_reported_only_when_valid():
    disks = {1: make_disk(1, media=3, endurance=87), 2: make_disk(2, media=3, endurance=255)}
    parsed = dh.parse_walk(make_walk(disks))
    assert parsed[1]["media_type"] == "SSD" and parsed[1]["endurance_remaining"] == 87
    assert parsed[2]["endurance_remaining"] is None


def test_unrecognised_codes_are_unknown_not_ok():
    disks = {1: make_disk(1, state=99), 2: make_disk(2, status=42)}
    parsed = dh.parse_walk(make_walk(disks))
    assert parsed[1]["health"] == "unknown"
    assert parsed[2]["health"] == "unknown"


def test_walk_with_only_a_few_columns_still_parses():
    text = make_walk(disks_of(8))
    keep = (".2.", ".4.", ".24.", ".31.")
    lines = [line for line in text.splitlines() if any(f"130.4.1{k}" in line for k in keep)]
    parsed = dh.parse_walk("\n".join(lines))
    assert len(parsed) == 8
    assert parsed[1]["health"] == "ok"
    assert parsed[1]["model"] is None and parsed[1]["size_gb"] is None


def test_garbage_and_empty_input():
    assert dh.parse_walk("") == {}
    assert dh.parse_walk("Timeout: No Response from 192.168.0.87") == {}
    assert dh.parse_walk("No Such Object available on this agent at this OID") == {}


def test_walk_raises_when_snmpwalk_missing(monkeypatch):
    def boom(*a, **k):
        raise FileNotFoundError
    monkeypatch.setattr(dh.subprocess, "run", boom)
    with pytest.raises(dh.DiskPollError):
        dh.walk_physical_disks("192.0.2.1", "public")


def test_walk_raises_when_no_disks_returned(monkeypatch):
    class Result:
        returncode = 0
        stdout = "No Such Object available on this agent at this OID"
        stderr = ""
    monkeypatch.setattr(dh.subprocess, "run", lambda *a, **k: Result())
    with pytest.raises(dh.DiskPollError):
        dh.walk_physical_disks("192.0.2.1", "public")


REAL_CAPTURE = Path(__file__).parent / "fixtures" / "idrac7_r720_physical_disks.txt"


@pytest.mark.skipif(not REAL_CAPTURE.exists(), reason="optional real iDRAC capture not present")
def test_real_idrac_capture_parses():
    parsed = dh.parse_walk(REAL_CAPTURE.read_text(encoding="utf-8"))
    assert len(parsed) >= 1
    assert all(disk["health"] in dh.HEALTH_LABELS for disk in parsed.values())


class MockMqtt:
    def __init__(self):
        self.is_connected = True
        self.discoveries = []
        self.removals = []
        self.states = []

    def publish_discovery(self, component, slug, name, **kwargs):
        self.discoveries.append((component, slug, name, kwargs))

    def remove_discovery(self, component, slug):
        self.removals.append((component, slug))

    def publish_state(self, component, slug, state, attributes=None, retain=False):
        self.states.append((component, slug, state, attributes, retain))


def test_poller_success_publishes_mqtt():
    mqtt = MockMqtt()
    logs = []
    poller = dh.DiskHealthPoller(
        host="192.168.1.50",
        community="public",
        poll_minutes=10,
        mqtt_client=mqtt,
        log_fn=lambda lvl, msg: logs.append((lvl, msg)),
    )
    walk_text = make_walk(disks_of(2))
    disks = dh.parse_walk(walk_text)
    poller._handle_success(disks)

    slugs = [d[1] for d in mqtt.discoveries]
    assert "disks_summary" in slugs
    assert "disks_problem" in slugs
    assert "disk_1_health" in slugs
    assert "disk_2_health" in slugs

    state_slugs = [s[1] for s in mqtt.states]
    assert "disks_summary" in state_slugs
    assert "disks_problem" in state_slugs
    assert "disk_1_health" in state_slugs
    assert "disk_2_health" in state_slugs

    summary_entry = next(s for s in mqtt.states if s[1] == "disks_summary")
    assert summary_entry[2] == "All 2 disks OK"
    assert summary_entry[4] is True

    prob_entry = next(s for s in mqtt.states if s[1] == "disks_problem")
    assert prob_entry[2] == "OFF"
    assert prob_entry[4] is True


def test_poller_removed_disk_cleanup():
    mqtt = MockMqtt()
    poller = dh.DiskHealthPoller(
        host="192.168.1.50",
        community="public",
        mqtt_client=mqtt,
    )
    disks2 = dh.parse_walk(make_walk(disks_of(2)))
    poller._handle_success(disks2)
    assert poller.discovered_disks == {1, 2}

    disks1 = dh.parse_walk(make_walk(disks_of(1)))
    poller._handle_success(disks1)
    assert poller.discovered_disks == {1}
    assert ("sensor", "disk_2_health") in mqtt.removals


def test_poller_failures_and_recovery():
    mqtt = MockMqtt()
    logs = []
    poller = dh.DiskHealthPoller(
        host="192.168.1.50",
        community="public",
        mqtt_client=mqtt,
        log_fn=lambda lvl, msg: logs.append((lvl, msg)),
    )
    disks = dh.parse_walk(make_walk(disks_of(1)))
    poller._handle_success(disks)

    poller._handle_failure(dh.DiskPollError("timeout"))
    assert len([lvl for lvl, msg in logs if lvl == "warning"]) == 1
    assert poller.consecutive_failures == 1

    poller._handle_failure(dh.DiskPollError("timeout"))
    assert len([lvl for lvl, msg in logs if lvl == "warning"]) == 1

    poller._handle_failure(dh.DiskPollError("timeout"))
    unknown_entries = [s for s in mqtt.states if s[2] == "Unknown"]
    assert len(unknown_entries) >= 2

    poller._handle_success(disks)
    assert poller.consecutive_failures == 0
    assert any(lvl == "info" and "recovered" in msg for lvl, msg in logs)


def test_poller_stop_terminates_promptly():
    poller = dh.DiskHealthPoller(
        host="192.168.1.50",
        community="public",
        poll_minutes=10,
    )
    poller.start()
    poller.stop()
    poller.join(timeout=1.0)
    assert not poller.is_alive()


class CapturingFakeMqtt:
    def __init__(self, base_topic="ha_idrac_controller/r720"):
        self.is_connected = True
        self.base_topic = base_topic
        self.availability_topic = f"{base_topic}/status"
        self.device_info_dict = {"identifiers": ["idrac_controller_r720"]}
        self.published = {}
        self.discoveries = {}

    def publish(self, topic, payload, retain=False, qos=0):
        self.published[topic] = (payload, retain)
        return True

    def publish_discovery(self, component, slug, name, device_class=None, unit=None, icon=None,
                          cmd_topic=None, val_template=None, state_class=None):
        unique_id = f"{self.device_info_dict['identifiers'][0]}_{slug}"
        config_topic = f"homeassistant/{component}/{unique_id}/config"
        payload = {
            "name": name,
            "unique_id": unique_id,
            "availability_topic": self.availability_topic,
        }
        if component == "sensor":
            payload["state_topic"] = f"{self.base_topic}/sensor/{slug}"
            payload["json_attributes_topic"] = f"{self.base_topic}/sensor/{slug}"
            payload["value_template"] = "{{ value_json.state }}"
        elif component == "binary_sensor":
            payload["state_topic"] = f"{self.base_topic}/binary_sensor/{slug}"
            payload["payload_on"] = "ON"
            payload["payload_off"] = "OFF"
        self.discoveries[slug] = (component, payload)
        self.publish(config_topic, json.dumps(payload), retain=True)

    def publish_state(self, component, slug, state, attributes=None, retain=False):
        topic = f"{self.base_topic}/{component}/{slug}"
        if component == "sensor":
            payload = {"state": state}
            if attributes:
                payload.update(attributes)
            self.publish(topic, json.dumps(payload), retain=retain)
        else:
            self.publish(topic, state, retain=retain)


def test_discovery_and_state_publishing_contract():
    mqtt = CapturingFakeMqtt()
    logs = []
    poller = dh.DiskHealthPoller(
        host="192.168.1.50",
        community="public",
        mqtt_client=mqtt,
        log_fn=lambda lvl, msg: logs.append((lvl, msg)),
    )
    disks = dh.parse_walk(make_walk(disks_of(2)))
    poller._handle_success(disks)

    # 4 entities discovered: 2 disks + 1 summary + 1 problem binary sensor
    assert len(mqtt.discoveries) == 4
    assert "disks_summary" in mqtt.discoveries
    assert "disks_problem" in mqtt.discoveries
    assert "disk_1_health" in mqtt.discoveries
    assert "disk_2_health" in mqtt.discoveries

    # Verify discovery-state contract for each entity
    for slug, (component, disc_payload) in mqtt.discoveries.items():
        state_topic = disc_payload["state_topic"]
        assert state_topic in mqtt.published, f"Topic {state_topic} was not published"
        raw_payload, retain = mqtt.published[state_topic]
        assert retain is True, f"State for {slug} must be published with retain=True"

        if component == "sensor":
            assert disc_payload["value_template"] == "{{ value_json.state }}"
            parsed = json.loads(raw_payload)
            assert "state" in parsed, f"JSON payload for sensor {slug} missing 'state' key"
            assert parsed["state"] is not None
        elif component == "binary_sensor":
            assert disc_payload["payload_on"] == "ON"
            assert disc_payload["payload_off"] == "OFF"
            assert raw_payload in ("ON", "OFF"), f"Binary sensor state must be 'ON' or 'OFF', got {raw_payload}"

    # Verify republishing cached states and debug logging
    mqtt.published.clear()
    poller.republish_cached_states(reason="test")
    assert len(mqtt.published) == 4
    for slug, (component, disc_payload) in mqtt.discoveries.items():
        assert disc_payload["state_topic"] in mqtt.published
        _, retain = mqtt.published[disc_payload["state_topic"]]
        assert retain is True

    debug_logs = [msg for lvl, msg in logs if lvl == "debug" and "Republished 4 disk states" in msg]
    assert len(debug_logs) >= 1


def test_poller_run_loop_republishes_3s_after_discovery(monkeypatch):
    import time
    mqtt = CapturingFakeMqtt()
    logs = []
    poller = dh.DiskHealthPoller(
        host="192.168.1.50",
        community="public",
        poll_minutes=10,
        mqtt_client=mqtt,
        log_fn=lambda lvl, msg: logs.append((lvl, msg)),
    )
    disks = dh.parse_walk(make_walk(disks_of(2)))
    monkeypatch.setattr(dh, "walk_physical_disks", lambda *a, **k: disks)

    poller.start()
    # Initial startup settle is 1s, then SNMP walk happens, then 3s delayed republish
    # Total wait ~4.5s
    time.sleep(4.5)
    poller.stop()
    poller.join(timeout=1.0)

    debug_3s = [msg for lvl, msg in logs if lvl == "debug" and "3s post-discovery" in msg]
    assert len(debug_3s) == 1


def test_disk_fields_never_collide_with_mqtt_payload_keys():
    disk = dh.parse_walk(make_walk(disks_of(1)))[1]
    assert "state" not in disk and "status" not in disk
    assert disk["health_label"] == "OK"


def test_published_disk_sensor_state_equals_health_label_and_contains_attributes():
    mqtt = CapturingFakeMqtt()
    poller = dh.DiskHealthPoller(
        host="192.168.1.50",
        community="public",
        mqtt_client=mqtt,
    )
    disks = dh.parse_walk(make_walk(disks_of(1)))
    poller._publish_disks_mqtt(disks, dh.summarize(disks))

    topic = f"{mqtt.base_topic}/sensor/disk_1_health"
    assert topic in mqtt.published
    raw_payload, _ = mqtt.published[topic]
    data = json.loads(raw_payload)

    # State must equal health_label ("OK"), NOT raid_state ("online")
    assert data["state"] == "OK"
    assert data["state"] == disks[1]["health_label"]

    # Must contain model, serial, raid_state, component_status
    assert data["model"] == "TESTDISK1000"
    assert data["serial"] == "SER0001"
    assert data["raid_state"] == "online"
    assert data["component_status"] == "ok"
    assert "status" not in data




