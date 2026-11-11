"""Regression tests for relative redirects under Home Assistant Ingress."""
import json
from urllib.parse import urljoin
import pytest

pytest.importorskip("flask")

from conftest import load_module, ROOT

web_server = load_module("dev_web_server", "ha-idrac-controller-dev/app/web_server.py")

BASE = "https://x.ui.nabu.casa/api/hassio_ingress/TOKEN//"


@pytest.fixture
def client(tmp_path):
    config_file = str(tmp_path / "servers_config.json")
    with open(config_file, "w") as f:
        json.dump([{"alias": "r720", "idrac_ip": "192.168.1.100", "enabled": True}], f)

    web_server.SERVERS_CONFIG_FILE = config_file
    web_server.global_config = {
        "base_fan_speed_percent": 20,
        "low_temp_threshold": 45,
        "high_temp_fan_speed_percent": 50,
        "critical_temp_threshold": 65,
    }
    web_server.app.template_folder = str(ROOT / "ha-idrac-controller-dev/app/templates")
    web_server.app.config["TESTING"] = True
    with web_server.app.test_client() as test_client:
        yield test_client


def assert_redirect_to_manage_page(response, path):
    assert response.status_code == 302
    location = response.headers.get("Location")
    assert location is not None
    expected = urljoin(BASE, "servers")
    resolved = urljoin(urljoin(BASE, path), location)
    assert resolved == expected


def test_redirect_add_success(client):
    path = "servers/add"
    res = client.post(f"/{path}", data={"alias": "r730", "idrac_ip": "192.168.1.101"})
    assert_redirect_to_manage_page(res, path)


def test_redirect_add_duplicate_alias(client):
    path = "servers/add"
    res = client.post(f"/{path}", data={"alias": "r720", "idrac_ip": "192.168.1.100"})
    assert_redirect_to_manage_page(res, path)


def test_redirect_edit_unknown_alias(client):
    path = "servers/edit/unknown_server"
    res = client.get(f"/{path}")
    assert_redirect_to_manage_page(res, path)


def test_redirect_update_unknown_alias(client):
    path = "servers/update/unknown_server"
    res = client.post(f"/{path}", data={"idrac_ip": "192.168.1.102"})
    assert_redirect_to_manage_page(res, path)


def test_redirect_delete_existing(client):
    path = "servers/delete/r720"
    res = client.post(f"/{path}")
    assert_redirect_to_manage_page(res, path)


def test_redirect_delete_unknown_alias(client):
    path = "servers/delete/unknown_server"
    res = client.post(f"/{path}")
    assert_redirect_to_manage_page(res, path)


def test_render_servers_page_with_servers(client):
    res = client.get("/servers")
    assert res.status_code == 200
    html = res.get_data(as_text=True)
    pos_cfg = html.find("Configured Servers")
    pos_add = html.find("Add New Server")
    assert pos_cfg != -1 and pos_add != -1
    assert pos_cfg < pos_add
    assert "<details open>" not in html


def test_render_servers_page_empty(client, tmp_path):
    config_file = str(tmp_path / "empty_servers.json")
    with open(config_file, "w") as f:
        json.dump([], f)
    web_server.SERVERS_CONFIG_FILE = config_file
    res = client.get("/servers")
    assert res.status_code == 200
    html = res.get_data(as_text=True)
    pos_cfg = html.find("Configured Servers")
    pos_add = html.find("Add New Server")
    assert pos_cfg != -1 and pos_add != -1
    assert "<details open>" in html


def test_pid_form_zero_value_handling_and_validation(client):
    # Test zero deadband is preserved and not replaced by default 0.75
    form_data = {
        "fan_mode": "target",
        "target_temp": "45",
        "min_speed": "10",
        "max_speed": "80",
        "deadband": "0",
        "ti_seconds": "150",
    }
    client.post("/servers/update/r720", data=form_data)
    servers = web_server.load_servers_config()
    server = next(s for s in servers if s["alias"] == "r720")
    pid_cfg = server["pid_config"]
    assert pid_cfg["target_temp"] == 45
    assert pid_cfg["min_speed"] == 10
    assert pid_cfg["max_speed"] == 80
    assert pid_cfg["deadband"] == 0.0
    assert pid_cfg["ti_seconds"] == 150.0

    # Test range clamping
    clamped = web_server.parse_and_validate_pid_config({
        "target_temp": "10",  # min is 20
        "min_speed": "2",     # min is 5
        "max_speed": "4",     # max must be >= min (so 5)
        "deadband": "-1",     # min is 0
        "ti_seconds": "0",    # default to 300
        "ramp_down_percent_per_min": "0",  # min is 1
    })
    assert clamped["target_temp"] == 20
    assert clamped["min_speed"] == 5
    assert clamped["max_speed"] == 5
    assert clamped["deadband"] == 0.0
    assert clamped["ti_seconds"] == 300.0
    assert clamped["ramp_down_percent_per_min"] == 1

    clamped_high = web_server.parse_and_validate_pid_config({
        "ramp_down_percent_per_min": "100",  # max is 50
    })
    assert clamped_high["ramp_down_percent_per_min"] == 50


def test_poll_interval_form_handling(client):
    # Set custom poll interval
    client.post("/servers/update/r720", data={
        "poll_interval_seconds": "10",
        "ramp_down_percent_per_min": "6",
    })
    servers = web_server.load_servers_config()
    server = next(s for s in servers if s["alias"] == "r720")
    assert server["poll_interval_seconds"] == 10
    assert server["pid_config"]["ramp_down_percent_per_min"] == 6

    # Clamped to min 5
    client.post("/servers/update/r720", data={
        "poll_interval_seconds": "2",
    })
    servers = web_server.load_servers_config()
    server = next(s for s in servers if s["alias"] == "r720")
    assert server["poll_interval_seconds"] == 5

    # Blank resets to None (use global)
    client.post("/servers/update/r720", data={
        "poll_interval_seconds": "",
    })
    servers = web_server.load_servers_config()
    server = next(s for s in servers if s["alias"] == "r720")
    assert server["poll_interval_seconds"] is None


def test_render_index_with_pid_status(client, tmp_path):
    status_file = str(tmp_path / "status.json")
    with open(status_file, "w") as f:
        json.dump([{
            "alias": "r720",
            "ip": "192.168.1.100",
            "last_updated": "2026-11-08 12:00:00",
            "hottest_cpu_temp_c": 48.0,
            "inlet_temp_c": 22.0,
            "exhaust_temp_c": 32.0,
            "power_consumption_watts": 180,
            "target_fan_speed_percent": 25,
            "pid_status": "Learning (3 samples)",
        }], f)
    web_server.STATUS_FILE = status_file
    res = client.get("/")
    assert res.status_code == 200
    html = res.get_data(as_text=True)
    assert "Learning (3 samples)" in html


def test_add_server_with_snmp_settings(client):
    res = client.post("/servers/add", data={
        "alias": "r730-snmp",
        "idrac_ip": "192.168.1.105",
        "snmp_enabled": "true",
        "snmp_community": "mysecret",
        "snmp_port": "161",
        "disk_poll_minutes": "15",
    })
    assert res.status_code == 302
    servers = web_server.load_servers_config()
    server = next(s for s in servers if s["alias"] == "r730-snmp")
    assert server["snmp_enabled"] is True
    assert server["snmp_community"] == "mysecret"
    assert server["snmp_port"] == 161
    assert server["disk_poll_minutes"] == 15


def test_update_server_snmp_blank_community_keeps_existing(client):
    client.post("/servers/add", data={
        "alias": "r730-edit",
        "idrac_ip": "192.168.1.106",
        "snmp_enabled": "true",
        "snmp_community": "originalsecret",
    })
    # Update with empty community
    client.post("/servers/update/r730-edit", data={
        "idrac_ip": "192.168.1.106",
        "snmp_enabled": "true",
        "snmp_community": "",
        "disk_poll_minutes": "20",
    })
    servers = web_server.load_servers_config()
    server = next(s for s in servers if s["alias"] == "r730-edit")
    assert server["snmp_community"] == "originalsecret"
    assert server["disk_poll_minutes"] == 20

    # Ensure edit form does not echo community back in HTML value
    edit_res = client.get("/servers/edit/r730-edit")
    assert edit_res.status_code == 200
    html = edit_res.get_data(as_text=True)
    assert 'value="originalsecret"' not in html


def test_test_snmp_endpoint(client, monkeypatch):
    class MockResult:
        returncode = 0
        stdout = (
            "iso.3.6.1.4.1.674.10892.5.5.1.20.130.4.1.2.1 = STRING: \"Physical Disk 0:1:0\"\n"
            "iso.3.6.1.4.1.674.10892.5.5.1.20.130.4.1.4.1 = INTEGER: 3\n"
            "iso.3.6.1.4.1.674.10892.5.5.1.20.130.4.1.6.1 = STRING: \"ST91000640SS\"\n"
            "iso.3.6.1.4.1.674.10892.5.5.1.20.130.4.1.11.1 = INTEGER: 953869\n"
            "iso.3.6.1.4.1.674.10892.5.5.1.20.130.4.1.24.1 = INTEGER: 3\n"
            "iso.3.6.1.4.1.674.10892.5.5.1.20.130.4.1.31.1 = INTEGER: 0\n"
            "iso.3.6.1.4.1.674.10892.5.5.1.20.130.4.1.35.1 = INTEGER: 2\n"
            "iso.3.6.1.4.1.674.10892.5.5.1.20.130.4.1.54.1 = STRING: \"Disk.Bay.0:Enclosure.Internal.0-1:RAID.Integrated.1-1\"\n"
        )
        stderr = ""

    monkeypatch.setattr(web_server.subprocess, "run", lambda *a, **k: MockResult())

    # Success case
    res = client.post("/servers/test_snmp/r720", json={
        "idrac_ip": "192.168.1.100",
        "snmp_community": "public",
        "snmp_port": 161
    })
    assert res.status_code == 200
    data = res.get_json()
    assert data["success"] is True
    assert len(data["disks"]) == 1
    assert data["disks"][0]["bay"] == 0
    assert data["disks"][0]["model"] == "ST91000640SS"
    assert data["disks"][0]["health"] == "ok"
    assert "Physical Disk 0:1:0" in data["raw"]

    # Failure case: command fails
    class FailResult:
        returncode = 1
        stdout = ""
        stderr = "Timeout: No Response from 192.168.1.100"

    monkeypatch.setattr(web_server.subprocess, "run", lambda *a, **k: FailResult())
    res_fail = client.post("/servers/test_snmp/r720", json={
        "idrac_ip": "192.168.1.100",
        "snmp_community": "public"
    })
    assert res_fail.status_code == 200
    data_fail = res_fail.get_json()
    assert data_fail["success"] is False
    assert "Timeout: No Response" in data_fail["error"]


def test_render_index_with_physical_disks(client, tmp_path):
    status_file = str(tmp_path / "status.json")
    with open(status_file, "w") as f:
        json.dump([{
            "alias": "r720",
            "ip": "192.168.1.100",
            "last_updated": "2026-11-11 12:00:00",
            "hottest_cpu_temp_c": 48.0,
            "inlet_temp_c": 22.0,
            "exhaust_temp_c": 32.0,
            "power_consumption_watts": 180,
            "target_fan_speed_percent": 25,
            "disks": [
                {
                    "bay": 0,
                    "model": "ST91000640SS",
                    "size_gb": 931,
                    "media_type": "HDD",
                    "health": "ok",
                    "health_label": "OK",
                },
                {
                    "bay": 1,
                    "model": "ST91000640SS",
                    "size_gb": 931,
                    "media_type": "HDD",
                    "health": "warning",
                    "health_label": "Warning",
                }
            ],
        }], f)
    web_server.STATUS_FILE = status_file
    res = client.get("/")
    assert res.status_code == 200
    html = res.get_data(as_text=True)
    assert "Physical Disks" in html
    assert "ST91000640SS" in html
    assert "disk-problem" in html



