import io
import json
import pytest

pytest.importorskip("flask")

import conftest  # noqa: F401, E402
from conftest import load_module, resolve_addon_path  # noqa: E402
from app import main, watchdog  # noqa: E402

web_server = load_module("dev_web_server_exp_imp", "ha-idrac-controller-dev/app/web_server.py")


def _sample_server(alias="r720", ip="10.0.0.10"):
    return {
        "alias": alias,
        "idrac_ip": ip,
        "idrac_username": "root",
        "idrac_password": "top-secret-password",
        "enabled": True,
        "fan_control_enabled": True,
        "disable_pcie_cooling_response": False,
        "on_fan_fault": "dell_auto",
        "min_safe_fan_speed": 20,
        "snmp_enabled": True,
        "snmp_community": "private-community-str",
        "snmp_port": 161,
        "disk_poll_minutes": 10,
        "fan_mode": "target",
        "base_fan_speed_percent": 20,
        "low_temp_threshold": 45,
        "high_temp_fan_speed_percent": 50,
        "critical_temp_threshold": 80,
        "poll_interval_seconds": 10,
        "pid_config": {
            "target_temp": 45,
            "min_speed": 20,
            "max_speed": 85,
            "deadband": 0.75,
            "ti_seconds": 300.0,
            "ramp_down_percent_per_min": 4,
        },
        "fan_curve": [],
    }


@pytest.fixture
def env_setup(tmp_path, monkeypatch):
    cfg_file = str(tmp_path / "servers_config.json")
    pid_file = str(tmp_path / "pid_states.json")
    hb_file = str(tmp_path / "hb.json")

    monkeypatch.setattr(watchdog, "HEARTBEAT_FILE", hb_file)
    monkeypatch.setattr(main, "PID_STATE_FILE", pid_file)
    web_server.SERVERS_CONFIG_FILE = cfg_file
    web_server.PID_STATE_FILE = pid_file
    web_server.app.template_folder = str(resolve_addon_path("ha-idrac-controller-dev/app/templates"))
    web_server.app.config["TESTING"] = True

    mgr = main.ServerManager(
        {"log_level": "warning", "check_interval_seconds": 30},
        config_file=cfg_file,
        config_lock=web_server.config_lock,
        workers=[],
        threads_list=[],
    )
    web_server.server_manager = mgr
    yield web_server.app.test_client(), mgr, cfg_file, pid_file
    web_server.server_manager = None


def test_export_strips_secrets_and_nested_secrets_with_and_without_learned_model(env_setup):
    client, mgr, cfg_file, pid_file = env_setup
    srv = _sample_server("r720", "10.0.0.10")
    srv["nested_extra"] = {"api_token": "tok-123", "my_secret_key": "sec-456", "safe_key": "ok"}
    with open(cfg_file, "w") as f:
        json.dump([srv], f)

    learned = {
        "theta": [0.14, 0.035],
        "cov": [[0.5, 0.0], [0.0, 0.5]],
        "trim": 1.2,
        "speed": 28.0,
        "samples": 18,
        "power_floor": {"2": [115.0, 30], "6": [165.0, 25]},
    }
    main.save_pid_state("r720", learned, filepath=pid_file)

    # 1. Export without learned model (default)
    res = client.get("/servers/export")
    assert res.status_code == 200
    assert "attachment; filename=ha-idrac-servers.json" in res.headers.get("Content-Disposition", "")
    raw_text = res.get_data(as_text=True)
    assert "top-secret-password" not in raw_text
    assert "private-community-str" not in raw_text
    assert "tok-123" not in raw_text
    assert "sec-456" not in raw_text

    payload = json.loads(raw_text)
    assert payload["format"] == "ha-idrac-servers"
    assert payload["version"] == 1
    assert "name" in payload["addon"] and "version" in payload["addon"]
    s0 = payload["servers"][0]
    assert s0["idrac_password"] is None
    assert s0["snmp_community"] is None
    assert s0["nested_extra"]["api_token"] is None
    assert s0["nested_extra"]["my_secret_key"] is None
    assert s0["nested_extra"]["safe_key"] == "ok"
    assert "pid_state" not in s0

    # 2. Export with learned model
    res_learned = client.get("/servers/export?include_learned=true")
    payload_learned = json.loads(res_learned.get_data(as_text=True))
    assert payload_learned["servers"][0]["pid_state"]["theta"] == [0.14, 0.035]
    assert payload_learned["servers"][0]["pid_state"]["power_floor"] == {"2": [115.0, 30], "6": [165.0, 25]}


def test_export_and_import_round_trip_into_fresh_manager_with_pid_state(tmp_path, monkeypatch):
    # Export from manager 1
    cfg1 = str(tmp_path / "servers1.json")
    pid1 = str(tmp_path / "pid1.json")
    monkeypatch.setattr(watchdog, "HEARTBEAT_FILE", str(tmp_path / "hb.json"))
    monkeypatch.setattr(main, "PID_STATE_FILE", pid1)
    web_server.SERVERS_CONFIG_FILE = cfg1
    web_server.PID_STATE_FILE = pid1
    web_server.app.config["TESTING"] = True

    with open(cfg1, "w") as f:
        json.dump([_sample_server("r720", "10.0.0.10"), _sample_server("r730", "10.0.0.20")], f)
    learned = {
        "theta": [0.155, 0.042],
        "cov": [[0.4, 0.0], [0.0, 0.4]],
        "trim": 0.5,
        "speed": 24.0,
        "samples": 22,
        "power_floor": {"2": [112.0, 25], "7": [180.0, 25]},
    }
    main.save_pid_state("r720", learned, filepath=pid1)

    with web_server.app.test_client() as client:
        exported_bytes = client.get("/servers/export?include_learned=true").get_data()

    # Import into fresh manager 2
    cfg2 = str(tmp_path / "servers2.json")
    pid2 = str(tmp_path / "pid2.json")
    with open(cfg2, "w") as f:
        json.dump([], f)
    monkeypatch.setattr(main, "PID_STATE_FILE", pid2)
    web_server.SERVERS_CONFIG_FILE = cfg2
    web_server.PID_STATE_FILE = pid2

    mgr2 = main.ServerManager(
        {"log_level": "warning", "check_interval_seconds": 30},
        config_file=cfg2,
        config_lock=web_server.config_lock,
        workers=[],
        threads_list=[],
    )
    web_server.server_manager = mgr2

    with web_server.app.test_client() as client:
        res = client.post(
            "/servers/import",
            data={"file": (io.BytesIO(exported_bytes), "ha-idrac-servers.json")},
            content_type="multipart/form-data",
        )
        assert res.status_code == 302
        assert res.headers["Location"] == "../servers"

    imported = web_server.load_servers_config()
    assert len(imported) == 2
    for srv in imported:
        assert srv["enabled"] is False
        assert srv["idrac_password"] == ""
        assert srv["snmp_community"] == ""
    assert len(mgr2.workers) == 0  # disabled servers do not start controlling fans

    # Verify learned PID state round-tripped and loads into a ServerWorker
    worker = main.ServerWorker(imported[0], mgr2.global_opts)
    assert worker.pid.theta == [0.155, 0.042]
    assert worker.pid.power_floor == {2: (112.0, 25), 7: (180.0, 25)}
    web_server.server_manager = None


def test_import_rejections_and_duplicate_skip(env_setup):
    client, mgr, cfg_file, pid_file = env_setup
    existing_srv = _sample_server("r720", "10.0.0.10")
    with open(cfg_file, "w") as f:
        json.dump([existing_srv], f)

    # 1. Invalid JSON
    client.post(
        "/servers/import",
        data={"file": (io.BytesIO(b"{not json"), "bad.json")},
        content_type="multipart/form-data",
    )
    assert len(web_server.load_servers_config()) == 1

    # 2. Wrong format marker / version
    wrong_fmt = json.dumps({"format": "other", "version": 99, "servers": []}).encode()
    client.post(
        "/servers/import",
        data={"file": (io.BytesIO(wrong_fmt), "wrong.json")},
        content_type="multipart/form-data",
    )
    assert len(web_server.load_servers_config()) == 1

    # 3. Oversize file (> 1 MB)
    oversize = b" " * (1024 * 1024 + 10)
    client.post(
        "/servers/import",
        data={"file": (io.BytesIO(oversize), "huge.json")},
        content_type="multipart/form-data",
    )
    assert len(web_server.load_servers_config()) == 1

    # 4. One valid + one invalid server -> nothing imported
    s_good = _sample_server("r730", "10.0.0.20")
    s_bad = _sample_server("r740", "10.0.0.30")
    s_bad["pid_config"]["min_speed"] = 95
    s_bad["pid_config"]["max_speed"] = 20
    batch_with_invalid = json.dumps({
        "format": "ha-idrac-servers",
        "version": 1,
        "servers": [s_good, s_bad],
    }).encode()
    client.post(
        "/servers/import",
        data={"file": (io.BytesIO(batch_with_invalid), "batch.json")},
        content_type="multipart/form-data",
    )
    assert [s["alias"] for s in web_server.load_servers_config()] == ["r720"]

    # 5. Duplicate alias skipped, new server imported (with corrupt pid_state discarded safely)
    s_dup = _sample_server("r720", "10.0.0.99")
    s_new = _sample_server("r730", "10.0.0.20")
    s_new["pid_state"] = {"theta": [999.0, -5.0], "cov": "bad"}
    batch_dup = json.dumps({
        "format": "ha-idrac-servers",
        "version": 1,
        "servers": [s_dup, s_new],
    }).encode()
    client.post(
        "/servers/import",
        data={"file": (io.BytesIO(batch_dup), "dup.json")},
        content_type="multipart/form-data",
    )
    configs = web_server.load_servers_config()
    assert [s["alias"] for s in configs] == ["r720", "r730"]
    # Existing r720 was not overwritten
    assert configs[0]["idrac_ip"] == "10.0.0.10"
    assert configs[0]["enabled"] is True
    # New r730 is disabled with blank password
    assert configs[1]["idrac_ip"] == "10.0.0.20"
    assert configs[1]["enabled"] is False
    assert configs[1]["idrac_password"] == ""
