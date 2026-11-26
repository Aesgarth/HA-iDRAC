import sys
import pytest
from conftest import load_module

md = load_module("make_dashboard", "tools/make_dashboard.py")


def _full_entities(alias="my_server"):
    p = f"idrac_{alias}"
    ents = [
        f"sensor.{p}_hottest_cpu_temp",
        f"sensor.{p}_inlet_temp",
        f"sensor.{p}_exhaust_temp",
        f"sensor.{p}_power",
        f"sensor.{p}_target_fan_speed",
        f"sensor.{p}_cpu_0_temperature",
        f"sensor.{p}_cpu_1_temperature",
        f"sensor.{p}_disks_last_checked",
        f"sensor.{p}_psu1_voltage",
        f"sensor.{p}_psu1_current",
        f"sensor.{p}_psu2_voltage",
        f"sensor.{p}_psu2_current",
        f"sensor.{p}_last_successful_poll",
        f"binary_sensor.{p}_fan_fault",
        f"binary_sensor.{p}_idrac_reachable",
        f"binary_sensor.{p}_disks_problem",
        f"binary_sensor.{p}_disk_monitoring_failing",
        f"binary_sensor.{p}_psu1",
        f"binary_sensor.{p}_psu2",
        f"binary_sensor.{p}_ps_redundancy",
        f"binary_sensor.{p}_fan_redundancy",
        f"button.{p}_shutdown_server",
    ]
    for i in range(1, 7):
        ents.append(f"sensor.{p}_fan{i}_rpm")
    for i in range(8):
        ents.append(f"sensor.{p}_disk_{i}_health")
    return ents


def _run_cli(tmp_path, monkeypatch, capsys, entities, extra_args=None):
    ent_file = tmp_path / "entities.txt"
    ent_file.write_text("\n".join(entities) + "\n", encoding="utf-8")
    argv = ["make_dashboard.py", str(ent_file)] + (extra_args or [])
    monkeypatch.setattr(sys, "argv", argv)
    md.main()
    return capsys.readouterr().out


def test_output_mentions_every_existing_entity_and_no_missing_entity(tmp_path, monkeypatch, capsys):
    ents = _full_entities("my_server")
    out = _run_cli(tmp_path, monkeypatch, capsys, ents, ["--shutdown"])
    for e in ents:
        assert e in out, f"Expected {e} in generated YAML"
    assert "sensor.idrac_my_server_nonexistent" not in out
    assert "sensor.idrac_my_server_disk_99_health" not in out
    assert "\t" not in out

    # Without optional entities, those entities do not appear
    minimal = ["sensor.idrac_my_server_hottest_cpu_temp", "sensor.idrac_my_server_power"]
    out_min = _run_cli(tmp_path, monkeypatch, capsys, minimal)
    assert "sensor.idrac_my_server_hottest_cpu_temp" in out_min
    assert "sensor.idrac_my_server_power" in out_min
    assert "psu1_voltage" not in out_min
    assert "disk_0_health" not in out_min


def test_multiple_servers_and_server_filter(tmp_path, monkeypatch, capsys):
    ents = _full_entities("my_server") + _full_entities("r730")
    out_all = _run_cli(tmp_path, monkeypatch, capsys, ents)
    assert "path: my-server" in out_all
    assert "path: r730" in out_all

    out_one = _run_cli(tmp_path, monkeypatch, capsys, ents, ["--server=idrac_my_server"])
    assert "path: my-server" in out_one
    assert "path: r730" not in out_one


def test_shutdown_flag_controls_shutdown_tile_and_confirmation(tmp_path, monkeypatch, capsys):
    ents = _full_entities("my_server")
    out_no_shut = _run_cli(tmp_path, monkeypatch, capsys, ents)
    assert "button.idrac_my_server_shutdown_server" not in out_no_shut
    assert "Shut down this server?" not in out_no_shut

    out_shut = _run_cli(tmp_path, monkeypatch, capsys, ents, ["--shutdown"])
    assert "button.idrac_my_server_shutdown_server" in out_shut
    assert "Shut down this server?" in out_shut


def test_stale_and_unrelated_entities_ignored(tmp_path, monkeypatch, capsys):
    ents = _full_entities("my_server") + [
        "sensor.idrac_controller_192_168_0_87_inlet_temperature",
        "update.ha_idrac_controller_standard_update",
        "device_tracker.idrac_my_server",
    ]
    out = _run_cli(tmp_path, monkeypatch, capsys, ents)
    assert "192_168_0_87" not in out
    assert "update.ha_idrac" not in out
    assert "device_tracker." not in out


def test_empty_or_unrelated_list_exits_with_error(tmp_path, monkeypatch, capsys):
    with pytest.raises(SystemExit) as exc_empty:
        _run_cli(tmp_path, monkeypatch, capsys, [])
    assert "No iDRAC server found" in str(exc_empty.value)

    with pytest.raises(SystemExit) as exc_unrel:
        _run_cli(tmp_path, monkeypatch, capsys, ["sensor.idrac_controller_192_168_0_87_inlet_temperature", "update.foo"])
    assert "No iDRAC server found" in str(exc_unrel.value)


def test_generated_output_parses_as_yaml(tmp_path, monkeypatch, capsys):
    yaml = pytest.importorskip("yaml")
    ents = _full_entities("my_server")
    out = _run_cli(tmp_path, monkeypatch, capsys, ents, ["--shutdown"])
    assert "\t" not in out
    parsed = yaml.safe_load(out)
    assert parsed["title"] == "Servers"
    assert len(parsed["views"]) == 1
    assert parsed["views"][0]["path"] == "my-server"
