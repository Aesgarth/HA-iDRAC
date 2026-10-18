"""Parser tests against output shapes from 12G (iDRAC7) and 13G (iDRAC8) servers."""
from conftest import load_module

ipmi_mod = load_module("dev_ipmi_manager", "ha-idrac-controller-dev/app/ipmi_manager.py")
stable_mod = load_module("stable_ipmi_manager", "ha-idrac-controller/app/ipmi_manager.py")

# Dell PowerEdge R730/T430 (13th gen, iDRAC8): CPU sensors carry the socket number.
SDR_13G = """\
CPU1 Temp        | 01h | ok  |  3.1 | 45 degrees C
CPU2 Temp        | 02h | ok  |  3.2 | 47 degrees C
Inlet Temp       | 04h | ok  |  7.1 | 22 degrees C
Exhaust Temp     | 05h | ok  |  7.2 | 30 degrees C
DIMM A1 Temp     | 20h | ok  | 32.1 | 40 degrees C
"""

# Dell PowerEdge R720 (12th gen, iDRAC7): sensors are literally named "Temp".
SDR_12G = """\
Temp             | 01h | ok  |  3.1 | 40 degrees C
Temp             | 02h | ok  |  3.2 | 42 degrees C
Inlet Temp       | 04h | ok  |  7.1 | 20 degrees C
Exhaust Temp     | 05h | ok  |  7.2 | 28 degrees C
"""

FAN_SDR = """\
Fan1A Tach       | 30h | ok  |  7.1 | 2040 RPM
Fan1B Tach       | 31h | ok  |  7.2 | 2160 RPM
Fan Redundancy   | 32h | ok  |  7.3 | 0x00
"""

ELIST = """\
PS1 Status       | 0Ah | ok  | 10.1 | Presence detected
PS2 Status       | 0Bh | ok  | 10.2 | Presence detected
PS1 PG Fail      | 0Ch | ok  | 10.1 |
PS2 PG Fail      | 0Dh | ok  | 10.2 |
Pwr Consumption  | 77h | ok  | 10.1 | 196 Watts
"""


def make_manager():
    return ipmi_mod.IPMIManager("10.0.0.10", "root", "secret", log_level="fatal")


def test_13g_cpu_sensors_are_detected_by_entity_id():
    temps = make_manager().parse_temperatures(SDR_13G)
    assert temps["cpu_temps"] == [45, 47]
    assert temps["inlet_temp"] == 22
    assert temps["exhaust_temp"] == 30
    assert 40 not in temps["cpu_temps"]  # DIMM sensor must not drive fan control


def test_12g_sensors_named_temp_are_detected():
    temps = make_manager().parse_temperatures(SDR_12G)
    assert temps["cpu_temps"] == [40, 42]


def test_stable_parser_handles_both_12g_and_13g_names():
    pattern = r"(?:^Temp$|CPU\s?\d)"
    for sdr, expected in ((SDR_13G, [45, 47]), (SDR_12G, [40, 42])):
        parsed = stable_mod.parse_temperatures(sdr, pattern, r"Inlet Temp", r"Exhaust Temp")
        assert parsed["cpu_temps"] == expected
        assert parsed["inlet_temp"] in (22, 20)
        assert parsed["exhaust_temp"] in (30, 28)


def test_fan_rpms_are_parsed_and_non_tach_rows_ignored():
    fans = make_manager().parse_fan_rpms(FAN_SDR)
    assert fans == [{"name": "Fan1A Tach", "rpm": 2040}, {"name": "Fan1B Tach", "rpm": 2160}]


def test_power_consumption_from_sdr_elist():
    assert make_manager().parse_power_consumption(ELIST) == 196


def test_psu_status_uses_presence_and_pg_fail_not_voltage():
    psus = make_manager().get_power_status(ELIST)
    assert psus == [{"name": "PSU1", "ok": True}, {"name": "PSU2", "ok": True}]


def test_psu_fault_detected():
    data = "PS1 Status       | 0Ah | ok  | 10.1 | Presence detected\n" \
           "PS1 PG Fail      | 0Ch | cr  | 10.1 |\n"
    assert make_manager().get_power_status(data) == [{"name": "PSU1", "ok": False}]


def test_fan_speed_floor_prevents_zero_percent():
    manager = make_manager()
    assert manager._decimal_to_hex_for_ipmi(0) == "0x05"
    assert manager._decimal_to_hex_for_ipmi(3) == "0x05"
    assert manager._decimal_to_hex_for_ipmi(100) == "0x64"
    assert manager._decimal_to_hex_for_ipmi(250) == "0x64"


def test_ipmi_password_is_not_in_the_process_arguments():
    manager = make_manager()
    assert "-E" in manager.base_args
    assert "secret" not in manager.base_args


def test_empty_psu_bay_is_not_reported_as_failed():
    data = "PS1 Status       | 0Ah | ok  | 10.1 | Presence detected\n" \
           "PS2 Status       | 0Bh | ok  | 10.2 | 0x00\n"
    assert make_manager().get_power_status(data) == [{"name": "PSU1", "ok": True}]


def test_stable_fan_floor_prevents_zero_percent():
    assert stable_mod.decimal_to_hex_for_ipmi(0) == "0x05"
    assert stable_mod.decimal_to_hex_for_ipmi(3) == "0x05"
    assert stable_mod.decimal_to_hex_for_ipmi(100) == "0x64"


SDR_WITH_PSU_INLET = """\
PSU1 Inlet Temp   | 0Dh | ok  | 10.1 | 34 degrees C
Inlet Temp       | 04h | ok  |  7.1 | 22 degrees C
CPU1 Temp        | 01h | ok  |  3.1 | 45 degrees C
"""


def test_psu_inlet_sensor_does_not_override_chassis_inlet():
    temps = make_manager().parse_temperatures(SDR_WITH_PSU_INLET)
    assert temps["inlet_temp"] == 22
    assert temps["cpu_temps"] == [45]


def test_third_party_cooling_response_uses_dell_oem_bytes(monkeypatch):
    manager = make_manager()
    calls = []
    monkeypatch.setattr(manager, "_run_ipmi_command", lambda args, **kwargs: calls.append(args) or "ok")

    assert manager.set_third_party_pcie_cooling_response(False) is True
    assert calls[-1] == ["0x30", "0xce", "0x00", "0x16", "0x05", "0x00", "0x00", "0x00",
                         "0x05", "0x00", "0x01", "0x00", "0x00"]

    assert manager.set_third_party_pcie_cooling_response(True) is True
    assert calls[-1][-3:] == ["0x00", "0x00", "0x00"]


def test_refusal_codes_disable_fan_control_until_something_was_accepted():
    manager = make_manager()
    manager.last_stderr = "Unable to send RAW command (channel=0x0 netfn=0x30 lun=0x0 cmd=0x30 rsp=0xd4): Insufficient privilege level"
    assert manager._is_fan_control_refusal() is True

    manager.fan_control_ever_accepted = True
    assert manager._is_fan_control_refusal() is False


def test_failed_speed_command_hands_fans_back_to_dell(monkeypatch):
    manager = make_manager()
    manager._log = lambda *args, **kwargs: None
    monkeypatch.setattr("time.sleep", lambda *_: None)
    calls = []
    replies = iter(["ok", None, "ok"])

    def fake_run(args, **kwargs):
        calls.append(args)
        return next(replies)

    monkeypatch.setattr(manager, "_run_ipmi_command", fake_run)
    assert manager.apply_user_fan_control_profile(30) is None
    assert calls[0] == ["0x30", "0x30", "0x01", "0x00"]
    assert calls[1] == ["0x30", "0x30", "0x02", "0xff", "0x1e"]
    assert calls[2] == ["0x30", "0x30", "0x01", "0x01"]  # Dell auto hand-back


def test_firmware_version_parsed(monkeypatch):
    manager = make_manager()
    monkeypatch.setattr(manager, "_run_ipmi_command",
                        lambda *args, **kwargs: "Firmware Revision : 2.86.45.45\n")
    assert manager.get_firmware_version() == "2.86.45.45"


def test_serial_number_parsed_from_fru(monkeypatch):
    manager = make_manager()
    fru = "Product Manufacturer : DELL\nProduct Name : PowerEdge T430\nProduct Serial : ABC1234\n"
    monkeypatch.setattr(manager, "_run_ipmi_command", lambda *args, **kwargs: fru)
    info = manager.get_server_model_info()
    assert info["model"] == "PowerEdge T430"
    assert info["serial"] == "ABC1234"


def test_implausible_temperature_is_ignored():
    sdr = "CPU1 Temp | 01h | ok | 3.1 | 255 degrees C\n" \
          "CPU2 Temp | 02h | ok | 3.2 | 47 degrees C\n"
    assert make_manager().parse_temperatures(sdr)["cpu_temps"] == [47]


def test_stable_refusal_helpers_are_defined():
    stable_mod._FAN_CONTROL_REFUSED = False
    stable_mod._FAN_CONTROL_EVER_ACCEPTED = False
    stable_mod._LAST_STDERR = "Unable to send RAW command ... rsp=0xd4"
    assert stable_mod._is_fan_control_refusal() is True
    stable_mod._note_fan_control_refusal()
    assert stable_mod.is_fan_control_refused() is True
    # reset module state for any later test
    stable_mod._FAN_CONTROL_REFUSED = False
    stable_mod._LAST_STDERR = ""
