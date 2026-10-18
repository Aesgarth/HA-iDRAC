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
