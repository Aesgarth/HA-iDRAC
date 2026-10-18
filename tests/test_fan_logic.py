"""Threshold/hysteresis tests for the 3-tier fan control."""
from conftest import load_module

fan_logic = load_module("dev_fan_logic", "ha-idrac-controller-dev/app/fan_logic.py")


def test_simple_mode_uses_base_below_low_threshold():
    state = fan_logic.SimpleFanState()
    assert state.simple_speed(40, low_temp_c=45, base_speed=20, high_speed=50) == 20


def test_simple_mode_switches_to_high_at_low_threshold():
    state = fan_logic.SimpleFanState()
    assert state.simple_speed(45, low_temp_c=45, base_speed=20, high_speed=50) == 50


def test_simple_mode_keeps_high_inside_hysteresis_band():
    state = fan_logic.SimpleFanState()
    assert state.simple_speed(46, low_temp_c=45, base_speed=20, high_speed=50) == 50
    # 44 C is below the threshold but inside the 2 C hysteresis band.
    assert state.simple_speed(44, low_temp_c=45, base_speed=20, high_speed=50) == 50
    # 42 C is outside the band and drops back to base.
    assert state.simple_speed(42, low_temp_c=45, base_speed=20, high_speed=50) == 20


def test_auto_mode_does_not_flap_around_the_critical_threshold():
    state = fan_logic.SimpleFanState()
    assert not state.should_use_auto(60, critical_temp_c=65)
    assert state.should_use_auto(65, critical_temp_c=65)
    assert state.should_use_auto(64, critical_temp_c=65)   # still inside hysteresis
    assert not state.should_use_auto(62, critical_temp_c=65)


def test_fan_speeds_are_clamped_to_safe_range():
    assert fan_logic.clamp_speed(0) == 5
    assert fan_logic.clamp_speed(3) == 5
    assert fan_logic.clamp_speed(20) == 20
    assert fan_logic.clamp_speed(150) == 100
    assert fan_logic.clamp_speed("nonsense") == 5


def test_validate_fan_config_corrects_inverted_thresholds():
    result = fan_logic.validate_fan_config(70, 60, 20, 50)
    assert result["critical"] == 71
    assert result["warnings"]


def test_validate_fan_config_corrects_high_below_base():
    result = fan_logic.validate_fan_config(45, 65, 60, 40)
    assert result["base"] == 60
    assert result["high"] == 60
    assert result["warnings"]


def test_validate_fan_config_clamps_speeds():
    result = fan_logic.validate_fan_config(45, 65, 0, 150)
    assert result["base"] == 5
    assert result["high"] == 100
    assert result["warnings"] == []
